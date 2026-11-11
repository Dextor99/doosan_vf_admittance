#pragma once
// Read-only Doosan monitoring + XJC Modbus acquisition. No robot motion or sensor tare.
#include <DRFLEx.h>
#include "ft_compensator.hpp"
#include "pose_math.hpp"
#include "xjc_serial.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>

namespace {
constexpr int kSamples = 30;
constexpr int kIntervalMs = 100;
constexpr double kMaxJointSpanDeg = 0.2;
constexpr double kMaxOrientationSpanDeg = 0.2;
constexpr auto kMaxPoseAge = std::chrono::milliseconds(250);

struct PoseState {
    std::array<double, 6> joints{};
    std::array<double, 6> tcp{};
    std::array<double, 6> min_joints{};
    std::array<double, 6> max_joints{};
    ftcal::Quat q{};
    ftcal::Quat first_q{};
    double max_rotation_deg = 0;
    Clock::time_point received_at{};
    std::uint64_t seq = 0;
    bool received = false;
    bool invalid = false;
    bool watching = false;
};
std::mutex pose_mutex;
PoseState pose;
ftcal::Quat mounting{};
std::atomic<bool> robot_disconnected{false};

void on_robot_disconnected() { robot_disconnected.store(true); }
void on_robot_data(const LPMONITORING_DATA data) {
    if (!data) return;
    std::lock_guard<std::mutex> lock(pose_mutex);
    try {
        std::array<double, 6> joints{};
        for (std::size_t i = 0; i < joints.size(); ++i) {
            joints[i] = data->_tCtrl._tJoint._fActualPos[i];
            if (!std::isfinite(joints[i])) throw std::runtime_error("Invalid joint data");
        }
        const auto& flange = data->_tCtrl._tTask._fActualPos[1];
        std::array<double, 6> tcp{};
        for (std::size_t i = 0; i < tcp.size(); ++i) {
            tcp[i] = data->_tCtrl._tTask._fActualPos[0][i];
            if (!std::isfinite(tcp[i])) throw std::runtime_error("Invalid TCP data");
        }
        const auto flange_q = ftcal::quaternion_from_zyz_degrees(flange[3], flange[4], flange[5]);
        const auto q = ftcal::normalize_quaternion(ftcal::quaternion_multiply(flange_q, mounting));
        if (pose.watching) {
            for (std::size_t i = 0; i < joints.size(); ++i) {
                pose.min_joints[i] = std::min(pose.min_joints[i], joints[i]);
                pose.max_joints[i] = std::max(pose.max_joints[i], joints[i]);
            }
            pose.max_rotation_deg = std::max(pose.max_rotation_deg,
                ftcal::quaternion_distance_degrees(pose.first_q, q));
        }
        pose.joints = joints;
        pose.tcp = tcp;
        pose.q = q;
        pose.received_at = Clock::now();
        pose.received = true;
        ++pose.seq;
    } catch (...) {
        pose.invalid = true;
    }
}
[[maybe_unused]] void on_robot_data_ex(const LPMONITORING_DATA_EX data) {
    if (!data) return;
    MONITORING_DATA base{};
    base._tCtrl._tJoint = data->_tCtrl._tJoint;
    base._tCtrl._tTask = data->_tCtrl._tTask;
    on_robot_data(&base);
}
PoseState snapshot() {
    std::lock_guard<std::mutex> lock(pose_mutex);
    return pose;
}
void require_fresh_pose(const PoseState& s) {
    if (robot_disconnected.load()) throw std::runtime_error("Robot disconnected");
    if (s.invalid) throw std::runtime_error("Invalid robot monitoring data");
    if (!s.received || Clock::now() - s.received_at > kMaxPoseAge)
        throw std::runtime_error("Robot monitoring pose is missing or stale (>250 ms)");
}
void begin_motion_check() {
    std::lock_guard<std::mutex> lock(pose_mutex);
    require_fresh_pose(pose);
    pose.first_q = pose.q;
    pose.min_joints = pose.joints;
    pose.max_joints = pose.joints;
    pose.max_rotation_deg = 0;
    pose.watching = true;
}
void end_motion_check() {
    std::lock_guard<std::mutex> lock(pose_mutex);
    pose.watching = false;
}
struct MotionWatch {
    MotionWatch() { begin_motion_check(); }
    ~MotionWatch() { end_motion_check(); }
};

struct Observation {
    ftcal::Quat q{};
    ftcal::Wrench wrench{};
    double unix_time = 0;
    double pose_age_ms = 0;
};
Observation read_observation(SerialPort& serial) {
    const auto start = Clock::now();
    std::array<uint8_t, 29> frame{};
    std::string error;
    if (!read_frame(serial.fd(), frame, error)) throw std::runtime_error("Sensor: " + error);
    const auto end = Clock::now();
    const auto s = snapshot();
    require_fresh_pose(s);
    // Host arrival time, not hardware synchronization; suitable only when stationary.
    const auto midpoint = start + (end - start) / 2;
    const auto diff = s.received_at > midpoint ? s.received_at - midpoint : midpoint - s.received_at;
    if (diff > kMaxPoseAge) throw std::runtime_error("Sensor/robot arrival time difference exceeds 250 ms");
    std::array<float, 6> values{};
    if (!decode_values(frame, values)) throw std::runtime_error("Sensor returned a non-finite value");
    Observation o;
    o.q = s.q;
    for (std::size_t i = 0; i < 6; ++i) o.wrench[i] = values[i];
    o.unix_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    o.pose_age_ms = std::chrono::duration<double, std::milli>(diff).count();
    return o;
}
void wait_interval(Clock::time_point start) {
    const auto until = start + std::chrono::milliseconds(kIntervalMs);
    while (!stop_requested && Clock::now() < until) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(until - Clock::now()).count();
        if (ms <= 0) break;
        poll(nullptr, 0, static_cast<int>(std::min<long long>(ms, 20)));
    }
}

bool read_command(std::string& command) {
    command.clear();
    while (!stop_requested) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 100);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) throw std::runtime_error(system_error("Cannot read command"));
        if (ready == 0) continue;
        if (input.revents & (POLLERR | POLLNVAL)) throw std::runtime_error("Command input failed");
        if (input.revents & (POLLIN | POLLHUP)) {
            char ch;
            const ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 0) return !command.empty();
            if (n < 0 && errno == EINTR) continue;
            if (n < 0) throw std::runtime_error(system_error("Cannot read command"));
            if (ch == '\n') return true;
            if (ch != '\r') command += ch;
            if (command.size() > 100) throw std::runtime_error("Command is too long");
        }
    }
    return false;
}

double number(const char* text) {
    std::size_t used = 0;
    const std::string str(text);
    const double v = std::stod(str, &used);
    if (used != str.size() || !std::isfinite(v)) throw std::runtime_error("Invalid number: " + str);
    return v;
}
const char* csv_header = "split,stamp_s,qx,qy,qz,qw,fx,fy,fz,mx,my,mz";
std::string metadata(const std::string& device, const ftcal::Quat& q) {
    std::ostringstream out;
    out << "xjc_ft_session_v1\nserial=" << device << "\npose_frame=flange\nq_flange_sensor=";
    out << std::setprecision(17) << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3];
    out << "\nmodbus=station1_fc04_start0_count12_be_float_115200_8n1\n";
    return out.str();
}
[[maybe_unused]] void check_model_metadata(const std::string& model, const std::string& device, const ftcal::Quat& q) {
    std::ifstream in(model + ".meta", std::ios::binary);
    if (!in) throw std::runtime_error("Model acquisition metadata missing; use a model calibrated from an ft_acquire session");
    const std::string existing((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (existing != metadata(device, q))
        throw std::runtime_error("Model metadata differs from the current device or sensor mounting");
}
void print_axes_in_base(const ftcal::Quat& q) {
    const double x=q[0], y=q[1], z=q[2], w=q[3];
    std::cout << std::fixed << std::setprecision(3)
              << "Sensor X in base: [" << 1-2*(y*y+z*z) << ", " << 2*(x*y+w*z) << ", " << 2*(x*z-w*y) << "]\n"
              << "Sensor Y in base: [" << 2*(x*y-w*z) << ", " << 1-2*(x*x+z*z) << ", " << 2*(y*z+w*x) << "]\n"
              << "Sensor Z in base: [" << 2*(x*z+w*y) << ", " << 2*(y*z-w*x) << ", " << 1-2*(x*x+y*y) << "]\n";
}
void prepare_csv(const std::string& file, const std::string& device, const ftcal::Quat& q) {
    namespace fs = std::filesystem;
    const std::string meta = file + ".meta";
    const bool have_csv = fs::exists(file), have_meta = fs::exists(meta);
    if (have_csv != have_meta) throw std::runtime_error("CSV and its .meta file must both exist or both be new");
    if (have_csv) {
        std::ifstream in(file, std::ios::binary), mi(meta, std::ios::binary);
        if (!in || !mi) throw std::runtime_error("Cannot read existing CSV or metadata");
        std::string hdr;
        std::getline(in, hdr);
        if (!hdr.empty() && hdr.back() == '\r') hdr.pop_back();
        if (hdr != csv_header) throw std::runtime_error("Existing CSV has the wrong header");
        in.seekg(-1, std::ios::end);
        char last = 0;
        in.get(last);
        if (last != '\n') throw std::runtime_error("CSV ends with an incomplete line; inspect it before appending");
        const std::string existing((std::istreambuf_iterator<char>(mi)), std::istreambuf_iterator<char>());
        if (existing != metadata(device, q)) throw std::runtime_error("CSV metadata differs from current device or mounting; use a new CSV");
        return;
    }
    // Creation is done before hardware capture; never overwrite existing files.
    std::ofstream m(meta), out(file);
    if (!m || !out) throw std::runtime_error("Cannot create CSV and metadata in the selected directory");
    m << metadata(device, q);
    out << csv_header << '\n';
    m.flush(); out.flush();
    if (!m || !out) throw std::runtime_error("Failed writing CSV or metadata header");
}
void capture_pose(SerialPort& serial, const std::string& csv, const std::string& split,
                  const std::function<void()>& guard = {}) {
    MotionWatch watch;
    if (guard) guard();
    const auto first = snapshot();
    std::array<double, 6> sum{}, sum_squares{};
    ftcal::Quat qsum{};
    double ts = 0, max_skew = 0;
    for (int i = 0; i < kSamples; ++i) {
        if (stop_requested) throw std::runtime_error("Capture interrupted; no row saved");
        if (guard) guard();
        const auto started = Clock::now();
        const Observation o = read_observation(serial);
        if (guard) guard();
        double dot = 0;
        for (int j = 0; j < 4; ++j) dot += first.q[j] * o.q[j];
        for (int j = 0; j < 4; ++j) qsum[j] += (dot < 0 ? -o.q[j] : o.q[j]);
        for (int j = 0; j < 6; ++j) { sum[j] += o.wrench[j]; sum_squares[j] += o.wrench[j] * o.wrench[j]; }
        ts += o.unix_time;
        max_skew = std::max(max_skew, o.pose_age_ms);
        wait_interval(started);
    }
    if (stop_requested) throw std::runtime_error("Capture interrupted; no row saved");
    if (guard) guard();
    const auto last = snapshot();
    require_fresh_pose(last);
    if (last.seq <= first.seq + 1) throw std::runtime_error("Insufficient new robot monitoring data; no row saved");
    for (int j = 0; j < 6; ++j) {
        if (last.max_joints[j] - last.min_joints[j] > kMaxJointSpanDeg)
            throw std::runtime_error("Joint motion >0.2 deg during capture; no row saved");
    }
    if (last.max_rotation_deg > kMaxOrientationSpanDeg)
        throw std::runtime_error("Flange orientation changed >0.2 deg during capture; no row saved");
    for (double& v : qsum) v /= kSamples;
    const auto q = ftcal::normalize_quaternion(qsum);
    std::ostringstream row;
    row << std::setprecision(17) << split << ',' << ts / kSamples;
    for (double v : q) row << ',' << v;
    for (double v : sum) row << ',' << v / kSamples;
    row << '\n';
    std::ofstream out(csv, std::ios::app);
    if (!out || !(out << row.str()) || !(out.flush())) throw std::runtime_error("Failed appending CSV row");
    std::cout << "Saved " << split << " mean of " << kSamples << " readings. Max host arrival difference="
              << max_skew << " ms\n" << "Wrench mean: ";
    for (double v : sum) std::cout << v / kSamples << ' ';
    std::cout << "\nWrench standard deviation: ";
    for (int j = 0; j < 6; ++j) {
        const double mean = sum[j] / kSamples;
        std::cout << std::sqrt(std::max(0.0, sum_squares[j] / kSamples - mean * mean)) << ' ';
    }
    std::cout << "\n";
}

[[maybe_unused]] void usage(const char* name) {
    std::cout << "Usage:\n  " << name << " record <new-or-session.csv> <robot-ip> <serial-device> <qx> <qy> <qz> <qw> --assume-xjc-format\n"
              << "  " << name << " monitor <model.txt> <robot-ip> <serial-device> <qx> <qy> <qz> <qw> --assume-xjc-format\n"
              << "Quaternion maps sensor frame to flange frame; aligned axes: 0 0 0 1.\n"
              << "XJC interpretation is provisional. Confirm axis order, units and calibration before relying on output.\n";
}
} // namespace

