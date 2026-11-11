// Y round-trip trajectory with XYZ disturbance recovery admittance.
#include "admittance_xyz_return.hpp"
#include "ft_acquisition.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using namespace std::chrono_literals;
constexpr auto kPeriod = 20ms;
constexpr auto kMaxSensorAge = 100ms;
constexpr auto kMaxRobotAge = 100ms;
constexpr double kDefaultForceLimitN = 20.0;
constexpr double kPathDistanceMm = 200.0;
constexpr double kPathSpeedMmS = 20.0;
constexpr double kPathAccelerationMmS2 = 50.0;

std::atomic<int> control_access{-1}, state{-1}, safety{SAFETY_STATE_LAST};
std::atomic<unsigned> alarms{0};
std::mutex alarm_mutex;
LOG_ALARM last_alarm{};
bool have_last_alarm = false;
void on_access(MONITORING_ACCESS_CONTROL value) { control_access.store(static_cast<int>(value)); }
void on_state(ROBOT_STATE value) { state.store(static_cast<int>(value)); }
void on_safety(SAFETY_STATE value) { safety.store(static_cast<int>(value)); }
void on_alarm(LPLOG_ALARM value) {
    if (!value || value->_iLevel < LOG_LEVEL_SYSWARN) return;
    {
        std::lock_guard<std::mutex> lock(alarm_mutex);
        last_alarm = *value;
        have_last_alarm = true;
    }
    alarms.fetch_add(1);
}
std::string last_alarm_description() {
    std::lock_guard<std::mutex> lock(alarm_mutex);
    if (!have_last_alarm) return "alarm details unavailable";
    std::ostringstream out;
    out << "level=" << static_cast<int>(last_alarm._iLevel)
        << ", group=" << static_cast<int>(last_alarm._iGroup)
        << ", index=" << last_alarm._iIndex;
    for (int i = 0; i < 3; ++i) {
        const char* begin = last_alarm._szParam[i];
        const char* end = std::find(begin, begin + MAX_STRING_SIZE, '\0');
        if (end != begin) out << ", param" << i + 1 << '=' << std::string(begin, end);
    }
    return out.str();
}

struct SensorSample {
    ftcal::Wrench raw{};
    Clock::time_point at{};
    unsigned long long sequence = 0;
};

class SensorPoller {
public:
    explicit SensorPoller(const std::string& device) {
        std::string error;
        if (!serial_.connect(device, error)) throw std::runtime_error(error);
        worker_ = std::thread([this] { loop(); });
    }
    ~SensorPoller() {
        done_.store(true);
        if (worker_.joinable()) worker_.join();
    }
    SensorPoller(const SensorPoller&) = delete;
    SensorPoller& operator=(const SensorPoller&) = delete;
    SensorSample latest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_.empty()) throw std::runtime_error("XJC serial: " + error_);
        return sample_;
    }
private:
    void loop() {
        while (!done_.load() && !stop_requested) {
            const auto start = Clock::now();
            std::array<uint8_t, 29> frame{};
            std::string error;
            if (!read_frame(serial_.fd(), frame, error)) {
                if (!done_.load() && !stop_requested) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    error_ = error;
                }
                return;
            }
            std::array<float, 6> values{};
            if (!decode_values(frame, values)) {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = "Non-finite sensor reading";
                return;
            }
            SensorSample next;
            next.at = start + (Clock::now() - start) / 2;
            for (int i = 0; i < 6; ++i) next.raw[i] = values[i];
            {
                std::lock_guard<std::mutex> lock(mutex_);
                next.sequence = sample_.sequence + 1;
                sample_ = next;
            }
            std::this_thread::sleep_until(start + kPeriod);
        }
    }
    SerialPort serial_;
    mutable std::mutex mutex_;
    SensorSample sample_{};
    std::string error_;
    std::atomic<bool> done_{false};
    std::thread worker_;
};

struct RobotConnection {
    DRAFramework::CDRFLEx& robot;
    bool open = false;
    ~RobotConnection() { if (open) robot.close_connection(); }
};

struct JointWindow {
    std::array<double, 6> low{}, high{};
};
JointWindow read_joint_window(DRAFramework::CDRFLEx& robot) {
    const auto* config = robot.get_safety_configuration();
    if (!config) throw std::runtime_error("Cannot read joint safety configuration");
    JointWindow window;
    for (int i = 0; i < 6; ++i) {
        window.low[i] = std::max(config->_tJointRange._Normal._fMinRange[i],
                                 config->_tJointRange._Reduced._fMinRange[i]) + 2.0;
        window.high[i] = std::min(config->_tJointRange._Normal._fMaxRange[i],
                                  config->_tJointRange._Reduced._fMaxRange[i]) - 2.0;
        if (!std::isfinite(window.low[i]) || !std::isfinite(window.high[i]) ||
            window.low[i] >= window.high[i])
            throw std::runtime_error("Invalid joint safety range");
    }
    return window;
}
void require_joint_window(const PoseState& pose, const JointWindow& window) {
    for (int i = 0; i < 6; ++i)
        if (pose.joints[i] < window.low[i] || pose.joints[i] > window.high[i])
            throw std::runtime_error("Joint J" + std::to_string(i+1) + " approached a limit");
}

PoseState fresh_robot() {
    const auto p = snapshot();
    require_fresh_pose(p);
    if (Clock::now() - p.received_at > kMaxRobotAge)
        throw std::runtime_error("Robot feedback older than 100 ms");
    return p;
}
SensorSample fresh_sensor(const SensorPoller& poller) {
    const auto s = poller.latest();
    if (s.sequence == 0 || Clock::now() - s.at > kMaxSensorAge)
        throw std::runtime_error("XJC reading older than 100 ms");
    return s;
}
ftcal::Vec3 rotate_force_to_base(const ftcal::Quat& q, const ftcal::Wrench& w) {
    const ftcal::Vec3 v{w[0], w[1], w[2]};
    const ftcal::Vec3 xyz{q[0], q[1], q[2]};
    const auto c1 = ftcal::cross(xyz, v);
    const auto c2 = ftcal::cross(xyz, c1);
    return {v[0] + 2*q[3]*c1[0] + 2*c2[0],
            v[1] + 2*q[3]*c1[1] + 2*c2[1],
            v[2] + 2*q[3]*c1[2] + 2*c2[2]};
}
ftcal::Vec3 base_force(const PoseState& robot, const SensorSample& sensor,
                      const ftcal::Compensator& compensator) {
    // The monitoring quaternion is flange->base, combined with q_FS in ft_acquisition.
    const auto compensated = compensator.compensate(sensor.raw, robot.q);
    return rotate_force_to_base(robot.q, compensated);
}
double force_norm(const ftcal::Vec3& force) {
    return std::sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]);
}
struct RoundTripY {
    double distance_mm = kPathDistanceMm;
    double speed_mm_s = kPathSpeedMmS;
    double acceleration_mm_s2 = kPathAccelerationMmS2;
    double acceleration_time_s = speed_mm_s / acceleration_mm_s2;
    double acceleration_distance_mm = 0.5 * acceleration_mm_s2 *
                                      acceleration_time_s * acceleration_time_s;
    double cruise_time_s = (distance_mm - 2.0 * acceleration_distance_mm) / speed_mm_s;
    double one_way_time_s = 2.0 * acceleration_time_s + cruise_time_s;
    double total_time_s = 2.0 * one_way_time_s;

    double one_way_position(double t) const {
        if (t <= 0) return 0;
        if (t < acceleration_time_s)
            return 0.5 * acceleration_mm_s2 * t * t;
        if (t < acceleration_time_s + cruise_time_s)
            return acceleration_distance_mm + speed_mm_s * (t - acceleration_time_s);
        if (t < one_way_time_s) {
            const double remaining = one_way_time_s - t;
            return distance_mm - 0.5 * acceleration_mm_s2 * remaining * remaining;
        }
        return distance_mm;
    }

    double round_trip_position(double t) const {
        if (t < 0) t = 0;
        t = std::fmod(t, total_time_s);
        if (t <= one_way_time_s) return one_way_position(t);
        if (t < total_time_s) return distance_mm - one_way_position(t - one_way_time_s);
        return 0;
    }
};
bool valid_sign(const char* value) {
    return std::string(value) == "1" || std::string(value) == "-1";
}
void wait_for_post_setup_samples(const SensorPoller& sensor) {
    const auto after_setup = Clock::now();
    const auto deadline = after_setup + 2s;
    while (Clock::now() < deadline && !stop_requested && !robot_disconnected.load()) {
        const auto p = snapshot();
        const auto s = sensor.latest();
        if (p.received && p.received_at > after_setup && s.sequence && s.at > after_setup) {
            fresh_robot();
            fresh_sensor(sensor);
            return;
        }
        std::this_thread::sleep_for(10ms);
    }
    throw std::runtime_error("No new robot and XJC samples after motion-mode setup");
}
void require_healthy(unsigned alarm_baseline) {
    if (stop_requested)
        throw std::runtime_error("Interrupted by SIGINT/SIGTERM");
    if (robot_disconnected.load())
        throw std::runtime_error("Robot TCP connection disconnected");
    const int access = control_access.load();
    if (access != MONITORING_ACCESS_CONTROL_GRANT)
        throw std::runtime_error("Robot control GRANT lost (access=" + std::to_string(access) + ")");
    if (alarms.load() != alarm_baseline)
        throw std::runtime_error("New Doosan SDK alarm/warning: " + last_alarm_description());
    const int current = state.load();
    if (current != STATE_STANDBY && current != STATE_MOVING)
        throw std::runtime_error("Robot left STANDBY/MOVING");
    const int safe = safety.load();
    if (safe != SAFETY_STATE_SW_RUN && safe != SAFETY_STATE_CW_RUN)
        throw std::runtime_error("Motion safety state is no longer SW_RUN/CW_RUN");
}
void prepare_control(DRAFramework::CDRFLEx& robot, unsigned alarm_baseline) {
    if (control_access.load() != MONITORING_ACCESS_CONTROL_GRANT) {
        if (!robot.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST))
            throw std::runtime_error("Control request failed");
        std::cout << "Waiting up to 60 s for pendant control GRANT...\n" << std::flush;
        const auto deadline = Clock::now() + 60s;
        while (Clock::now() < deadline && !stop_requested && !robot_disconnected.load()) {
            if (control_access.load() == MONITORING_ACCESS_CONTROL_GRANT) break;
            if (control_access.load() == MONITORING_ACCESS_CONTROL_DENY)
                throw std::runtime_error("Control request denied");
            std::this_thread::sleep_for(50ms);
        }
    }
    if (control_access.load() != MONITORING_ACCESS_CONTROL_GRANT)
        throw std::runtime_error("No control GRANT");
    if (alarms.load() != alarm_baseline) throw std::runtime_error("New alarm before servo-on");
    if (robot.get_robot_state() == STATE_SAFE_OFF) {
        const int safe = safety.load();
        if (safe != SAFETY_STATE_JH_SOS && safe != SAFETY_STATE_VD_STO)
            throw std::runtime_error("Unexpected SAFE_OFF safety state");
        if (!robot.set_robot_control(CONTROL_SERVO_ON))
            throw std::runtime_error("Servo-on rejected");
    }
    const auto deadline = Clock::now() + 15s;
    while (Clock::now() < deadline && !stop_requested) {
        if (control_access.load() != MONITORING_ACCESS_CONTROL_GRANT ||
            alarms.load() != alarm_baseline || robot_disconnected.load())
            throw std::runtime_error("Control lost or alarm during servo preparation");
        if (robot.get_robot_state() == STATE_STANDBY && state.load() == STATE_STANDBY) break;
        std::this_thread::sleep_for(50ms);
    }
    if (robot.get_robot_state() != STATE_STANDBY || state.load() != STATE_STANDBY)
        throw std::runtime_error("Servo did not reach STANDBY");
    if (robot.get_robot_mode() != ROBOT_MODE_AUTONOMOUS &&
        !robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS))
        throw std::runtime_error("AUTONOMOUS mode rejected");
    if (robot.get_robot_mode() != ROBOT_MODE_AUTONOMOUS ||
        !robot.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE))
        throw std::runtime_error("AUTONOMOUS / EVENT_MOVE rejected");
    const auto safety_deadline = Clock::now() + 5s;
    while (Clock::now() < safety_deadline) {
        const int safe = safety.load();
        if (safe == SAFETY_STATE_SW_RUN || safe == SAFETY_STATE_CW_RUN) break;
        std::this_thread::sleep_for(50ms);
    }
    require_healthy(alarm_baseline);
    if (robot.check_motion() != 0) throw std::runtime_error("Robot already moving");
}

bool terminal_stop() {
    pollfd input{STDIN_FILENO, static_cast<short>(POLLIN | POLLHUP), 0};
    const int ready = poll(&input, 1, 0);
    return ready > 0 && (input.revents & (POLLIN | POLLHUP));
}

void wait_for_data(const SensorPoller& sensor) {
    const auto deadline = Clock::now() + 5s;
    while (Clock::now() < deadline && !stop_requested) {
        fresh_robot();
        if (sensor.latest().sequence != 0) { fresh_sensor(sensor); return; }
        std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("No XJC sample within 5 s");
}

void admittance_usage(const char* name) {
    std::cout << "Usage:\n  " << name << " <ip> <serial-device> <model.txt> --monitor\n"
              << "  " << name << " <ip> <serial-device> <model.txt> --run <sign-x> <sign-y> <sign-z>"
                 " [--force-limit-n <1..200>]\n"
              << "Repeating Y-axis 200 mm out-and-back trajectory with XYZ disturbance recovery;"
                 " each sign must be 1 or -1."
                 " --monitor is read-only.\n";
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") { admittance_usage(argv[0]); return 0; }
    const bool monitor = argc == 5 && std::string(argv[4]) == "--monitor";
    const bool run = (argc == 8 || argc == 10) && std::string(argv[4]) == "--run" &&
                     valid_sign(argv[5]) && valid_sign(argv[6]) && valid_sign(argv[7]);
    if (!monitor && !run) { admittance_usage(argv[0]); return 2; }
    in_addr parsed{};
    if (inet_pton(AF_INET, argv[1], &parsed) != 1) {
        std::cerr << "Invalid controller IPv4 address\n"; return 2;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    try {
        double force_limit_n = kDefaultForceLimitN;
        if (run && argc == 10) {
            if (std::string(argv[8]) != "--force-limit-n")
                throw std::runtime_error("Expected --force-limit-n <1..200>");
            force_limit_n = number(argv[9]);
            if (force_limit_n < 1.0 || force_limit_n > 200.0)
                throw std::runtime_error("Force limit must be 1..200 N");
        }
        const std::string device = argv[2], model = argv[3];
        mounting = {0,0,0,1};
        check_model_metadata(model, device, mounting);
        const ftcal::Compensator compensator(ftcal::load_parameters(model));
        DRAFramework::CDRFLEx robot;
        robot.set_on_monitoring_data_ex(on_robot_data_ex);
        robot.set_on_disconnected(on_robot_disconnected);
        robot.set_on_monitoring_access_control(on_access);
        robot.set_on_monitoring_state(on_state);
        robot.set_on_monitoring_safety_state(on_safety);
        robot.set_on_log_alarm(on_alarm);
        if (!robot.open_connection(argv[1], 12345)) throw std::runtime_error("Robot connection failed");
        RobotConnection connection{robot, true};
        if (!robot.setup_monitoring_version(1)) throw std::runtime_error("Monitoring setup failed");
        const auto deadline = Clock::now() + 5s;
        while (!snapshot().received && Clock::now() < deadline && !stop_requested)
            std::this_thread::sleep_for(20ms);
        fresh_robot();
        SensorPoller sensor(device);
        wait_for_data(sensor);
        std::cout << "Connected. Current TCP [mm]: X=" << fresh_robot().tcp[0]
                  << " Y=" << fresh_robot().tcp[1] << " Z=" << fresh_robot().tcp[2]
                  << "; state=" << state.load() << "; safety=" << safety.load()
                  << "; access=" << control_access.load() << '\n';
        std::cout << "XJC model matched. Hold only a handle beyond the sensor; verify force sign with --monitor.\n";
        if (monitor) {
            std::cout << "Read-only base XYZ force [N]; Ctrl+C to exit.\n";
            auto next = Clock::now();
            while (!stop_requested && !robot_disconnected.load()) {
                next += 200ms;
                const auto p = fresh_robot();
                const auto s = fresh_sensor(sensor);
                const auto force = base_force(p, s, compensator);
                std::cout << "Fbase=[" << force[0] << ',' << force[1] << ',' << force[2]
                          << "] N, TCP=[" << p.tcp[0] << ',' << p.tcp[1] << ',' << p.tcp[2]
                          << "] mm, sensor_seq=" << s.sequence
                          << ", sample_skew_ms="
                          << std::abs(std::chrono::duration<double, std::milli>(p.received_at-s.at).count())
                          << '\n';
                std::this_thread::sleep_until(next);
            }
            return 0;
        }
        if (robot.get_robot_system() != ROBOT_SYSTEM_REAL)
            throw std::runtime_error("Robot system must be REAL");
        const auto joint_window = read_joint_window(robot);
        const auto initial_state = robot.get_robot_state();
        if (initial_state != STATE_STANDBY && initial_state != STATE_SAFE_OFF)
            throw std::runtime_error("Robot must be STANDBY or ordinary SAFE_OFF");
        const std::array<int, 3> force_sign{
            std::string(argv[5]) == "1" ? 1 : -1,
            std::string(argv[6]) == "1" ? 1 : -1,
            std::string(argv[7]) == "1" ? 1 : -1};
        const auto initial = fresh_robot();
        require_joint_window(initial, joint_window);
        const auto initial_force = base_force(initial, fresh_sensor(sensor), compensator);
        if (force_norm(initial_force) > 0.5)
            throw std::runtime_error("Release the handle before arming: force norm > 0.5 N");
        std::cout << "Repeating Y round-trip trajectory with XYZ disturbance recovery. Force signs=[" << force_sign[0]
                  << ',' << force_sign[1] << ',' << force_sign[2]
                  << "]; nominal path: +Y 200 mm then -Y 200 mm"
                  << " (400 mm per cycle), path speed=20 mm/s"
                  << ", recovery M=1.0 kg, D=12 N*s/m, K=36 N/m, deadband=0.25 N"
                  << ", recovery max speed=30 mm/s, acceleration=150 mm/s^2"
                  << ", servo follow caps=50 mm/s, 1000 mm/s^2"
                  << ", force-norm limit=" << force_limit_n << " N.\n"
                  << (force_limit_n > 30.0
                      ? "High software force limit selected; controller safety limits still apply.\n" : "")
                  << "The nominal Y path repeats from the current pose to Y+200 mm and back."
                  << " External force offsets XYZ; after release, the TCP returns to the moving path.\n"
                  << "Verify the full 400 mm cycle and disturbance workspace; orientation stays fixed."
                  << " Keep the independent emergency stop ready.\n"
                  << "Press Enter to request control and start; other input cancels: " << std::flush;
        std::string answer;
        if (!read_command(answer) || !answer.empty()) { std::cout << "Cancelled.\n"; return 0; }
        const unsigned alarm_baseline = alarms.load();
        prepare_control(robot, alarm_baseline);
        wait_for_post_setup_samples(sensor);
        const auto armed = fresh_robot();
        require_joint_window(armed, joint_window);
        for (int i = 0; i < 3; ++i)
            if (std::abs(armed.tcp[i] - initial.tcp[i]) > 1.0)
                throw std::runtime_error("TCP moved while preparing control");
        for (int i = 0; i < 6; ++i)
            if (std::abs(armed.joints[i] - initial.joints[i]) > 0.2)
                throw std::runtime_error("Joints moved while preparing control");
        const auto armed_force = base_force(armed, fresh_sensor(sensor), compensator);
        if (force_norm(armed_force) > 0.5)
            throw std::runtime_error("Release the handle before servol starts");
        const admittance::ReturnVec3 force_zero{armed_force[0], armed_force[1], armed_force[2]};
        admittance::ReturnXYZ controller;
        controller.reset({0.0, 0.0, 0.0});
        const RoundTripY path;
        float target[6]{};
        for (int i = 0; i < 6; ++i) target[i] = static_cast<float>(initial.tcp[i]);
        // The path and recovery target stay low speed; leave SDK headroom and
        // avoid the very-low translation-acceleration warning.
        float velocity[2]{50.0f, 5.0f};
        float acceleration[2]{1000.0f, 20.0f};
        std::cout << "Armed XYZ-force zero reference=[" << force_zero[0] << ','
                  << force_zero[1] << ',' << force_zero[2] << "] N.\n"
                  << "Running. Press Enter or Ctrl+C to stop.\n" << std::flush;
        bool commanded = false;
        try {
            auto previous = Clock::now(), next = previous;
            const auto trajectory_start = previous;
            unsigned log_divider = 0;
            while (!stop_requested && !terminal_stop()) {
                next += kPeriod;
                require_healthy(alarm_baseline);
                const auto p = fresh_robot();
                const auto s = fresh_sensor(sensor);
                require_joint_window(p, joint_window);
                const auto q0 = ftcal::quaternion_from_zyz_degrees(initial.tcp[3], initial.tcp[4], initial.tcp[5]);
                const auto q = ftcal::quaternion_from_zyz_degrees(p.tcp[3], p.tcp[4], p.tcp[5]);
                if (ftcal::quaternion_distance_degrees(q0, q) > 2.0)
                    throw std::runtime_error("TCP orientation changed by >2 deg");
                const auto measured_force = base_force(p, s, compensator);
                const double measured_norm = force_norm(measured_force);
                if (measured_norm > force_limit_n)
                    throw std::runtime_error("External force norm=" + std::to_string(measured_norm) +
                        " N exceeds " + std::to_string(force_limit_n) + " N; base F=[" +
                        std::to_string(measured_force[0]) + ", " +
                        std::to_string(measured_force[1]) + ", " +
                        std::to_string(measured_force[2]) + "] N");
                const admittance::ReturnVec3 force{
                    force_sign[0] * (measured_force[0] - force_zero[0]),
                    force_sign[1] * (measured_force[1] - force_zero[1]),
                    force_sign[2] * (measured_force[2] - force_zero[2])};
                const auto now = Clock::now();
                const double dt = std::chrono::duration<double>(now - previous).count();
                previous = now;
                const auto output = controller.update(force, dt);
                const double elapsed_s = std::chrono::duration<double>(now - trajectory_start).count();
                const double path_y_mm = path.round_trip_position(elapsed_s);
                target[0] = static_cast<float>(initial.tcp[0] + output.target_m[0] * 1000.0);
                target[1] = static_cast<float>(initial.tcp[1] + path_y_mm + output.target_m[1] * 1000.0);
                target[2] = static_cast<float>(initial.tcp[2] + output.target_m[2] * 1000.0);
                const double tracking_error_mm = std::sqrt(
                    std::pow(p.tcp[0] - target[0], 2) +
                    std::pow(p.tcp[1] - target[1], 2) +
                    std::pow(p.tcp[2] - target[2], 2));
                // A failed/ambiguous SDK reply may still have started motion.
                commanded = true;
                if (!robot.servol(target, velocity, acceleration, 0.0f))
                    throw std::runtime_error("Controller rejected servol");
                if (++log_divider % 10 == 0)
                    std::cout << "Fbase=[" << measured_force[0] << ',' << measured_force[1]
                              << ',' << measured_force[2] << "] N, |F|=" << measured_norm
                              << " N, Fcontrol=[" << force[0] << ',' << force[1] << ','
                              << force[2] << "] N, actual=[" << p.tcp[0] << ',' << p.tcp[1]
                              << ',' << p.tcp[2] << "] mm, target=[" << target[0] << ','
                              << target[1] << ',' << target[2] << "] mm, tracking="
                              << tracking_error_mm << " mm, pathY=" << path_y_mm
                              << " mm\n" << std::flush;
                std::this_thread::sleep_until(next);
                if (Clock::now() > next + 40ms)
                    throw std::runtime_error("Control cycle fell behind by >40 ms");
            }
            if (commanded) robot.stop(STOP_TYPE_SLOW);
            std::cout << "Repeating trajectory stopped by operator.\n";
        } catch (...) {
            if (commanded && !robot_disconnected.load()) robot.stop(STOP_TYPE_QUICK);
            throw;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return stop_requested ? 130 : 1;
    }
}
