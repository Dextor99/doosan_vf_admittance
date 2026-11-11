// One-shot Doosan joint-space MoveJ test. This executable does not read the
// force sensor or run the calibration sequence.
#include <DRFLEx.h>

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using Joints = std::array<double, 6>;
constexpr char kDefaultRobotIp[] = "192.168.123.156";
constexpr float kVelocityDegPerSec = 2.0f;
constexpr float kAccelerationDegPerSec2 = 4.0f;
constexpr double kPositionToleranceDeg = 0.2;
constexpr auto kFreshness = std::chrono::milliseconds(250);
volatile std::sig_atomic_t stop_requested = 0;

std::atomic<int> robot_state{STATE_LAST};
std::atomic<int> safety_state{SAFETY_STATE_LAST};
std::atomic<int> access_state{-1};
std::atomic<bool> disconnected{false};
std::atomic<std::uint64_t> warning_events{0};
std::atomic<int> last_alarm_level{-1};
std::atomic<int> last_alarm_group{-1};
std::atomic<int> last_alarm_index{-1};

struct Snapshot {
    Joints joints{};
    Clock::time_point received{};
    std::uint64_t sequence = 0;
    bool valid = false;
};
std::mutex snapshot_mutex;
Snapshot latest;

void on_signal(int) { stop_requested = 1; }

void on_monitoring_data(const LPMONITORING_DATA_EX data) {
    if (!data) return;
    Joints joints{};
    for (int i = 0; i < 6; ++i) {
        joints[i] = data->_tCtrl._tJoint._fActualPos[i];
        if (!std::isfinite(joints[i])) return;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex);
    latest.joints = joints;
    latest.received = Clock::now();
    latest.valid = true;
    ++latest.sequence;
}

void on_state(const ROBOT_STATE state) { robot_state.store(static_cast<int>(state)); }
void on_safety_state(const SAFETY_STATE state) { safety_state.store(static_cast<int>(state)); }
void on_access(const MONITORING_ACCESS_CONTROL state) { access_state.store(static_cast<int>(state)); }
void on_disconnected() { disconnected.store(true); }
void on_alarm(LPLOG_ALARM alarm) {
    if (!alarm) return;
    last_alarm_level.store(static_cast<int>(alarm->_iLevel));
    last_alarm_group.store(static_cast<int>(alarm->_iGroup));
    last_alarm_index.store(static_cast<int>(alarm->_iIndex));
    if (alarm->_iLevel >= LOG_LEVEL_SYSWARN) warning_events.fetch_add(1);
}

Snapshot get_snapshot() {
    std::lock_guard<std::mutex> lock(snapshot_mutex);
    return latest;
}

bool fresh(const Snapshot& snapshot) {
    return snapshot.valid && Clock::now() - snapshot.received <= kFreshness;
}

double parse_number(const char* text) {
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !std::isfinite(value))
        throw std::runtime_error(std::string("Invalid angle: ") + text);
    return value;
}

std::string joints_text(const Joints& joints) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i) out << ", ";
        out << joints[i];
    }
    return out.str();
}

const char* robot_state_name(int state) {
    switch (state) {
        case STATE_INITIALIZING: return "INITIALIZING";
        case STATE_STANDBY: return "STANDBY";
        case STATE_MOVING: return "MOVING";
        case STATE_SAFE_OFF: return "SAFE_OFF";
        case STATE_TEACHING: return "TEACHING";
        case STATE_SAFE_STOP: return "SAFE_STOP";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case STATE_HOMMING: return "HOMMING";
        case STATE_RECOVERY: return "RECOVERY";
        case STATE_SAFE_STOP2: return "SAFE_STOP2";
        case STATE_SAFE_OFF2: return "SAFE_OFF2";
        case STATE_NOT_READY: return "NOT_READY";
        default: return "UNKNOWN";
    }
}

const char* safety_state_name(int state) {
    switch (state) {
        case SAFETY_STATE_VD_STO: return "VD_STO";
        case SAFETY_STATE_VD_SOS: return "VD_SOS";
        case SAFETY_STATE_JH_SOS: return "JH_SOS";
        case SAFETY_STATE_JH_MOVE: return "JH_MOVE";
        case SAFETY_STATE_SW_SOS: return "SW_SOS";
        case SAFETY_STATE_SW_RUN: return "SW_RUN";
        case SAFETY_STATE_CW_SOS: return "CW_SOS";
        case SAFETY_STATE_CW_RUN: return "CW_RUN";
        case SAFETY_STATE_CM_RUN: return "CM_RUN";
        case SAFETY_STATE_AM_RUN: return "AM_RUN";
        default: return "UNKNOWN";
    }
}

Snapshot wait_for_monitoring() {
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (!stop_requested && !disconnected.load() && Clock::now() < deadline) {
        const auto snapshot = get_snapshot();
        if (fresh(snapshot)) return snapshot;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (disconnected.load()) throw std::runtime_error("Robot disconnected while waiting for monitoring data");
    if (stop_requested) throw std::runtime_error("Interrupted");
    throw std::runtime_error("No fresh joint monitoring data received within 5 seconds");
}

void print_diagnostics(DRAFramework::CDRFLEx& robot) {
    const auto snapshot = get_snapshot();
    const auto mode = robot.get_robot_mode();
    const auto state = robot.get_robot_state();
    std::cerr << "Robot mode=" << static_cast<int>(mode)
              << ", state=" << robot_state_name(static_cast<int>(state))
              << " (" << static_cast<int>(state) << ")"
              << ", safety=" << safety_state_name(safety_state.load())
              << ", access=" << access_state.load()
              << ", disconnected=" << disconnected.load() << '\n';
    if (fresh(snapshot)) std::cerr << "Joint feedback [deg]: " << joints_text(snapshot.joints) << '\n';
    if (last_alarm_index.load() >= 0)
        std::cerr << "Last callback alarm: level=" << last_alarm_level.load()
                  << ", group=" << last_alarm_group.load()
                  << ", index=" << last_alarm_index.load() << '\n';
}

void validate_target(const Joints& current, const Joints& target, DRAFramework::CDRFLEx& robot) {
    const auto* config = robot.get_safety_configuration();
    if (!config) throw std::runtime_error("Cannot read controller joint limits");
    double largest_delta = 0.0;
    int largest_joint = -1;
    for (int i = 0; i < 6; ++i) {
        const double low = std::max(config->_tJointRange._Normal._fMinRange[i],
                                    config->_tJointRange._Reduced._fMinRange[i]);
        const double high = std::min(config->_tJointRange._Normal._fMaxRange[i],
                                     config->_tJointRange._Reduced._fMaxRange[i]);
        const double speed = std::min(config->_tJointRange._Normal._fMaxVelocity[i],
                                      config->_tJointRange._Reduced._fMaxVelocity[i]);
        if (!std::isfinite(low) || !std::isfinite(high) || low >= high ||
            !std::isfinite(speed) || speed < kVelocityDegPerSec)
            throw std::runtime_error("Invalid controller limits for J" + std::to_string(i + 1));
        if (target[i] < low + 0.2 || target[i] > high - 0.2)
            throw std::runtime_error("Target is outside the controller limit margin for J" + std::to_string(i + 1));
        const double delta = std::abs(target[i] - current[i]);
        if (delta > 360.0)
            throw std::runtime_error("Target changes J" + std::to_string(i + 1) + " by more than 360 degrees");
        if (delta > largest_delta) {
            largest_delta = delta;
            largest_joint = i;
        }
    }
    std::cout << "Largest joint change: " << largest_delta << " deg";
    if (largest_joint >= 0) std::cout << " (J" << largest_joint + 1 << ')';
    std::cout << ".\n";
}

void request_control(DRAFramework::CDRFLEx& robot) {
    int state = access_state.load();
    if (state == MONITORING_ACCESS_CONTROL_GRANT) {
        std::cout << "Control right is already granted.\n";
        return;
    }
    if (!robot.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST))
        throw std::runtime_error("Could not request normal control right");
    std::cout << "Control right requested. Approve it on the teach pendant; waiting up to 60 seconds.\n" << std::flush;
    const auto deadline = Clock::now() + std::chrono::seconds(60);
    while (!stop_requested && !disconnected.load() && Clock::now() < deadline) {
        state = access_state.load();
        if (state == MONITORING_ACCESS_CONTROL_GRANT) return;
        if (state == MONITORING_ACCESS_CONTROL_DENY) throw std::runtime_error("Control request was denied");
        if (state == MONITORING_ACCESS_CONTROL_LOSS) throw std::runtime_error("Control right was lost");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("No control-right grant before timeout or interruption");
}

void require_ready(DRAFramework::CDRFLEx& robot, std::uint64_t warning_baseline) {
    if (stop_requested || disconnected.load() || access_state.load() != MONITORING_ACCESS_CONTROL_GRANT)
        throw std::runtime_error("Interrupted, disconnected, or control right lost");
    if (warning_events.load() != warning_baseline) {
        print_diagnostics(robot);
        throw std::runtime_error("A new controller warning/alarm arrived; inspect the teach pendant");
    }
}

void ensure_servo_standby(DRAFramework::CDRFLEx& robot, bool safe_off_confirmed,
                          std::uint64_t warning_baseline) {
    auto state = robot.get_robot_state();
    if (state == STATE_STANDBY) return;
    if (state != STATE_SAFE_OFF || !safe_off_confirmed)
        throw std::runtime_error("Only an operator-confirmed ordinary SAFE_OFF can receive one servo-on request");
    require_ready(robot, warning_baseline);
    if (robot.get_robot_state() != STATE_SAFE_OFF)
        throw std::runtime_error("Robot state changed before servo-on; no command was sent");
    std::cout << "Sending CONTROL_SERVO_ON once for confirmed SAFE_OFF.\n" << std::flush;
    if (!robot.set_robot_control(CONTROL_SERVO_ON)) {
        print_diagnostics(robot);
        throw std::runtime_error("Controller rejected servo-on; inspect the teach pendant");
    }
    const auto deadline = Clock::now() + std::chrono::seconds(15);
    while (!stop_requested && Clock::now() < deadline) {
        require_ready(robot, warning_baseline);
        state = robot.get_robot_state();
        if (state == STATE_STANDBY && robot_state.load() == STATE_STANDBY) return;
        if (state != STATE_SAFE_OFF && state != STATE_STANDBY) {
            print_diagnostics(robot);
            throw std::runtime_error("Robot entered an unexpected state during servo-on");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    print_diagnostics(robot);
    throw std::runtime_error("Servo-on did not reach STANDBY within 15 seconds; no retry was sent");
}

void configure_motion_mode(DRAFramework::CDRFLEx& robot, std::uint64_t warning_baseline) {
    if (robot.get_robot_state() != STATE_STANDBY || robot_state.load() != STATE_STANDBY)
        throw std::runtime_error("Robot must be in STANDBY before motion-mode setup");
    if (robot.get_robot_mode() != ROBOT_MODE_AUTONOMOUS &&
        !robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
        print_diagnostics(robot);
        throw std::runtime_error("Controller rejected AUTONOMOUS mode; no motion was sent");
    }
    if (robot.get_robot_mode() != ROBOT_MODE_AUTONOMOUS)
        throw std::runtime_error("AUTONOMOUS mode was not confirmed");
    require_ready(robot, warning_baseline);
    if (!robot.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE)) {
        print_diagnostics(robot);
        throw std::runtime_error("Controller rejected AUTONOMOUS / EVENT_MOVE; no motion was sent");
    }
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (!stop_requested && Clock::now() < deadline) {
        require_ready(robot, warning_baseline);
        const int safety = safety_state.load();
        if (safety == SAFETY_STATE_SW_RUN || safety == SAFETY_STATE_CW_RUN) {
            std::cout << "Mode ready: AUTONOMOUS, safety=" << safety_state_name(safety) << ".\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    print_diagnostics(robot);
    throw std::runtime_error("Safety state did not enter SW_RUN/CW_RUN; no motion was sent");
}

bool at_target(const Joints& actual, const Joints& target) {
    for (int i = 0; i < 6; ++i)
        if (std::abs(actual[i] - target[i]) > kPositionToleranceDeg) return false;
    return true;
}

bool wait_for_target(DRAFramework::CDRFLEx& robot, const Joints& target,
                     std::uint64_t warning_baseline) {
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    int stable_frames = 0;
    while (!stop_requested && Clock::now() < deadline) {
        require_ready(robot, warning_baseline);
        const auto snapshot = get_snapshot();
        const auto state = robot.get_robot_state();
        if (state == STATE_SAFE_OFF || state == STATE_SAFE_STOP || state == STATE_EMERGENCY_STOP ||
            state == STATE_SAFE_OFF2 || state == STATE_SAFE_STOP2) {
            print_diagnostics(robot);
            return false;
        }
        if (fresh(snapshot) && state == STATE_STANDBY && robot_state.load() == STATE_STANDBY &&
            robot.check_motion() == 0 && at_target(snapshot.joints, target)) {
            if (++stable_frames >= 3) return true;
        } else stable_frames = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void stop_if_still_moving(DRAFramework::CDRFLEx& robot) {
    if (disconnected.load() || robot.get_robot_state() != STATE_MOVING) return;
    const bool accepted = robot.stop(STOP_TYPE_QUICK);
    std::cerr << "MoveJ is still active; QUICK stop request "
              << (accepted ? "was accepted" : "failed")
              << ". Check the robot and use the independent emergency stop if needed.\n";
}

void usage(const char* program) {
    std::cout << "Usage: " << program << " J1 J2 J3 J4 J5 J6 [robot-ip]\n"
              << "All six targets are absolute joint angles in degrees (not XYZ/RX/RY/RZ).\n"
              << "Default robot IP: " << kDefaultRobotIp
              << "; speed: 2 deg/s; acceleration: 4 deg/s^2.\n";
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        usage(argv[0]);
        return 0;
    }
    if (argc != 7 && argc != 8) {
        usage(argv[0]);
        return 2;
    }
    try {
        Joints target{};
        for (int i = 0; i < 6; ++i) target[i] = parse_number(argv[i + 1]);
        const std::string ip = argc == 8 ? argv[7] : kDefaultRobotIp;
        in_addr parsed_ip{};
        if (inet_pton(AF_INET, ip.c_str(), &parsed_ip) != 1)
            throw std::runtime_error("Invalid robot IPv4 address: " + ip);
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);

        DRAFramework::CDRFLEx robot;
        robot.set_on_monitoring_data_ex(on_monitoring_data);
        robot.set_on_monitoring_state(on_state);
        robot.set_on_monitoring_safety_state(on_safety_state);
        robot.set_on_monitoring_access_control(on_access);
        robot.set_on_disconnected(on_disconnected);
        robot.set_on_log_alarm(on_alarm);

        std::cout << "Connecting to Doosan " << ip << ":12345...\n" << std::flush;
        if (!robot.open_connection(ip.c_str(), 12345))
            throw std::runtime_error("Cannot connect to robot");
        struct CloseConnection {
            DRAFramework::CDRFLEx& robot;
            ~CloseConnection() { robot.close_connection(); }
        } close_connection{robot};
        if (!robot.setup_monitoring_version(1))
            throw std::runtime_error("Could not enable monitoring version 1");

        const auto current_snapshot = wait_for_monitoring();
        const auto system = robot.get_robot_system();
        if (system != ROBOT_SYSTEM_REAL)
            throw std::runtime_error("Robot system is not REAL; refusing motion");
        const auto initial_state = robot.get_robot_state();
        robot_state.store(static_cast<int>(initial_state));
        if (initial_state != STATE_STANDBY && initial_state != STATE_SAFE_OFF)
            throw std::runtime_error(std::string("Robot must be STANDBY or ordinary SAFE_OFF; current state is ") +
                                     robot_state_name(static_cast<int>(initial_state)));

        SYSTEM_VERSION version{};
        robot.get_system_version(&version);
        std::cout << "Controller " << version._szController << ", library "
                  << robot.get_library_version() << '\n'
                  << "Mode=" << (robot.get_robot_mode() == ROBOT_MODE_AUTONOMOUS ? "AUTONOMOUS" : "MANUAL")
                  << ", state=" << robot_state_name(static_cast<int>(initial_state))
                  << ", safety=" << safety_state_name(safety_state.load()) << '\n'
                  << "Current joints [deg]: " << joints_text(current_snapshot.joints) << '\n'
                  << "Target joints [deg]:  " << joints_text(target) << '\n'
                  << "Velocity=" << kVelocityDegPerSec << " deg/s; acceleration="
                  << kAccelerationDegPerSec2 << " deg/s^2.\n";
        const auto* last_alarm = robot.get_last_alarm();
        if (last_alarm && (last_alarm->_iLevel != 0 || last_alarm->_iGroup != 0 || last_alarm->_iIndex != 0))
            std::cout << "SDK last alarm (may be historical): level=" << static_cast<unsigned>(last_alarm->_iLevel)
                      << ", group=" << static_cast<unsigned>(last_alarm->_iGroup)
                      << ", index=" << last_alarm->_iIndex << '\n';
        validate_target(current_snapshot.joints, target, robot);
        if (at_target(current_snapshot.joints, target)) {
            std::cout << "Already at target within 0.2 deg; no control or motion command sent.\n";
            return 0;
        }

        const auto warning_baseline = warning_events.load();
        if (initial_state == STATE_SAFE_OFF)
            std::cout << "SAFE_OFF: press Enter only after confirming the pendant has no active alarm and servo-on is safe.\n";
        std::cout << "Review the complete path, cables and clearance. Keep the independent emergency stop ready.\n"
                  << "Press Enter to request control, enable servo if needed, set AUTONOMOUS mode and make one MoveJ; "
                  << "type anything else to cancel: " << std::flush;
        std::string command;
        if (!std::getline(std::cin, command) || !command.empty()) {
            std::cout << "Cancelled; no control, servo or motion command sent.\n";
            return 0;
        }

        request_control(robot);
        ensure_servo_standby(robot, initial_state == STATE_SAFE_OFF, warning_baseline);
        configure_motion_mode(robot, warning_baseline);
        if (!robot.change_operation_speed(50.0f))
            throw std::runtime_error("Controller rejected the 50% operation speed override");
        require_ready(robot, warning_baseline);
        if (robot.get_robot_state() != STATE_STANDBY || robot_state.load() != STATE_STANDBY)
            throw std::runtime_error("Robot is not in confirmed STANDBY immediately before MoveJ");
        const auto before_move = wait_for_monitoring();
        if (!at_target(before_move.joints, current_snapshot.joints))
            throw std::runtime_error("Robot moved after preflight; target command was not sent");

        float destination[6];
        for (int i = 0; i < 6; ++i) destination[i] = static_cast<float>(target[i]);
        std::cout << "Sending one synchronous MoveJ. Use the independent emergency stop if needed.\n" << std::flush;
        const bool accepted = robot.movej(destination, kVelocityDegPerSec, kAccelerationDegPerSec2,
                                          0.0f, MOVE_MODE_ABSOLUTE);
        if (!accepted) {
            stop_if_still_moving(robot);
            print_diagnostics(robot);
            throw std::runtime_error("Controller rejected or interrupted MoveJ");
        }
        if (!wait_for_target(robot, target, warning_baseline)) {
            stop_if_still_moving(robot);
            print_diagnostics(robot);
            throw std::runtime_error("MoveJ ended without stable feedback at the target");
        }
        std::cout << "MoveJ complete. Final joints [deg]: " << joints_text(get_snapshot().joints) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
