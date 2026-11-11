#include "vf_control/mock/mock_robot.hpp"

namespace vf_control {

void MockRobot::set_state(const RobotState& state) {
    const std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
}

RobotState MockRobot::state() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void MockRobot::accept_candidate(const ControlOutput& output) {
    const std::lock_guard<std::mutex> lock(mutex_);
    last_candidate_ = output;
}

std::optional<ControlOutput> MockRobot::last_candidate() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return last_candidate_;
}

}  // namespace vf_control
