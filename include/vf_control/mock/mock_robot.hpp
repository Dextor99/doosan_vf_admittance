#pragma once

#include <mutex>
#include <optional>

#include "vf_control/core/types.hpp"

namespace vf_control {

// In-memory backend for offline development. It has no network or robot SDK calls.
class MockRobot {
public:
    void set_state(const RobotState& state);
    [[nodiscard]] RobotState state() const;

    void accept_candidate(const ControlOutput& output);
    [[nodiscard]] std::optional<ControlOutput> last_candidate() const;

private:
    mutable std::mutex mutex_;
    RobotState state_{};
    std::optional<ControlOutput> last_candidate_{};
};

}  // namespace vf_control
