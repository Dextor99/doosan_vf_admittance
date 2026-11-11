#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>

// Tracks an acknowledged move without treating an initial idle response as
// completion. The caller remains responsible for controller safety checks.
class MotionCompletion {
public:
    using Clock = std::chrono::steady_clock;
    using Joints = std::array<double, 6>;

    MotionCompletion(const Joints& initial, const Joints& target,
                     std::uint64_t initial_sequence, std::uint64_t initial_moving_events,
                     bool synchronous_completed)
        : initial_(initial), target_(target), initial_sequence_(initial_sequence),
          initial_moving_events_(initial_moving_events), started_(synchronous_completed) {}

    bool started() const { return started_; }

    bool observe(int motion_status, bool robot_moving, bool robot_standby,
                 std::uint64_t moving_events, std::uint64_t sequence,
                 const Joints& actual, Clock::time_point now) {
        if (motion_status == 2 || robot_moving || moving_events > initial_moving_events_)
            started_ = true;
        if (sequence > initial_sequence_) {
            for (std::size_t i = 0; i < actual.size(); ++i)
                if (std::abs(actual[i] - initial_[i]) > 0.02) started_ = true;
        }
        bool at_target = true;
        for (std::size_t i = 0; i < actual.size(); ++i)
            if (std::abs(actual[i] - target_[i]) > 0.2) at_target = false;
        if (!started_ || motion_status != 0 || !robot_standby || !at_target ||
            sequence <= initial_sequence_) {
            stable_ = false;
            return false;
        }
        if (!stable_) {
            stable_ = true; stable_since_ = now; last_sequence_ = sequence;
            min_ = max_ = actual; stable_frames_ = 1;
            return false;
        }
        if (sequence != last_sequence_) {
            last_sequence_ = sequence;
            bool too_much_motion = false;
            for (std::size_t i = 0; i < actual.size(); ++i) {
                min_[i] = std::min(min_[i], actual[i]);
                max_[i] = std::max(max_[i], actual[i]);
                if (max_[i] - min_[i] > 0.1) too_much_motion = true;
            }
            if (too_much_motion) {
                stable_since_ = now; min_ = max_ = actual; stable_frames_ = 1;
            } else {
                ++stable_frames_;
            }
        }
        return stable_frames_ >= 3 && now - stable_since_ >= std::chrono::milliseconds(500);
    }

private:
    Joints initial_{}, target_{}, min_{}, max_{};
    std::uint64_t initial_sequence_ = 0, initial_moving_events_ = 0, last_sequence_ = 0;
    std::size_t stable_frames_ = 0;
    Clock::time_point stable_since_{};
    bool started_ = false, stable_ = false;
};
