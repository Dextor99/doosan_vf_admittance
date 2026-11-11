#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace admittance {

struct Parameters {
    double mass_kg = 1.0;
    double damping_ns_m = 12.0;
    double force_deadband_n = 0.25;
    double filter_seconds = 0.08;
    double max_speed_m_s = 0.030;
    double max_acceleration_m_s2 = 0.150;
};

struct Output {
    double target_x_m = 0;
    double velocity_m_s = 0;
    double filtered_force_n = 0;
};

class FreeX {
public:
    explicit FreeX(Parameters p = {}) : p_(p) {
        if (!(p_.mass_kg > 0 && p_.damping_ns_m > 0 && p_.force_deadband_n >= 0 &&
              p_.filter_seconds > 0 && p_.max_speed_m_s > 0 &&
              p_.max_acceleration_m_s2 > 0))
            throw std::invalid_argument("Invalid admittance parameters");
    }

    void reset(double measured_x_m) {
        if (!std::isfinite(measured_x_m)) throw std::invalid_argument("Invalid initial X");
        target_ = measured_x_m;
        velocity_ = filtered_ = 0;
        ready_ = true;
    }

    Output update(double force_n, double dt_seconds) {
        if (!ready_ || !std::isfinite(force_n) || !std::isfinite(dt_seconds) ||
            dt_seconds <= 0 || dt_seconds > 0.1)
            throw std::invalid_argument("Invalid admittance input or cycle time");
        const double alpha = dt_seconds / (p_.filter_seconds + dt_seconds);
        filtered_ += alpha * (force_n - filtered_);
        const double active_force = filtered_ > p_.force_deadband_n
            ? filtered_ - p_.force_deadband_n
            : filtered_ < -p_.force_deadband_n
                ? filtered_ + p_.force_deadband_n : 0.0;
        double acceleration = (active_force - p_.damping_ns_m * velocity_) / p_.mass_kg;
        acceleration = std::clamp(acceleration, -p_.max_acceleration_m_s2,
                                  p_.max_acceleration_m_s2);
        velocity_ = std::clamp(velocity_ + acceleration * dt_seconds,
                               -p_.max_speed_m_s, p_.max_speed_m_s);
        if (active_force == 0 && std::abs(velocity_) < 0.0001) velocity_ = 0;
        target_ += velocity_ * dt_seconds;
        return {target_, velocity_, filtered_};
    }

private:
    Parameters p_;
    double target_ = 0, velocity_ = 0, filtered_ = 0;
    bool ready_ = false;
};

}  // namespace admittance
