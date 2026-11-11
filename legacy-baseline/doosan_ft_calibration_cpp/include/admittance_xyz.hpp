#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace admittance {

using Vec3 = std::array<double, 3>;

struct XYZParameters {
    double mass_kg = 1.0;
    double damping_ns_m = 12.0;
    double force_deadband_n = 0.25;
    double filter_seconds = 0.08;
    // These are resultant XYZ limits, not independent per-axis limits.
    double max_speed_m_s = 0.030;
    double max_acceleration_m_s2 = 0.150;
};

struct XYZOutput {
    Vec3 target_m{};
    Vec3 velocity_m_s{};
    Vec3 filtered_force_n{};
};

class FreeXYZ {
public:
    explicit FreeXYZ(XYZParameters p = {}) : p_(p) {
        if (!(p_.mass_kg > 0 && p_.damping_ns_m > 0 &&
              p_.force_deadband_n >= 0 && p_.filter_seconds > 0 &&
              p_.max_speed_m_s > 0 && p_.max_acceleration_m_s2 > 0))
            throw std::invalid_argument("Invalid XYZ admittance parameters");
    }

    void reset(const Vec3& measured_m) {
        for (double value : measured_m)
            if (!std::isfinite(value)) throw std::invalid_argument("Invalid initial XYZ");
        target_ = measured_m;
        velocity_ = {};
        filtered_ = {};
        ready_ = true;
    }

    XYZOutput update(const Vec3& force_n, double dt_seconds) {
        if (!ready_ || !std::isfinite(dt_seconds) || dt_seconds <= 0 || dt_seconds > 0.1)
            throw std::invalid_argument("Invalid XYZ admittance input or cycle time");
        for (double value : force_n)
            if (!std::isfinite(value)) throw std::invalid_argument("Invalid XYZ force input");

        const double alpha = dt_seconds / (p_.filter_seconds + dt_seconds);
        Vec3 active_force{};
        bool any_active_force = false;
        for (std::size_t i = 0; i < 3; ++i) {
            filtered_[i] += alpha * (force_n[i] - filtered_[i]);
            if (filtered_[i] > p_.force_deadband_n)
                active_force[i] = filtered_[i] - p_.force_deadband_n;
            else if (filtered_[i] < -p_.force_deadband_n)
                active_force[i] = filtered_[i] + p_.force_deadband_n;
            any_active_force = any_active_force || active_force[i] != 0;
        }

        Vec3 acceleration{};
        for (std::size_t i = 0; i < 3; ++i)
            acceleration[i] = (active_force[i] - p_.damping_ns_m * velocity_[i]) / p_.mass_kg;
        limit_norm(acceleration, p_.max_acceleration_m_s2);

        for (std::size_t i = 0; i < 3; ++i)
            velocity_[i] += acceleration[i] * dt_seconds;
        limit_norm(velocity_, p_.max_speed_m_s);
        if (!any_active_force) {
            for (double& value : velocity_)
                if (std::abs(value) < 0.0001) value = 0;
        }
        for (std::size_t i = 0; i < 3; ++i)
            target_[i] += velocity_[i] * dt_seconds;

        return {target_, velocity_, filtered_};
    }

private:
    static void limit_norm(Vec3& value, double limit) {
        const double norm = std::sqrt(value[0] * value[0] + value[1] * value[1] +
                                      value[2] * value[2]);
        if (norm > limit) {
            const double scale = limit / norm;
            for (double& component : value) component *= scale;
        }
    }

    XYZParameters p_;
    Vec3 target_{};
    Vec3 velocity_{};
    Vec3 filtered_{};
    bool ready_ = false;
};

}  // namespace admittance
