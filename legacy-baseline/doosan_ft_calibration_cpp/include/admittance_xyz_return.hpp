#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace admittance {

using ReturnVec3 = std::array<double, 3>;

struct XYZReturnParameters {
    double mass_kg = 1.0;
    double damping_ns_m = 12.0;
    double stiffness_n_m = 36.0;
    double force_deadband_n = 0.25;
    double filter_seconds = 0.08;
    double max_speed_m_s = 0.030;
    double max_acceleration_m_s2 = 0.150;
};

struct XYZReturnOutput {
    ReturnVec3 target_m{};
    ReturnVec3 velocity_m_s{};
    ReturnVec3 filtered_force_n{};
};

class ReturnXYZ {
public:
    explicit ReturnXYZ(XYZReturnParameters p = {}) : p_(p) {
        if (!(p_.mass_kg > 0 && p_.damping_ns_m > 0 && p_.stiffness_n_m > 0 &&
              p_.force_deadband_n >= 0 && p_.filter_seconds > 0 &&
              p_.max_speed_m_s > 0 && p_.max_acceleration_m_s2 > 0))
            throw std::invalid_argument("Invalid XYZ return-admittance parameters");
    }

    void reset(const ReturnVec3& measured_m) {
        for (double value : measured_m)
            if (!std::isfinite(value)) throw std::invalid_argument("Invalid return point");
        origin_ = target_ = measured_m;
        velocity_ = {};
        filtered_ = {};
        ready_ = true;
    }

    XYZReturnOutput update(const ReturnVec3& force_n, double dt_seconds) {
        if (!ready_ || !std::isfinite(dt_seconds) || dt_seconds <= 0 || dt_seconds > 0.1)
            throw std::invalid_argument("Invalid XYZ return-admittance input or cycle time");
        for (double value : force_n)
            if (!std::isfinite(value)) throw std::invalid_argument("Invalid XYZ force input");

        const double alpha = dt_seconds / (p_.filter_seconds + dt_seconds);
        ReturnVec3 active_force{};
        bool any_active_force = false;
        for (std::size_t i = 0; i < 3; ++i) {
            filtered_[i] += alpha * (force_n[i] - filtered_[i]);
            if (filtered_[i] > p_.force_deadband_n)
                active_force[i] = filtered_[i] - p_.force_deadband_n;
            else if (filtered_[i] < -p_.force_deadband_n)
                active_force[i] = filtered_[i] + p_.force_deadband_n;
            any_active_force = any_active_force || active_force[i] != 0;
        }

        ReturnVec3 acceleration{};
        for (std::size_t i = 0; i < 3; ++i) {
            const double displacement = target_[i] - origin_[i];
            acceleration[i] = (active_force[i] - p_.damping_ns_m * velocity_[i] -
                               p_.stiffness_n_m * displacement) / p_.mass_kg;
        }
        limit_norm(acceleration, p_.max_acceleration_m_s2);
        for (std::size_t i = 0; i < 3; ++i)
            velocity_[i] += acceleration[i] * dt_seconds;
        limit_norm(velocity_, p_.max_speed_m_s);
        for (std::size_t i = 0; i < 3; ++i)
            target_[i] += velocity_[i] * dt_seconds;

        if (!any_active_force && norm(velocity_) < 0.0001 &&
            norm(subtract(target_, origin_)) < 0.0001) {
            target_ = origin_;
            velocity_ = {};
        }
        return {target_, velocity_, filtered_};
    }

private:
    static double norm(const ReturnVec3& value) {
        return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                         value[2] * value[2]);
    }
    static ReturnVec3 subtract(const ReturnVec3& a, const ReturnVec3& b) {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
    static void limit_norm(ReturnVec3& value, double limit) {
        const double magnitude = norm(value);
        if (magnitude > limit) {
            const double scale = limit / magnitude;
            for (double& component : value) component *= scale;
        }
    }

    XYZReturnParameters p_;
    ReturnVec3 origin_{};
    ReturnVec3 target_{};
    ReturnVec3 velocity_{};
    ReturnVec3 filtered_{};
    bool ready_ = false;
};

}  // namespace admittance
