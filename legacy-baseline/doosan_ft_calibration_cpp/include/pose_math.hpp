#pragma once

#include "ft_compensator.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace ftcal {
// q_AB maps vectors from B to A; quaternion storage is (x,y,z,w).
inline Quat normalize_quaternion(Quat q) {
    double n = 0;
    for (double v : q) {
        if (!std::isfinite(v)) throw std::runtime_error("Non-finite quaternion");
        n += v * v;
    }
    n = std::sqrt(n);
    if (n < 0.98 || n > 1.02) throw std::runtime_error("Quaternion norm must be close to 1");
    for (double& v : q) v /= n;
    return q;
}
inline Quat quaternion_multiply(const Quat& a, const Quat& b) {
    return {a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
            a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
            a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
            a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]};
}
// Doosan task orientation: active Rz(A) * Ry(B) * Rz(C), degrees.
inline Quat quaternion_from_zyz_degrees(double a, double b, double c) {
    constexpr double rad = 3.14159265358979323846 / 180.0;
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c))
        throw std::runtime_error("Non-finite robot orientation");
    a *= rad / 2; b *= rad / 2; c *= rad / 2;
    const Quat za{0, 0, std::sin(a), std::cos(a)};
    const Quat yb{0, std::sin(b), 0, std::cos(b)};
    const Quat zc{0, 0, std::sin(c), std::cos(c)};
    return normalize_quaternion(quaternion_multiply(quaternion_multiply(za, yb), zc));
}
inline double quaternion_distance_degrees(const Quat& a, const Quat& b) {
    double dot = 0;
    for (std::size_t i = 0; i < 4; ++i) dot += a[i] * b[i];
    dot = std::min(1.0, std::abs(dot));
    return 2 * std::acos(dot) * 180.0 / 3.14159265358979323846;
}
}  // namespace ftcal
