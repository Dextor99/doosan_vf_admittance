#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace vf_control {

using MonotonicTime = std::chrono::steady_clock::time_point;
using Vec3 = std::array<double, 3>;
using Vec6 = std::array<double, 6>;
using QuaternionWxyz = std::array<double, 4>;

enum class FrameId : std::uint8_t {
    Unknown,
    Base,
    Flange,
    Sensor,
    Tcp,
};

enum class ReferencePoint : std::uint8_t {
    Unknown,
    BaseOrigin,
    FlangeOrigin,
    SensorOrigin,
    TcpOrigin,
};

struct Pose {
    Vec3 position_m{};
    QuaternionWxyz orientation_wxyz{1.0, 0.0, 0.0, 0.0};
};

struct RobotState {
    Vec6 joints_rad{};
    Pose base_to_tcp{};
    Vec6 twist_base_tcp{};  // m/s followed by rad/s
    MonotonicTime sample_time{};
    bool connected{false};
    bool motion_allowed{false};
    bool valid{false};
};

struct WrenchSample {
    Vec3 force_n{};
    Vec3 torque_nm{};
    MonotonicTime sample_time{};
    FrameId frame{FrameId::Unknown};
    ReferencePoint reference_point{ReferencePoint::Unknown};
    bool compensated{false};
    bool valid{false};
};

struct PoseReference {
    Pose pose{};
    Vec6 twist{};  // m/s followed by rad/s
    FrameId frame{FrameId::Base};
};

struct ControlInput {
    RobotState robot{};
    WrenchSample wrench{};
    PoseReference reference{};
    double dt_seconds{0.0};
};

struct ControlOutput {
    PoseReference candidate{};
    bool valid{false};
    std::string diagnostic{};
};

}  // namespace vf_control
