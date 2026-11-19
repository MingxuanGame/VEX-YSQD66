#pragma once

#include <cstdint>
#include <span>

namespace auto_route {

// 英寸，原点为观众视角左下角，X 向右、Y 向远端；航向为北零、顺时针为正。
struct Point {
    double x;
    double y;
};

struct Pose {
    Point position;
    double heading_deg;
};

enum class Action { drive, turn, dock, wait };

struct Step {
    Action action;
    std::uint32_t limit_ms;
    unsigned checkpoint;
    int max_power = 0;
    bool reverse = false;
    double heading_deg = 0.0;
    Point control1{};
    Point control2{};
    Point end{};
};

struct DriveConfig {
    double travel_in_per_motor_rev;
    double heading_p;
    double heading_i;
    double heading_d;
    double speed_p;
    double speed_i;
    double speed_d;
    double static_power;
    double velocity_gain;
    double acceleration_gain;
    double left_trim;
    double cruise_in_s;
    double acceleration_in_s2;
    double lookahead_in;
    double position_tolerance_in;
    double heading_tolerance_deg;
    double tracking_limit_in;
};

struct Config {
    Pose start;
    DriveConfig drive;
};

// 停止、切换模式或禁用时停车。
void stop();
// 仅在 autonomous() 中运行。任一动作超时、传感器异常、越出追踪误差或模式切换即停车。
void run(const Config &config, std::span<const Step> steps);

} // namespace auto_route
