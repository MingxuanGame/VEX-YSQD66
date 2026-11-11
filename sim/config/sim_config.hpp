#pragma once
#include "sim/hardware/sim_motor.hpp"

namespace sim {

/**
 * @brief 抬升/级联的仿真参数。
 *
 * 这些值只影响仿真，不进入固件。拿到实机后，把电机端口、级联级数、
 * 每级高度、最大速度替换成实测值即可。
 */
struct LiftSimConfig {
    int motor_port = 6;
    Gearset gearset = Gearset::Red;
    int cascade_stages = 3;
    double stage_height_m = 0.30;
    double max_speed_mps = 0.22;
    double position_kp = 8.0;
    double hold_current_ma = 350.0;
    double move_current_ma = 1200.0;
};

struct ClampSimConfig {
    int servo_port = 7;
    double open_angle_deg = 20.0;
    double closed_angle_deg = 150.0;
    double slew_deg_per_sec = 240.0;
    double hold_current_ma = 60.0;
};

struct PushSimConfig {
    int motor_port = 8;
    Gearset gearset = Gearset::Green;
    double travel_time_sec = 0.35;
    double move_current_ma = 500.0;
};

/**
 * @brief 机构仿真的统一配置。
 *
 * 当前默认值是按“三级级联 + 舵机夹爪 + 推板”写的占位配置。
 * 实机尺寸/端口确定后，只改这里，不需要动 SimMechanism。
 */
struct SimConfig {
    LiftSimConfig lift;
    ClampSimConfig clamp;
    PushSimConfig push;
};

inline SimConfig defaultSimConfig() {
    return SimConfig{};
}

} // namespace sim
