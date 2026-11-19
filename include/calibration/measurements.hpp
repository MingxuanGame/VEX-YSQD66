#pragma once

#include <array>
#include <cstddef>
#include <span>

namespace calibration {

enum class Test {
    Travel,
    StaticPower,
    Steady,
    Accelerate,
    HeadingPid,
    SpeedPid,
    Stop,
    Tracking,
    Lookahead,
    Radius,
    Count
};
enum class TrialKind { None, Travel, Breakaway, Steady, Accelerate, Turn, StopAtDistance };
enum class Remember {
    None,
    Travel,
    StaticPower,
    VelocityGain,
    PositionTolerance,
    CruiseSpeed,
    Count
};

struct Field {
    const char *label;
    double min;
    double max;
    double initial; // NaN 表示等候实测值。
    Remember remember = Remember::None;
};

struct Recipe {
    const char *title;
    std::array<const char *, 4> guide;
    TrialKind trial;
    std::span<const Field> before;
    std::span<const Field> after;
};

struct TrialConfig {
    TrialKind kind = TrialKind::None;
    double travel_per_rev = 0; // in / motor rev
    int power = 0;             // PROS move(), [-127, 127]
    double target_in = 0;
};

struct TrialResult {
    std::array<double, 4> delta_deg{};
    double yaw_deg = 0;
    int breakaway_power = 0;
    double left_in_s = 0;
    double right_in_s = 0;
    double accel_start_in_s = 0;
    double accel_end_in_s = 0;
    double accel_time_s = 0;
    double travel_in = 0;
};

struct Value {
    const char *key;
    double value;
    const char *unit;
    Remember remember = Remember::None;
};

struct Result {
    std::array<Value, 5> values{};
    std::size_t count = 0;
    const char *error = nullptr;
};

// 返回单个有效测量项目的输入字段与屏幕短说明。
const Recipe &recipe(Test test);

// 解析一项练习试车的前置输入。输入不在范围内时返回 None。
TrialConfig trial_config(Test test, std::span<const double> before);

// 将试车采样和手工输入转换为 Override Auto Studio 的字段值，超出网页范围时报告错误。
Result calculate(Test test, std::span<const double> before, std::span<const double> after,
                 const TrialResult &measured);

} // namespace calibration
