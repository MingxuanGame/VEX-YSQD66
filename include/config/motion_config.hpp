#pragma once

/**
 * @file motion_config.hpp
 * @brief 运动控制与硬件配置中心。
 *        所有与物理尺寸、齿轮比、安全限幅相关的常量全部集中在此。
 *        换车、换轮子、换齿轮箱时，只需修改这一个文件，所有底层计算自动生效。
 */

namespace cfg {

// ==================== 数学常数 ====================
constexpr double kPi = 3.141592653589793;

// ==================== 底盘几何参数 ====================
constexpr double kWheelDiameterInch = 3.25; // 轮径（英寸），需实测橡胶胎外径
constexpr double kWheelCircumferenceInch = kWheelDiameterInch * kPi;
constexpr double kTrackWidthInch = 12.0; // 轮距（左右轮中心距离，用于转向积分计算）

// 齿轮比：电机转多少圈，轮子转一圈。
// 如果电机直连轮子填 1.0；如果中间有齿轮组（比如电机转3圈轮子转1圈）填 3.0。
constexpr double kDriveGearRatio = 1.0;
constexpr double kEncoderTicksPerRev = 360.0; // V5 智能电机转一圈固定返回 360 tick

// 换算比例：1 个 tick 对应多少英寸。
// 见 --> vxmath/units.hpp (ticksToInches / inchesToTicks)
// 公式：轮子周长 / (电机一圈 tick 数 × 齿轮比)
constexpr double kInchPerTick = kWheelCircumferenceInch / (kEncoderTicksPerRev * kDriveGearRatio);

// ==================== 电机与硬件协议限制 ====================
// 见 --> vxmath/units.hpp (clampMotorOutput)
// 见 --> vxmath/filters.hpp (expoCurve)
constexpr int kMotorMaxOutput = 127;         // V5 正向满速输出
constexpr int kMotorReverseMaxOutput = -127; // V5 反向满速输出（防止被误认为“最小值”）

// 舵机安全限幅：防止角度给过头导致齿轮卡死烧毁
// 见 --> vxmath/units.hpp (clampServoAngle)
constexpr int kServoMinAngle = 0;
constexpr int kServoMaxAngle = 180;

// 百分比基数：用于在百分比（0~100）和电机输出（0~127）之间做映射
// 见 --> vxmath/units.hpp (percentToMotor / motorToPercent)
constexpr double kPercentBase = 100.0;

// ==================== 速度限制（防打滑/防翻车） ====================
// 你们用的是蓝齿轮（600 RPM），如果不限制最高速，起步容易打滑，手动也容易翻车
constexpr int kMaxDriveSpeed = 100; // 见 --> control/drive_controller.cpp
constexpr int kMaxTurnSpeed = 80;   // 见 --> control/drive_controller.cpp

// ==================== PID 初始参数 ====================
// 注：这里只是初始值。比赛现场我们会通过遥控器调参系统实时覆盖它们。
// 见 --> motion/drive_control.cpp (构造函数)
// 见 --> tuning/param_apply.cpp (从 SD 卡读盘后覆盖)

// 直线距离环 PID（控制走多远）
constexpr double kDriveKp = 1.5;
constexpr double kDriveKi = 0.0;
constexpr double kDriveKd = 0.3;

// 原地转向环 PID（控制转多少度）
constexpr double kTurnKp = 2.0;
constexpr double kTurnKi = 0.0;
constexpr double kTurnKd = 0.5;

// 直线航向保持环 PID（控制直行时不跑偏）
constexpr double kHeadingKp = 0.8;
constexpr double kHeadingKi = 0.0;
constexpr double kHeadingKd = 0.0;

// ==================== 运动控制层参数 ====================
constexpr double kMaxDriveRpm = 600.0; // 蓝齿轮空载转速
constexpr double kMaxTurnDps = 360.0;  // 原地转向最大角速度（度/秒）
constexpr double kDriveAccelInchesPerSec2 = 60.0;
constexpr double kTurnAccelDegPerSec2 = 720.0;
// 位置式梯形曲线在起点速度为 0，直接用会“原地不动”；加一点前视距离给初始速度。
constexpr double kDriveProfileLookaheadInch = 0.5;
constexpr double kTurnProfileLookaheadDeg = 2.0;
constexpr int kControlLoopMs = 10;

// 速度环 PID（输出是 -127~127 的电压）
// 当前值由 vex_sim_motion 扫参得到：90° 转向 89.8~90.2°。
constexpr double kVelocityKp = 2.2;
constexpr double kVelocityKi = 0.0;
constexpr double kVelocityKd = 0.02;
constexpr double kIntegralLimit = 100.0; // 积分项硬限幅
constexpr int kOutputLimit = 127;        // 最终输出硬限幅

// 到位判定
constexpr double kPositionToleranceInch = 0.2;
constexpr double kAngleToleranceDeg = 0.2;

// 电池补偿
constexpr int kNominalBatteryMv = 12000;
constexpr double kMaxBatteryCompensation = 1.3;

} // namespace cfg
