#include "chassis.h"

#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <cmath>
#include <cstdint>

namespace chassis {
namespace {

constexpr std::uint8_t kImuPort = 5;                    // IMU 端口
constexpr float kTrackWidthIn = 11.58F;                 // 轮距（in）
constexpr float kWheelDiameterIn = 2.75F;               // 轮径（in）
constexpr float kDrivetrainRpm = 600.0F * 6.0F / 10.0F; // 蓝色齿轮组 600 rpm，减速比 6:10
constexpr float kHorizontalDrift = 8.0F;                // 水平漂移补偿值

// 车头（夹爪）
// 2-1
// 3-4
pros::MotorGroup _g_left({-2, -1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({3, 4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Imu _g_imu(kImuPort);

lemlib::Drivetrain _g_drivetrain(&_g_left, &_g_right, kTrackWidthIn, kWheelDiameterIn,
                                 kDrivetrainRpm, kHorizontalDrift);
lemlib::OdomSensors _g_sensors(nullptr, nullptr, nullptr, nullptr, &_g_imu);

// TODO: 未测定
// see https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html
lemlib::ControllerSettings _g_lateral_settings(10.0F,  // kP
                                               0.0F,   // kI
                                               3.0F,   // kD
                                               3.0F,   // 积分生效的误差范围
                                               1.0F,   // 	高精度结束条件的误差范围 (in)
                                               100.0F, // 保持在小误差范围内的时间 (ms)
                                               3.0F,   // 	低精度结束条件的误差范围 (in)
                                               500.0F, // 保持在大误差范围内的时间 (ms)
                                               20.0F   // 限制电机控制输出的变化速度
);

lemlib::ControllerSettings _g_angular_settings(2.0F,   // kP
                                               0.0F,   // kI
                                               10.0F,  // kD
                                               3.0F,   // 积分生效的误差范围
                                               1.0F,   // 高精度结束条件的误差范围 (deg)
                                               100.0F, // 保持在小误差范围内的时间 (ms)
                                               3.0F,   // 低精度结束条件的误差范围 (deg)
                                               500.0F, // 保持在大误差范围内的时间 (ms)
                                               0.0F    // 限制电机控制输出的变化速度
);

lemlib::ExpoDriveCurve _g_throttle_curve(3.0F,  // 死区
                                         10.0F, // 最小输出
                                         1.019F // 指数曲线增益
);
lemlib::ExpoDriveCurve _g_steer_curve(3.0F,  // 死区
                                      10.0F, // 最小输出
                                      1.019F // 指数曲线增益
);

lemlib::Chassis _g_chassis(_g_drivetrain, _g_lateral_settings, _g_angular_settings, _g_sensors,
                           &_g_throttle_curve, &_g_steer_curve);

} // namespace

void initialize() {
    stop();
    _g_chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    _g_chassis.calibrate();
    _g_chassis.setPose(0.0F, 0.0F, 0.0F);
}

void stop() {
    _g_chassis.cancelAllMotions();
    _g_left.brake();
    _g_right.brake();
}

void set_brake_mode(pros::motor_brake_mode_e mode) {
    _g_chassis.setBrakeMode(mode);
}

void arcade(int throttle, int turn) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return;
    }
    _g_chassis.arcade(throttle, turn);
}

lemlib::Chassis &controller() {
    return _g_chassis;
}

lemlib::Pose pose() {
    return _g_chassis.getPose();
}

std::optional<double> imu_rotation_deg() {
    const double rotation_deg = _g_imu.get_rotation();
    return std::isfinite(rotation_deg) ? std::optional{rotation_deg} : std::nullopt;
}

} // namespace chassis
