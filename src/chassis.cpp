#include "chassis.h"

#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <cmath>
#include <cstdint>

namespace chassis {
namespace {

// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮 600 rpm。
// LemLib 约定正控制量驱动底盘前进、左正右负使车头顺时针（向右）旋转。
// 若实机推前摇杆反而倒车，请将两侧端口符号同时取反，
// 勿仅在手动输入取反，否则自动路线与里程计方向会反向。
constexpr std::uint8_t kImuPort = 5;

// 轮距和轮径来自路线规划配置，待实车复核。
constexpr float kTrackWidthIn = 11.58F;
constexpr float kWheelDiameterIn = 2.75F;
constexpr float kDrivetrainRpm = 600.0F * 6.0F / 10.0F; // 蓝齿轮组 600 rpm，轮端 360 rpm。
constexpr float kHorizontalDrift = 8.0F;                // LemLib 转弯限速参数，待实机标定。

pros::MotorGroup _g_left({-2, -1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({3, 4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Imu _g_imu(kImuPort);

lemlib::Drivetrain _g_drivetrain(&_g_left, &_g_right, kTrackWidthIn, kWheelDiameterIn,
                                 kDrivetrainRpm, kHorizontalDrift);

// 无外接定位轮。LemLib 校准后用左右驱动电机编码器和传动比换算纵向位移。
lemlib::OdomSensors _g_sensors(nullptr, nullptr, nullptr, nullptr, &_g_imu);

// TODO: 待实机标定纵向与转向 PID 参数。
lemlib::ControllerSettings _g_lateral_settings(10.0F,  // kP
                                               0.0F,   // kI
                                               3.0F,   // kD
                                               3.0F,   // anti-windup
                                               1.0F,   // small error range (in)
                                               100.0F, // small error timeout (ms)
                                               3.0F,   // large error range (in)
                                               500.0F, // large error timeout (ms)
                                               20.0F   // slew
);

lemlib::ControllerSettings _g_angular_settings(2.0F,   // kP
                                               0.0F,   // kI
                                               10.0F,  // kD
                                               3.0F,   // anti-windup
                                               1.0F,   // small error range (deg)
                                               100.0F, // small error timeout (ms)
                                               3.0F,   // large error range (deg)
                                               500.0F, // large error timeout (ms)
                                               0.0F    // slew
);

// TODO: 按手柄手感实机微调死区、最小输出和指数曲线增益。
lemlib::ExpoDriveCurve _g_throttle_curve(3.0F, 10.0F, 1.019F);
lemlib::ExpoDriveCurve _g_steer_curve(3.0F, 10.0F, 1.019F);

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
