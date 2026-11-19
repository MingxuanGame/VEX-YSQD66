#include "chassis.h"

#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <cstdint>

namespace chassis {
namespace {

// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮 600 rpm。
// LemLib 约定正控制量驱动底盘前进、左正右负使车头顺时针（向右）旋转。
// 若实机推前摇杆反而倒车，请将两侧端口符号同时取反（改为 {-2, -1} 与 {3, 4}），
// 勿仅在手动输入取反，否则自动路线与里程计方向会反向。
constexpr std::uint8_t kImuPort = 5;

// TODO: 以下机械几何与 PID 参数均为初值，须按实机测量与调试结果标定。
constexpr float kTrackWidthIn = 12.0F;                       // 待实机测量左右轮中心距（英寸）
constexpr float kWheelDiameterIn = lemlib::Omniwheel::NEW_4; // 待实机确认直轮外径
constexpr float kDrivetrainRpm = 600.0F; // 蓝齿轮直驱；若有外接减速比须按轮轴转速修改
constexpr float kHorizontalDrift = 8.0F; // 双侧直轮抓地力较高，LemLib 建议初值 8

pros::MotorGroup _g_left({2, 1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-3, -4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Imu _g_imu(kImuPort);

lemlib::Drivetrain _g_drivetrain(&_g_left, &_g_right, kTrackWidthIn, kWheelDiameterIn,
                                 kDrivetrainRpm, kHorizontalDrift);

// 沿用双侧直轮设计，不外接从动定位轮；vertical1/vertical2 传 nullptr 时
// LemLib::Chassis::calibrate() 会自动绑定左右驱动电机组作为纵向定位轮。
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

} // namespace chassis
