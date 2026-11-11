#pragma once
#include <cstdint>
#include "motion/chassis_kinematics.hpp"
#include "motion/motion_profile.hpp"
#include "motion/velocity_controller.hpp"

namespace robot {
class Chassis;
class Imu;
class Battery;
} // namespace robot

namespace motion {

enum class State { Idle, Driving, Turning, Waiting, Fault };

class MotionEngine {
public:
    MotionEngine(robot::Chassis &chassis, robot::Imu &imu, robot::Battery *battery = nullptr);

    // 距离单位英寸；max_speed 为 0~127 的输出上限。
    void startDrive(double distance_inch, int max_speed);
    // target_angle_deg 为相对当前航向的角度（正数逆时针）。
    void startTurn(double target_angle_deg, int max_speed);
    void startWait(uint32_t duration_ms);

    void stop();
    void clearFault();

    // 每 10ms 调一次；input_connected 为 false 时立即刹车。
    void update(uint32_t now_ms, bool input_connected = true);

    bool isBusy() const;
    State getState() const;
    const Pose &getPose() const;
    void resetPose();

    // 供调参系统实时修改速度环增益。
    void setVelocityKp(double kp);
    void setVelocityKi(double ki);
    void setVelocityKd(double kd);
    void setPositionTolerance(double inch);
    void setAngleTolerance(double deg);

private:
    void updateDrive(uint32_t now_ms);
    void updateTurn(uint32_t now_ms);
    void updateWaiting(uint32_t now_ms);
    void applyOutputs();
    void enterFault();
    int compensate(int output) const;

    robot::Chassis *m_chassis;
    robot::Imu *m_imu;
    robot::Battery *m_battery;

    ChassisKinematics m_kinematics;
    VelocityController m_velocity;
    TrapezoidProfile m_profile;

    State m_state;
    uint32_t m_last_update_ms;
    uint32_t m_deadline_ms;
    uint32_t m_wait_until_ms;

    double m_dt_sec;
    double m_drive_target_inch;
    double m_drive_traveled;
    double m_drive_direction;
    double m_drive_target_heading_deg;
    double m_max_drive_velocity_ips;
    double m_turn_target_deg;
    double m_turn_total_deg;
    double m_turn_traveled;
    double m_max_turn_dps;
    double m_position_tolerance_inch;
    double m_angle_tolerance_deg;
};

} // namespace motion
