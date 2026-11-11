#include "motion/motion_engine.hpp"
#include "config/motion_config.hpp"
#include "robot/battery.hpp"
#include "robot/chassis.hpp"
#include "robot/clock.hpp"
#include "robot/imu.hpp"
#include "vxmath/math_utils.hpp"
#include "vxmath/units.hpp"
#include <cmath>

namespace motion {

namespace {
constexpr double kRadToDeg = 180.0 / 3.141592653589793;
constexpr double kDegToRad = 3.141592653589793 / 180.0;
constexpr double kMaxHeadingCorrectionRpm = 30.0;
constexpr double kMaxDtSec = 0.1;
constexpr double kMinDtSec = 0.001;
constexpr uint32_t kMinTimeoutMs = 300;
constexpr uint32_t kMaxTimeoutMs = 15000;

int clampSpeed(int speed) {
    if (speed > cfg::kOutputLimit)
        return cfg::kOutputLimit;
    if (speed < 1)
        return 1;
    return speed;
}

int clampOutput(int output) {
    if (output > cfg::kOutputLimit)
        return cfg::kOutputLimit;
    if (output < -cfg::kOutputLimit)
        return -cfg::kOutputLimit;
    return output;
}

uint32_t timeoutFor(double seconds) {
    if (!(seconds > 0.0))
        seconds = 1.0;
    double ms = (seconds * 1.8 + 0.4) * 1000.0;
    if (ms < static_cast<double>(kMinTimeoutMs))
        return kMinTimeoutMs;
    if (ms > static_cast<double>(kMaxTimeoutMs))
        return kMaxTimeoutMs;
    return static_cast<uint32_t>(ms);
}

} // namespace

MotionEngine::MotionEngine(robot::Chassis &chassis, robot::Imu &imu, robot::Battery *battery)
    : m_chassis(&chassis), m_imu(&imu), m_battery(battery), m_state(State::Idle),
      m_last_update_ms(0), m_deadline_ms(0), m_wait_until_ms(0),
      m_dt_sec(cfg::kControlLoopMs / 1000.0), m_drive_target_inch(0.0), m_drive_traveled(0.0),
      m_drive_direction(1.0), m_drive_target_heading_deg(0.0), m_max_drive_velocity_ips(0.0),
      m_turn_target_deg(0.0), m_turn_total_deg(0.0), m_turn_traveled(0.0), m_max_turn_dps(0.0),
      m_position_tolerance_inch(cfg::kPositionToleranceInch),
      m_angle_tolerance_deg(cfg::kAngleToleranceDeg) {}

void MotionEngine::startDrive(double distance_inch, int max_speed) {
    if (!m_chassis || !m_imu)
        return;
    if (!std::isfinite(distance_inch))
        return;

    m_kinematics.update(m_chassis, m_imu, cfg::kControlLoopMs / 1000.0);
    const double target_distance = std::fabs(distance_inch);
    if (target_distance < m_position_tolerance_inch) {
        stop();
        return;
    }
    if (max_speed <= 0) {
        stop();
        return;
    }

    const int speed = clampSpeed(max_speed);
    m_drive_direction = (distance_inch >= 0.0) ? 1.0 : -1.0;
    m_drive_target_inch = target_distance;
    m_drive_traveled = 0.0;
    m_max_drive_velocity_ips =
        vxmath::rpmToInchesPerSec(cfg::kMaxDriveRpm * speed / cfg::kOutputLimit);
    m_profile.configure(m_drive_target_inch, m_max_drive_velocity_ips,
                        cfg::kDriveAccelInchesPerSec2);
    m_drive_target_heading_deg = m_kinematics.getPose().heading * kRadToDeg;

    m_velocity.reset();
    const uint32_t now = robot::nowMs();
    m_last_update_ms = now;
    const double estimate = m_drive_target_inch / m_max_drive_velocity_ips +
                            m_max_drive_velocity_ips / cfg::kDriveAccelInchesPerSec2;
    m_deadline_ms = now + timeoutFor(estimate);
    m_state = State::Driving;
}

void MotionEngine::startTurn(double target_angle_deg, int max_speed) {
    if (!m_chassis || !m_imu)
        return;
    if (!std::isfinite(target_angle_deg))
        return;

    m_kinematics.update(m_chassis, m_imu, cfg::kControlLoopMs / 1000.0);
    if (std::fabs(target_angle_deg) < m_angle_tolerance_deg) {
        stop();
        return;
    }
    if (max_speed <= 0) {
        stop();
        return;
    }

    const int speed = clampSpeed(max_speed);
    const double current_deg = m_kinematics.getPose().heading * kRadToDeg;
    m_turn_target_deg = vxmath::restrictAngle(current_deg + target_angle_deg);
    m_turn_total_deg = std::fabs(target_angle_deg);
    m_turn_traveled = 0.0;
    m_max_turn_dps = cfg::kMaxTurnDps * speed / cfg::kOutputLimit;
    m_profile.configure(m_turn_total_deg, m_max_turn_dps, cfg::kTurnAccelDegPerSec2);

    m_velocity.reset();
    const uint32_t now = robot::nowMs();
    m_last_update_ms = now;
    const double estimate =
        m_turn_total_deg / m_max_turn_dps + m_max_turn_dps / cfg::kTurnAccelDegPerSec2;
    m_deadline_ms = now + timeoutFor(estimate);
    m_state = State::Turning;
}

void MotionEngine::startWait(uint32_t duration_ms) {
    if (!m_chassis)
        return;
    m_chassis->stop(true);
    m_velocity.reset();
    m_last_update_ms = robot::nowMs();
    m_wait_until_ms = m_last_update_ms + duration_ms;
    m_state = State::Waiting;
}

void MotionEngine::stop() {
    if (m_chassis)
        m_chassis->stop(true);
    m_velocity.reset();
    m_state = State::Idle;
    m_drive_traveled = 0.0;
    m_turn_traveled = 0.0;
}

void MotionEngine::clearFault() {
    if (m_state == State::Fault) {
        m_velocity.reset();
        m_state = State::Idle;
    }
}

void MotionEngine::enterFault() {
    if (m_chassis)
        m_chassis->stop(true);
    m_velocity.reset();
    m_state = State::Fault;
}

void MotionEngine::update(uint32_t now_ms, bool input_connected) {
    if (!m_chassis || !m_imu)
        return;
    if (!input_connected) {
        stop();
        return;
    }

    uint32_t delta_ms = now_ms - m_last_update_ms;
    if (m_last_update_ms == 0 || delta_ms == 0)
        delta_ms = cfg::kControlLoopMs;
    if (delta_ms > static_cast<uint32_t>(kMaxDtSec * 1000.0)) {
        delta_ms = static_cast<uint32_t>(kMaxDtSec * 1000.0);
    }
    if (delta_ms < static_cast<uint32_t>(kMinDtSec * 1000.0)) {
        delta_ms = static_cast<uint32_t>(kMinDtSec * 1000.0);
    }
    m_dt_sec = delta_ms / 1000.0;
    m_last_update_ms = now_ms;

    m_kinematics.update(m_chassis, m_imu, m_dt_sec);

    if (m_state == State::Idle || m_state == State::Fault)
        return;

    if (m_state != State::Waiting && static_cast<int32_t>(now_ms - m_deadline_ms) >= 0) {
        enterFault();
        return;
    }

    switch (m_state) {
    case State::Driving:
        updateDrive(now_ms);
        break;
    case State::Turning:
        updateTurn(now_ms);
        break;
    case State::Waiting:
        updateWaiting(now_ms);
        break;
    default:
        break;
    }
}

void MotionEngine::updateDrive(uint32_t now_ms) {
    (void)now_ms;
    m_drive_traveled += m_kinematics.getLastDistanceInches() * m_drive_direction;

    // 位置式 profile 在 0 处速度为 0，前视一点距离，避免起步死锁；
    // 接近终点时用实际位置，避免前视把目标速度提前压成 0。
    double lookahead = cfg::kDriveProfileLookaheadInch;
    if (lookahead > m_drive_target_inch * 0.5)
        lookahead = m_drive_target_inch * 0.5;
    double profile_position = m_drive_traveled;
    if (profile_position < lookahead)
        profile_position = lookahead;
    const double target_velocity = m_profile.getVelocity(profile_position);
    const double base_rpm = vxmath::inchesPerSecToRpm(target_velocity) * m_drive_direction;

    const double current_deg = m_kinematics.getPose().heading * kRadToDeg;
    const double heading_error = vxmath::getError(current_deg, m_drive_target_heading_deg);
    double correction = cfg::kHeadingKp * heading_error;
    if (correction > kMaxHeadingCorrectionRpm)
        correction = kMaxHeadingCorrectionRpm;
    if (correction < -kMaxHeadingCorrectionRpm)
        correction = -kMaxHeadingCorrectionRpm;

    const double target_left = base_rpm - correction;
    const double target_right = base_rpm + correction;

    m_velocity.update(target_left, target_right, m_chassis->getLeftVelocity(),
                      m_chassis->getRightVelocity(), m_dt_sec);
    applyOutputs();

    if (m_drive_traveled >= m_drive_target_inch - m_position_tolerance_inch ||
        m_profile.isDone(m_drive_traveled)) {
        stop();
    }
}

void MotionEngine::updateTurn(uint32_t now_ms) {
    (void)now_ms;
    const double delta_deg = m_kinematics.getLastDeltaHeadingRad() * kRadToDeg;
    m_turn_traveled += std::fabs(delta_deg);

    const double current_deg = m_kinematics.getPose().heading * kRadToDeg;
    const double error = vxmath::getError(current_deg, m_turn_target_deg);
    if (std::fabs(error) < m_angle_tolerance_deg) {
        stop();
        return;
    }

    // 用“剩余角度”而不是累计角度做规划，避免 IMU 噪声让累计角度提前到达终点。
    const double direction = (error >= 0.0) ? 1.0 : -1.0;
    double progress = m_turn_total_deg - std::fabs(error);
    if (progress < 0.0)
        progress = 0.0;
    double lookahead = cfg::kTurnProfileLookaheadDeg;
    if (lookahead > m_turn_total_deg * 0.5)
        lookahead = m_turn_total_deg * 0.5;
    if (progress < lookahead)
        progress = lookahead;
    const double target_dps = m_profile.getVelocity(progress) * direction;
    const double omega_rad_s = target_dps * kDegToRad;
    const double wheel_ips = omega_rad_s * cfg::kTrackWidthInch / 2.0;
    const double target_rpm = vxmath::inchesPerSecToRpm(wheel_ips);

    m_velocity.update(-target_rpm, target_rpm, m_chassis->getLeftVelocity(),
                      m_chassis->getRightVelocity(), m_dt_sec);
    applyOutputs();
}

void MotionEngine::updateWaiting(uint32_t now_ms) {
    if (now_ms >= m_wait_until_ms) {
        m_state = State::Idle;
        if (m_chassis)
            m_chassis->stop(true);
    }
}

void MotionEngine::applyOutputs() {
    if (!m_chassis)
        return;
    const int left = clampOutput(compensate(m_velocity.getLeftOutput()));
    const int right = clampOutput(compensate(m_velocity.getRightOutput()));
    m_chassis->setDrive(left, right);
}

int MotionEngine::compensate(int output) const {
    if (!m_battery)
        return output;
    const int voltage_mv = m_battery->getVoltageMv();
    if (voltage_mv <= 0)
        return output;

    double scale = static_cast<double>(cfg::kNominalBatteryMv) / voltage_mv;
    if (scale < 1.0)
        scale = 1.0;
    if (scale > cfg::kMaxBatteryCompensation)
        scale = cfg::kMaxBatteryCompensation;

    const int scaled = static_cast<int>(std::lround(output * scale));
    return clampOutput(scaled);
}

bool MotionEngine::isBusy() const {
    return m_state == State::Driving || m_state == State::Turning || m_state == State::Waiting;
}

State MotionEngine::getState() const {
    return m_state;
}

const Pose &MotionEngine::getPose() const {
    return m_kinematics.getPose();
}

void MotionEngine::resetPose() {
    m_kinematics.reset();
}

void MotionEngine::setVelocityKp(double kp) {
    m_velocity.setKp(kp);
}

void MotionEngine::setVelocityKi(double ki) {
    m_velocity.setKi(ki);
}

void MotionEngine::setVelocityKd(double kd) {
    m_velocity.setKd(kd);
}

void MotionEngine::setPositionTolerance(double inch) {
    if (inch >= 0.0)
        m_position_tolerance_inch = inch;
}

void MotionEngine::setAngleTolerance(double deg) {
    if (deg >= 0.0)
        m_angle_tolerance_deg = deg;
}

} // namespace motion
