#include "sim/hardware/sim_mechanism.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

namespace {

double clampValue(double value, double lo, double hi) {
    if (value < lo)
        return lo;
    if (value > hi)
        return hi;
    return value;
}

} // namespace

SimMechanism::SimMechanism(const SimConfig &config)
    : m_config(config), m_lift_height_m(0.0), m_lift_velocity_mps(0.0), m_lift_target_m(0.0),
      m_lift_speed_limit_mps(0.0), m_lift_position_mode(false), m_lift_holding(true),
      m_clamp_angle_deg(config.clamp.open_angle_deg),
      m_clamp_target_deg(config.clamp.open_angle_deg), m_push_position(0.0), m_push_target(0.0),
      m_current_ma(0.0) {}

void SimMechanism::update(double dt_sec) {
    if (!(dt_sec > 0.0))
        return;

    updateLift(dt_sec);
    updateClamp(dt_sec);
    updatePush(dt_sec);

    const bool lift_moving = std::fabs(m_lift_velocity_mps) > 0.001;
    const bool clamp_moving = std::fabs(m_clamp_target_deg - m_clamp_angle_deg) > 0.1;
    m_current_ma = 0.0;
    if (lift_moving) {
        m_current_ma += m_config.lift.move_current_ma;
    } else {
        m_current_ma += m_config.lift.hold_current_ma;
    }
    if (clamp_moving) {
        m_current_ma += m_config.clamp.hold_current_ma;
    }
}

void SimMechanism::reset() {
    m_lift_height_m = 0.0;
    m_lift_velocity_mps = 0.0;
    m_lift_target_m = 0.0;
    m_lift_speed_limit_mps = 0.0;
    m_lift_position_mode = false;
    m_lift_holding = true;
    m_clamp_angle_deg = m_config.clamp.open_angle_deg;
    m_clamp_target_deg = m_config.clamp.open_angle_deg;
    m_push_position = 0.0;
    m_push_target = 0.0;
    m_current_ma = 0.0;
}

void SimMechanism::updateLift(double dt_sec) {
    const double max_height = m_config.lift.cascade_stages * m_config.lift.stage_height_m;

    if (m_lift_position_mode) {
        const double error = m_lift_target_m - m_lift_height_m;
        if (std::fabs(error) < 0.002) {
            m_lift_velocity_mps = 0.0;
            m_lift_height_m = m_lift_target_m;
            return;
        }
        m_lift_velocity_mps = clampValue(m_config.lift.position_kp * error, -m_lift_speed_limit_mps,
                                         m_lift_speed_limit_mps);
    }

    m_lift_height_m += m_lift_velocity_mps * dt_sec;
    if (m_lift_height_m <= 0.0) {
        m_lift_height_m = 0.0;
        if (m_lift_velocity_mps < 0.0)
            m_lift_velocity_mps = 0.0;
    }
    if (m_lift_height_m >= max_height) {
        m_lift_height_m = max_height;
        if (m_lift_velocity_mps > 0.0)
            m_lift_velocity_mps = 0.0;
    }
}

void SimMechanism::updateClamp(double dt_sec) {
    const double max_step = m_config.clamp.slew_deg_per_sec * dt_sec;
    const double diff = m_clamp_target_deg - m_clamp_angle_deg;
    if (std::fabs(diff) <= max_step) {
        m_clamp_angle_deg = m_clamp_target_deg;
    } else {
        m_clamp_angle_deg += (diff > 0.0) ? max_step : -max_step;
    }
    m_clamp_angle_deg = clampValue(m_clamp_angle_deg, 0.0, 180.0);
}

void SimMechanism::updatePush(double dt_sec) {
    const double travel = std::max(0.01, m_config.push.travel_time_sec);
    const double step = dt_sec / travel;
    if (m_push_position < m_push_target) {
        m_push_position = std::min(m_push_target, m_push_position + step);
    } else if (m_push_position > m_push_target) {
        m_push_position = std::max(m_push_target, m_push_position - step);
    }
    m_push_position = clampValue(m_push_position, 0.0, 1.0);
}

void SimMechanism::liftTo(double pos, int max_speed) {
    const double max_height = m_config.lift.cascade_stages * m_config.lift.stage_height_m;
    m_lift_target_m = clampValue(pos, 0.0, max_height);
    const int speed = std::max(1, std::min(127, max_speed));
    m_lift_speed_limit_mps = m_config.lift.max_speed_mps * speed / 127.0;
    m_lift_position_mode = true;
    m_lift_holding = false;
}

void SimMechanism::liftManual(int speed) {
    const int clipped = std::max(-127, std::min(127, speed));
    m_lift_velocity_mps = m_config.lift.max_speed_mps * clipped / 127.0;
    m_lift_position_mode = false;
    m_lift_holding = false;
}

void SimMechanism::liftHold() {
    m_lift_velocity_mps = 0.0;
    m_lift_position_mode = false;
    m_lift_holding = true;
}

void SimMechanism::clampOpen() {
    m_clamp_target_deg = m_config.clamp.open_angle_deg;
}

void SimMechanism::clampClose() {
    m_clamp_target_deg = m_config.clamp.closed_angle_deg;
}

void SimMechanism::clampSetAngle(int angle) {
    m_clamp_target_deg = clampValue(static_cast<double>(angle), 0.0, 180.0);
}

void SimMechanism::pushExtend() {
    m_push_target = 1.0;
}

void SimMechanism::pushRetract() {
    m_push_target = 0.0;
}

double SimMechanism::getCurrentMa() const {
    return m_current_ma;
}

double SimMechanism::getLiftHeightM() const {
    return m_lift_height_m;
}

double SimMechanism::getClampAngleDeg() const {
    return m_clamp_angle_deg;
}

double SimMechanism::getPushPosition() const {
    return m_push_position;
}

double SimMechanism::getCascadeStage() const {
    if (m_config.lift.stage_height_m <= 0.0)
        return 0.0;
    return m_lift_height_m / m_config.lift.stage_height_m;
}

bool SimMechanism::isLiftMoving() const {
    return std::fabs(m_lift_velocity_mps) > 0.001;
}

} // namespace sim
