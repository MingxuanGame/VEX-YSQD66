#include "motion/wheel_velocity_controller.hpp"

#include <algorithm>
#include <cmath>

namespace motion {

WheelVelocityController::WheelVelocityController() {
    const WheelVelocityGains gains{};
    for (int i = 0; i < 4; ++i)
        m_gains[i] = gains;
    for (int i = 0; i < 4; ++i)
        applyGains(i);
}

void WheelVelocityController::applyGains(int wheel) {
    const WheelVelocityGains &gains = m_gains[wheel];
    m_pids[wheel].setGains(gains.kp, gains.ki, gains.kd);
    m_pids[wheel].setIntegralLimits(-gains.integral_limit, gains.integral_limit);
    m_pids[wheel].setOutputLimits(-gains.output_limit, gains.output_limit);
    m_pids[wheel].setDerivativeFilterTau(gains.derivative_filter_tau);
}

void WheelVelocityController::setGains(const WheelVelocityGains &gains) {
    for (int i = 0; i < 4; ++i) {
        m_gains[i] = gains;
        applyGains(i);
        m_pids[i].reset();
        m_outputs[i] = 0;
    }
}

void WheelVelocityController::setWheelGains(int wheel, const WheelVelocityGains &gains) {
    if (wheel < 0 || wheel >= 4)
        return;
    m_gains[wheel] = gains;
    applyGains(wheel);
    m_pids[wheel].reset();
    m_outputs[wheel] = 0;
}

const WheelVelocityGains &WheelVelocityController::getGains() const {
    return m_gains[0];
}

std::array<int, 4> WheelVelocityController::update(const std::array<double, 4> &target_rpm,
                                                   const std::array<double, 4> &current_rpm,
                                                   double dt_sec) {
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(target_rpm[i]) || !std::isfinite(current_rpm[i])) {
            reset();
            return m_outputs;
        }
    }

    for (int i = 0; i < 4; ++i) {
        const double error = target_rpm[i] - current_rpm[i];
        const double output = m_pids[i].update(error, dt_sec);
        m_outputs[i] = std::clamp(static_cast<int>(std::lround(output)), -127, 127);
    }
    return m_outputs;
}

const std::array<int, 4> &WheelVelocityController::getOutputs() const {
    return m_outputs;
}

void WheelVelocityController::reset() {
    for (Pid &pid : m_pids)
        pid.reset();
    m_outputs.fill(0);
}

} // namespace motion
