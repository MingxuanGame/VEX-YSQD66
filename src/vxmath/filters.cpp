#include "vxmath/filters.hpp"
#include "vxmath/math_utils.hpp"
#include "config/motion_config.hpp"
#include <cmath>

namespace vxmath {

LowPass::LowPass(double alpha) : m_alpha(clampVal(alpha, 1e-6, 1.0)), m_last(0.0), m_init(false) {}

double LowPass::update(double v) {
    if (!m_init) {
        m_last = v;
        m_init = true;
    } else {
        m_last = m_alpha * v + (1.0 - m_alpha) * m_last;
    }
    return m_last;
}

void LowPass::reset() {
    m_last = 0.0;
    m_init = false;
}

int applyDeadzone(int v, int deadzone) {
    if (std::abs(v) < deadzone)
        return 0;
    return v;
}

int expoCurve(int v, double expo) {
    // expo <= 0 会让 pow(x, 0) 恒等于 1，摇杆会瞬间满输出，必须兜底。
    if (!(expo > 0.0) || !std::isfinite(expo))
        expo = 1.0;
    int clamped = clampVal(v, -cfg::kMotorMaxOutput, cfg::kMotorMaxOutput);
    double x = static_cast<double>(clamped) / cfg::kMotorMaxOutput;
    double y = std::copysign(std::pow(std::fabs(x), expo), x);
    return static_cast<int>(y * cfg::kMotorMaxOutput);
}

Hysteresis::Hysteresis(double threshold, double hysteresis)
    : m_threshold(threshold), m_hys(hysteresis), m_state(false) {}

bool Hysteresis::update(double v) {
    if (m_state) {
        if (v < m_threshold - m_hys)
            m_state = false;
    } else {
        if (v > m_threshold + m_hys)
            m_state = true;
    }
    return m_state;
}

void Hysteresis::reset() {
    m_state = false;
}

} // namespace vxmath
