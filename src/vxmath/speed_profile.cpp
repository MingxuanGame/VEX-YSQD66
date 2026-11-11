#include "vxmath/speed_profile.hpp"
#include <cmath>
#include "vxmath/math_utils.hpp"

namespace vxmath {

TrapezoidProfile::TrapezoidProfile() : m_d(0.0), m_vmax(0.0), m_a(0.0), m_da(0.0), m_tc(0.0) {}

TrapezoidProfile::TrapezoidProfile(double distance, double maxVel, double accel)
    : TrapezoidProfile() {
    configure(distance, maxVel, accel);
}

void TrapezoidProfile::configure(double distance, double maxVel, double accel) {
    m_d = std::fabs(distance);
    m_vmax = std::fabs(maxVel);
    m_a = std::fabs(accel);
    m_da = 0.0;
    m_tc = 0.0;
    if (m_d <= 0.0 || m_vmax <= 0.0 || m_a <= 0.0)
        return;

    // 计算加速到最大速度需要的距离： v^2 = 2 * a * x
    double d_accel = (m_vmax * m_vmax) / (2.0 * m_a);

    if (2.0 * d_accel >= m_d) {
        // 距离太短，还没来得及加速就得减速（三角形曲线）
        m_vmax = std::sqrt(m_a * m_d); // 重新计算峰值速度
        m_da = m_d / 2.0;
        m_tc = 0.0;
    } else {
        // 距离够长，可以加速到最高速再匀速（梯形曲线）
        m_da = d_accel;
        m_tc = m_d - 2.0 * m_da;
    }
}

double TrapezoidProfile::getVelocity(double traveled) const {
    if (m_d <= 0.0)
        return 0.0;
    double x = clampVal(std::fabs(traveled), 0.0, m_d);

    // 加速段：速度随距离开根号增长
    if (x <= m_da)
        return std::sqrt(2.0 * m_a * x);
    // 减速段：距离终点越近，速度越低
    if (x >= m_d - m_da)
        return std::sqrt(2.0 * m_a * (m_d - x));
    // 匀速段
    return m_vmax;
}

double TrapezoidProfile::getVelocityRatio(double traveled) const {
    if (m_vmax <= 0.0)
        return 0.0;
    return clampVal(getVelocity(traveled) / m_vmax, 0.0, 1.0);
}

bool TrapezoidProfile::isDone(double traveled) const {
    return std::fabs(traveled) >= m_d;
}

} // namespace vxmath
