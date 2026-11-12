#include "sim/physics/toggle_field.hpp"

#include <algorithm>
#include <cmath>

namespace sim_physics {

namespace {

constexpr double kToggleRadiusM = 0.28;
constexpr double kToggleRotationDegPerMps = 240.0;
constexpr double kToggleSnapPerSec = 8.0;

double normalizeAngle(double angle_deg) {
    while (angle_deg < 0.0)
        angle_deg += 360.0;
    while (angle_deg >= 360.0)
        angle_deg -= 360.0;
    return angle_deg;
}

} // namespace

ToggleField::ToggleField(double field_size_m) : m_field_size_m(field_size_m) {
    reset();
}

void ToggleField::reset() {
    const double edge = 0.18;
    for (int i = 0; i < kToggleCount; ++i) {
        ToggleState &toggle = m_toggles[i];
        toggle.angle_deg = 240.0;
        toggle.seated = true;
        toggle.touching = false;
        toggle.quadrant = i;
        toggle.face = ToggleColor::Yellow;
    }
    m_toggles[0].x_m = 0.5 * m_field_size_m;
    m_toggles[0].y_m = m_field_size_m - edge;
    m_toggles[1].x_m = m_field_size_m - edge;
    m_toggles[1].y_m = 0.5 * m_field_size_m;
    m_toggles[2].x_m = 0.5 * m_field_size_m;
    m_toggles[2].y_m = edge;
    m_toggles[3].x_m = edge;
    m_toggles[3].y_m = 0.5 * m_field_size_m;
}

double ToggleField::nearestDetent(double angle_deg) const {
    const int index = static_cast<int>(std::round(normalizeAngle(angle_deg) / 120.0)) % 3;
    return static_cast<double>(index * 120);
}

ToggleColor ToggleField::colorForAngle(double angle_deg) const {
    const int index = static_cast<int>(std::round(normalizeAngle(angle_deg) / 120.0)) % 3;
    if (index == 0)
        return ToggleColor::Red;
    if (index == 1)
        return ToggleColor::Blue;
    return ToggleColor::Yellow;
}

void ToggleField::update(double dt_sec, double robot_x_m, double robot_y_m, double robot_vx_mps,
                         double robot_vy_mps) {
    if (!(dt_sec > 0.0))
        return;

    for (int i = 0; i < kToggleCount; ++i) {
        ToggleState &toggle = m_toggles[i];
        const double dx = robot_x_m - toggle.x_m;
        const double dy = robot_y_m - toggle.y_m;
        const bool touching = dx * dx + dy * dy < kToggleRadiusM * kToggleRadiusM;
        toggle.touching = touching;

        if (touching) {
            toggle.seated = false;
            toggle.face = ToggleColor::Yellow;
            const bool vertical = i == 1 || i == 3;
            const double tangent_speed = vertical ? robot_vy_mps : robot_vx_mps;
            toggle.angle_deg = normalizeAngle(toggle.angle_deg +
                                              tangent_speed * kToggleRotationDegPerMps * dt_sec);
            continue;
        }

        const double target = nearestDetent(toggle.angle_deg);
        double difference = target - toggle.angle_deg;
        if (difference > 180.0)
            difference -= 360.0;
        if (difference < -180.0)
            difference += 360.0;
        toggle.angle_deg = normalizeAngle(toggle.angle_deg +
                                          difference * std::min(1.0, kToggleSnapPerSec * dt_sec));
        if (std::fabs(difference) < 1.0) {
            toggle.angle_deg = target;
            toggle.seated = true;
            toggle.face = colorForAngle(target);
        } else {
            toggle.seated = false;
            toggle.face = ToggleColor::Yellow;
        }
    }
}

const ToggleState &ToggleField::get(int index) const {
    static const ToggleState kEmpty{};
    if (index < 0 || index >= kToggleCount)
        return kEmpty;
    return m_toggles[index];
}

int ToggleField::count() const {
    return kToggleCount;
}

} // namespace sim_physics
