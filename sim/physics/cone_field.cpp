#include "sim/physics/cone_field.hpp"

#include <algorithm>
#include <cmath>

namespace sim_physics {

namespace {

constexpr double kRobotRadiusM = 0.23;
constexpr double kConeDampingPerSec = 4.0;
constexpr double kPushSpeedFloorMps = 0.12;

// 屏幕俯视图中的起点，换算到世界坐标时 y 轴取反。
constexpr double kStartPositions[ConeField::kConeCount][2] = {
    {1.0 / 6.0, 1.0 / 6.0}, {2.0 / 6.0, 2.0 / 6.0}, {4.0 / 6.0, 4.0 / 6.0}, {5.0 / 6.0, 5.0 / 6.0}};

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

} // namespace

ConeField::ConeField(double field_size_m) : m_field_size_m(field_size_m), m_contact(false) {
    reset();
}

void ConeField::reset() {
    m_contact = false;
    for (int i = 0; i < kConeCount; ++i) {
        ConeBody &cone = m_cones[i];
        cone.x_m = kStartPositions[i][0] * m_field_size_m;
        cone.y_m = (1.0 - kStartPositions[i][1]) * m_field_size_m;
        cone.velocity_x_mps = 0.0;
        cone.velocity_y_mps = 0.0;
        cone.radius_m = 0.10;
        cone.active = true;
        cone.out_of_bounds = false;
    }
}

void ConeField::update(double dt_sec, double robot_x_m, double robot_y_m, double robot_vx_mps,
                       double robot_vy_mps) {
    if (!(dt_sec > 0.0))
        return;

    m_contact = false;
    const double damping = std::exp(-kConeDampingPerSec * dt_sec);
    const double robot_speed = std::hypot(robot_vx_mps, robot_vy_mps);

    for (int i = 0; i < kConeCount; ++i) {
        ConeBody &cone = m_cones[i];
        if (!cone.active || cone.out_of_bounds)
            continue;

        cone.velocity_x_mps *= damping;
        cone.velocity_y_mps *= damping;
        cone.x_m += cone.velocity_x_mps * dt_sec;
        cone.y_m += cone.velocity_y_mps * dt_sec;

        const double min_center = cone.radius_m;
        const double max_center = m_field_size_m - cone.radius_m;
        if (cone.x_m < min_center || cone.x_m > max_center || cone.y_m < min_center ||
            cone.y_m > max_center) {
            cone.x_m = clampValue(cone.x_m, min_center, max_center);
            cone.y_m = clampValue(cone.y_m, min_center, max_center);
            cone.velocity_x_mps = 0.0;
            cone.velocity_y_mps = 0.0;
            cone.active = false;
            cone.out_of_bounds = true;
            continue;
        }

        double dx = cone.x_m - robot_x_m;
        double dy = cone.y_m - robot_y_m;
        const double min_distance = cone.radius_m + kRobotRadiusM;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq >= min_distance * min_distance)
            continue;

        m_contact = true;
        double distance = std::sqrt(distance_sq);
        if (distance < 1e-6) {
            dx = 1.0;
            dy = 0.0;
            distance = 1.0;
        }
        const double nx = dx / distance;
        const double ny = dy / distance;
        cone.x_m = robot_x_m + nx * (min_distance + 0.001);
        cone.y_m = robot_y_m + ny * (min_distance + 0.001);

        const double push_speed = std::max(kPushSpeedFloorMps, robot_speed * 0.75);
        cone.velocity_x_mps = nx * push_speed;
        cone.velocity_y_mps = ny * push_speed;
    }
}

bool ConeField::hasContact() const {
    return m_contact;
}

const ConeBody &ConeField::get(int index) const {
    static const ConeBody kEmpty{};
    if (index < 0 || index >= kConeCount)
        return kEmpty;
    return m_cones[index];
}

int ConeField::count() const {
    return kConeCount;
}

} // namespace sim_physics
