#include "sim/physics/cone_field.hpp"

#include <algorithm>
#include <cmath>

namespace sim_physics {

namespace {

constexpr double kRobotRadiusM = 0.23;
constexpr double kConeDampingPerSec = 4.0;
constexpr double kPushSpeedFloorMps = 0.12;

// 屏幕俯视图中的组合中心；x 从左到右，y 从上到下。
constexpr double kGroupCenters[ConeField::kGroupCount][2] = {
    {1.0 / 6.0, 1.0 / 6.0}, {2.0 / 6.0, 2.0 / 6.0}, {4.0 / 6.0, 4.0 / 6.0}, {5.0 / 6.0, 5.0 / 6.0}};
constexpr double kDoubleCenters[ConeField::kDoubleCount][2] = {{5.0 / 6.0, 1.0 / 6.0},
                                                               {1.0 / 6.0, 5.0 / 6.0}};

constexpr double kGroupOffsets[ConeField::kConesPerGroup][2] = {
    {-0.065, 0.0}, {0.065, 0.0}, {0.0, -0.065}, {0.0, 0.065}};
constexpr ConeColor kGroupColors[ConeField::kConesPerGroup] = {
    ConeColor::Red, ConeColor::Blue, ConeColor::Yellow, ConeColor::Yellow};
constexpr double kDoubleOffsets[ConeField::kConesPerDouble][2] = {{-0.060, 0.0}, {0.060, 0.0}};

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

} // namespace

ConeField::ConeField(double field_size_m) : m_field_size_m(field_size_m), m_contact(false) {
    reset();
}

void ConeField::reset() {
    m_contact = false;
    int index = 0;

    for (int group = 0; group < kGroupCount; ++group) {
        const double center_x_m = kGroupCenters[group][0] * m_field_size_m;
        const double center_y_m = (1.0 - kGroupCenters[group][1]) * m_field_size_m;
        for (int cone = 0; cone < kConesPerGroup; ++cone) {
            ConeBody &body = m_cones[index++];
            body.x_m = center_x_m + kGroupOffsets[cone][0];
            body.y_m = center_y_m + kGroupOffsets[cone][1];
            body.velocity_x_mps = 0.0;
            body.velocity_y_mps = 0.0;
            body.radius_m = 0.030;
            body.group = group;
            body.color = kGroupColors[cone];
            body.active = true;
            body.out_of_bounds = false;
        }
    }

    for (int pair = 0; pair < kDoubleCount; ++pair) {
        const double center_x_m = kDoubleCenters[pair][0] * m_field_size_m;
        const double center_y_m = (1.0 - kDoubleCenters[pair][1]) * m_field_size_m;
        for (int cone = 0; cone < kConesPerDouble; ++cone) {
            ConeBody &body = m_cones[index++];
            body.x_m = center_x_m + kDoubleOffsets[cone][0];
            body.y_m = center_y_m + kDoubleOffsets[cone][1];
            body.velocity_x_mps = 0.0;
            body.velocity_y_mps = 0.0;
            body.radius_m = 0.030;
            body.group = kGroupCount + pair;
            body.color = ConeColor::DoubleYellow;
            body.active = true;
            body.out_of_bounds = false;
        }
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
        ConeBody &body = m_cones[i];
        if (!body.active || body.out_of_bounds)
            continue;

        body.velocity_x_mps *= damping;
        body.velocity_y_mps *= damping;
        body.x_m += body.velocity_x_mps * dt_sec;
        body.y_m += body.velocity_y_mps * dt_sec;

        const double min_center = body.radius_m;
        const double max_center = m_field_size_m - body.radius_m;
        if (body.x_m < min_center || body.x_m > max_center || body.y_m < min_center ||
            body.y_m > max_center) {
            body.x_m = clampValue(body.x_m, min_center, max_center);
            body.y_m = clampValue(body.y_m, min_center, max_center);
            body.velocity_x_mps = 0.0;
            body.velocity_y_mps = 0.0;
            body.active = false;
            body.out_of_bounds = true;
            continue;
        }

        double dx = body.x_m - robot_x_m;
        double dy = body.y_m - robot_y_m;
        const double min_distance = body.radius_m + kRobotRadiusM;
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
        body.x_m = robot_x_m + nx * (min_distance + 0.001);
        body.y_m = robot_y_m + ny * (min_distance + 0.001);

        const double push_speed = std::max(kPushSpeedFloorMps, robot_speed * 0.75);
        body.velocity_x_mps = nx * push_speed;
        body.velocity_y_mps = ny * push_speed;
    }

    for (int first = 0; first < kConeCount; ++first) {
        for (int second = first + 1; second < kConeCount; ++second) {
            resolveConePair(first, second);
        }
    }
}

void ConeField::resolveConePair(int first, int second) {
    ConeBody &a = m_cones[first];
    ConeBody &b = m_cones[second];
    if (!a.active || !b.active || a.out_of_bounds || b.out_of_bounds)
        return;

    double dx = b.x_m - a.x_m;
    double dy = b.y_m - a.y_m;
    const double min_distance = a.radius_m + b.radius_m;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq >= min_distance * min_distance)
        return;

    double distance = std::sqrt(distance_sq);
    if (distance < 1e-6) {
        dx = 1.0;
        dy = 0.0;
        distance = 1.0;
    }
    const double nx = dx / distance;
    const double ny = dy / distance;
    const double overlap = min_distance - distance;
    a.x_m -= nx * overlap * 0.5;
    a.y_m -= ny * overlap * 0.5;
    b.x_m += nx * overlap * 0.5;
    b.y_m += ny * overlap * 0.5;

    const double relative_velocity =
        (b.velocity_x_mps - a.velocity_x_mps) * nx + (b.velocity_y_mps - a.velocity_y_mps) * ny;
    if (relative_velocity < 0.0) {
        const double impulse = -relative_velocity * 0.5;
        a.velocity_x_mps -= nx * impulse;
        a.velocity_y_mps -= ny * impulse;
        b.velocity_x_mps += nx * impulse;
        b.velocity_y_mps += ny * impulse;
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
