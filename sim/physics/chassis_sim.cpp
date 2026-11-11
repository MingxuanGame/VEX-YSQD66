#include "chassis_sim.hpp"
#include "sim/hardware/sim_battery.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace sim_physics {

ChassisSim::ChassisSim(sim::Motor *lf, sim::Motor *lb, sim::Motor *rf, sim::Motor *rb,
                       sim::Imu *imu)
    : m_imu(imu), m_rng(0x5EED1234u) {
    m_motors[0] = lf;
    m_motors[1] = lb;
    m_motors[2] = rf;
    m_motors[3] = rb;
}

void ChassisSim::setRandomSeed(uint32_t seed) {
    m_seed = seed;
    m_rng.seed(seed);
}

void ChassisSim::reset() {
    for (int i = 0; i < 4; ++i) {
        if (m_motors[i])
            m_motors[i]->reset();
    }
    if (m_imu)
        m_imu->reset(true);

    m_chassis_vel_mps = 0.0;
    m_true_angular_vel_dps = 0.0;
    m_pose_x_m = 0.0;
    m_pose_y_m = 0.0;
    m_pose_heading_deg = 0.0;
    m_total_current_ma = 0.0;
    m_current_object = CollisionObject::None;
    m_is_being_pushed = false;
    m_push_force_nm = 0.0;
    m_opponent_car_velocity = 0.0;
    m_cone_velocity = 0.0;
    m_rng.seed(m_seed);
    sim::Battery::updateLoad(0.0);
}

void ChassisSim::setPose(double x_m, double y_m, double heading_deg) {
    m_pose_x_m = x_m;
    m_pose_y_m = y_m;
    m_pose_heading_deg = heading_deg;
}

void ChassisSim::setCollisionObject(CollisionObject obj) {
    m_current_object = obj;
    if (obj == CollisionObject::OpponentCar)
        m_opponent_car_velocity = 0.0;
    else if (obj == CollisionObject::Cone)
        m_cone_velocity = 0.0;
}

void ChassisSim::setBeingPushedByOpponent(bool pushed, double push_force_nm) {
    m_is_being_pushed = pushed;
    m_push_force_nm = push_force_nm;
}

void ChassisSim::set_collision_object(CollisionObject obj) {
    setCollisionObject(obj);
}

void ChassisSim::set_being_pushed_by_opponent(bool pushed, double push_force_nm) {
    setBeingPushedByOpponent(pushed, push_force_nm);
}

double ChassisSim::getChassisVelocityMps() const {
    return m_chassis_vel_mps;
}

double ChassisSim::getTrueAngularVelDps() const {
    return m_true_angular_vel_dps;
}

double ChassisSim::getPoseXMeters() const {
    return m_pose_x_m;
}

double ChassisSim::getPoseYMeters() const {
    return m_pose_y_m;
}

double ChassisSim::getPoseHeadingDeg() const {
    return m_pose_heading_deg;
}

double ChassisSim::getTotalCurrentMa() const {
    return m_total_current_ma;
}

// Legacy simulation API. New code should use the camelCase names above.
double ChassisSim::get_chassis_velocity_mps() const {
    return m_chassis_vel_mps;
}

double ChassisSim::get_true_angular_vel_dps() const {
    return m_true_angular_vel_dps;
}

void ChassisSim::step(double dt_sec) {
    if (!m_motors[0] || !m_motors[1] || !m_motors[2] || !m_motors[3])
        return;

    // 1. 获取四个轮子的真实转速
    double rpm_l = (m_motors[0]->get_actual_velocity() + m_motors[1]->get_actual_velocity()) / 2.0;
    double rpm_r = (m_motors[2]->get_actual_velocity() + m_motors[3]->get_actual_velocity()) / 2.0;

    // 2. 转速转换为线速度 (m/s)
    double wheel_circumference = 2.0 * 3.1415926535 * kWheelRadiusM;
    double v_l = (rpm_l / 60.0) * wheel_circumference;
    double v_r = (rpm_r / 60.0) * wheel_circumference;

    // 底盘的宏观线速度（平均速度）
    m_chassis_vel_mps = (v_l + v_r) / 2.0;

    // 3. 根据环境计算阻力矩
    double external_resistance_nm = 0.0;

    switch (m_current_object) {
    case CollisionObject::Wall:
        // 撞墙：整车直接卡死，速度强行归零
        m_chassis_vel_mps = 0;
        external_resistance_nm = 100.0; // 给一个巨大的阻力，电机必定堵转
        break;

    case CollisionObject::OpponentCar:
        // 撞车：动量守恒与对抗
        if (m_chassis_vel_mps > 0.01) {
            double relative_vel = m_chassis_vel_mps - m_opponent_car_velocity;
            if (relative_vel > 0) {
                double force = relative_vel * 20.0;
                m_opponent_car_velocity += (force / kOpponentCarMass) * dt_sec;
                external_resistance_nm = force * kWheelRadiusM;
            } else {
                external_resistance_nm = 2.0 + static_cast<double>(m_rng.next() % 100) / 100.0;
                m_chassis_vel_mps = 0;
            }
        } else {
            external_resistance_nm = 3.0; // 顶住不动
        }
        break;

    case CollisionObject::Cone:
        // 撞锥：轻质物体，推力大于阻力就飞
        if (m_chassis_vel_mps > 0.01) {
            double relative_vel = m_chassis_vel_mps - m_cone_velocity;
            if (relative_vel > 0) {
                double force = relative_vel * 2.0;
                m_cone_velocity += (force / kConeMass) * dt_sec;
                external_resistance_nm = force * kWheelRadiusM;
                if (m_cone_velocity > m_chassis_vel_mps * 0.8) {
                    m_current_object = CollisionObject::None; // 锥被推开
                }
            }
        }
        break;

    case CollisionObject::None:
    default:
        // 正常行驶的滚动摩擦
        external_resistance_nm = std::abs(m_chassis_vel_mps) * 0.5;
        break;
    }

    if (m_is_being_pushed) {
        external_resistance_nm += m_push_force_nm;
    }

    // 4. 将阻力平分给四个电机进行物理更新
    double resistance_per_motor = external_resistance_nm / 4.0;
    for (int i = 0; i < 4; ++i) {
        m_motors[i]->physics_update(dt_sec, resistance_per_motor);
    }

    // 5. 用更新后的轮速计算底盘速度和角速度
    const double rpm_l_new =
        (m_motors[0]->get_actual_velocity() + m_motors[1]->get_actual_velocity()) / 2.0;
    const double rpm_r_new =
        (m_motors[2]->get_actual_velocity() + m_motors[3]->get_actual_velocity()) / 2.0;
    const double v_l_new = (rpm_l_new / 60.0) * wheel_circumference;
    const double v_r_new = (rpm_r_new / 60.0) * wheel_circumference;
    m_chassis_vel_mps = (v_l_new + v_r_new) / 2.0;

    // 转弯角速度 = (右轮速 - 左轮速) / 轮距
    const double omega_rad_s = (v_r_new - v_l_new) / kTrackWidthM;
    m_true_angular_vel_dps = omega_rad_s * (180.0 / 3.1415926535);

    // 6. 积分仿真位姿，供回放/可视化/定位测试使用。
    const double heading_rad = m_pose_heading_deg * 3.1415926535 / 180.0;
    m_pose_x_m += m_chassis_vel_mps * std::cos(heading_rad) * dt_sec;
    m_pose_y_m += m_chassis_vel_mps * std::sin(heading_rad) * dt_sec;
    m_pose_heading_deg += m_true_angular_vel_dps * dt_sec;

    if (m_imu)
        m_imu->physics_update(dt_sec, m_true_angular_vel_dps);

    // 7. 汇总四轮电流，供下一帧电池内阻压降使用。
    m_total_current_ma =
        std::abs(m_motors[0]->get_current_draw()) + std::abs(m_motors[1]->get_current_draw()) +
        std::abs(m_motors[2]->get_current_draw()) + std::abs(m_motors[3]->get_current_draw());
    sim::Battery::updateLoad(m_total_current_ma);
}

} // namespace sim_physics
