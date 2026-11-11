#pragma once
#include "config/motion_config.hpp"
#include "sim/core/sim_rng.hpp"
#include "sim/hardware/sim_imu.hpp"
#include "sim/hardware/sim_motor.hpp"

/**
 * @file chassis_sim.hpp
 * @brief 底盘物理仿真引擎（带 IMU 动力学联动）。
 *        负责计算四轮合力、整车质量、碰撞阻力，并将差速转弯积分推送给 IMU。
 */
namespace sim_physics {

enum class CollisionObject {
    None,        // 正常行驶
    Wall,        // 撞墙/死区（绝对阻力，车停死，电流飙升）
    OpponentCar, // 撞对方车（带动量守恒，会导致电流剧烈波动）
    Cone         // 撞锥（轻阻力，推开后消失，车会猛地窜一下）
};

class ChassisSim {
public:
    /**
     * @param lf 左前, lb 左后, rf 右前, rb 右后
     * @param imu IMU 虚拟指针，用于接收物理角速度
     */
    ChassisSim(sim::Motor *lf, sim::Motor *lb, sim::Motor *rf, sim::Motor *rb, sim::Imu *imu);

    // 物理更新（由 Clock 的 tick 每 1ms 自动调用）
    void step(double dt_sec);

    // 场景/积分状态复位；不重置电机端口和 IMU 硬件对象。
    void reset();
    void setPose(double x_m, double y_m, double heading_deg);
    void setRandomSeed(uint32_t seed);

    // ==================== 场景注入接口 ====================
    void setCollisionObject(CollisionObject obj);
    void setBeingPushedByOpponent(bool pushed, double push_force_nm = 1.5);
    // 兼容旧调用。
    void set_collision_object(CollisionObject obj);
    void set_being_pushed_by_opponent(bool pushed, double push_force_nm = 1.5);

    // 获取当前底盘宏观状态（用于调试/回放）
    double getChassisVelocityMps() const;
    double getTrueAngularVelDps() const;
    double getPoseXMeters() const;
    double getPoseYMeters() const;
    double getPoseHeadingDeg() const;
    double getTotalCurrentMa() const;

    // 兼容旧仿真调用；新代码使用上面的 camelCase 接口。
    double get_chassis_velocity_mps() const;
    double get_true_angular_vel_dps() const;

private:
    sim::Motor *m_motors[4];
    sim::Imu *m_imu;

    // ==================== 车辆物理参数 ====================
    const double kMassKg = 12.0; // 整车质量
    // 与 config/motion_config.hpp 共用同一份几何参数，避免真车/仿真漂移
    const double kWheelRadiusM = cfg::kWheelDiameterInch * 0.0254 / 2.0;
    const double kTrackWidthM = cfg::kTrackWidthInch * 0.0254;

    // ==================== 状态变量 ====================
    double m_chassis_vel_mps = 0.0;      // 底盘线速度（m/s）
    double m_true_angular_vel_dps = 0.0; // 真实的角速度（度/秒）
    double m_pose_x_m = 0.0;
    double m_pose_y_m = 0.0;
    double m_pose_heading_deg = 0.0;
    double m_total_current_ma = 0.0;
    uint32_t m_seed = 0x5EED1234u;
    sim::XorShift32 m_rng;

    // ==================== 场景状态 ====================
    CollisionObject m_current_object = CollisionObject::None;
    bool m_is_being_pushed = false;
    double m_push_force_nm = 0.0;

    double m_opponent_car_velocity = 0.0;
    const double kOpponentCarMass = 10.0;

    double m_cone_velocity = 0.0;
    const double kConeMass = 0.2;
};
} // namespace sim_physics
