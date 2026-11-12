#pragma once
#include "robot/mechanism.hpp"
#include "sim/config/sim_config.hpp"

namespace sim {

/**
 * @brief 级联抬升 + 舵机夹爪 + 推板的仿真机构。
 *
 * 物理步进由主循环调用 update(dt) 推进；Mechanism 接口只负责接收命令。
 */
class SimMechanism : public robot::Mechanism {
public:
    explicit SimMechanism(const SimConfig &config = defaultSimConfig());

    void update(double dt_sec);
    void reset();

    void liftTo(double pos, int max_speed = 100) override;
    void liftManual(int speed) override;
    void liftHold() override;

    void clampOpen() override;
    void clampClose() override;
    void clampSetAngle(int angle) override;

    void pushExtend() override;
    void pushRetract() override;

    double getCurrentMa() const override;

    // 供 UI / 测试读取。
    double getLiftHeightM() const;
    double getClampAngleDeg() const;
    double getPushPosition() const;
    double getCascadeStage() const;
    bool isLiftMoving() const;

private:
    void updateLift(double dt_sec);
    void updateClamp(double dt_sec);
    void updatePush(double dt_sec);

    SimConfig m_config;

    double m_lift_height_m;
    double m_lift_velocity_mps;
    double m_lift_target_m;
    double m_lift_speed_limit_mps;
    bool m_lift_position_mode;
    bool m_lift_holding;

    double m_clamp_angle_deg;
    double m_clamp_target_deg;

    double m_push_position;
    double m_push_target;

    double m_current_ma;
};

} // namespace sim
