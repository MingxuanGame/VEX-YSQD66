#pragma once

/**
 * @file speed_profile.hpp
 * @brief 梯形速度规划。
 *        为什么需要它？直接给 127 满速会导致底盘瞬间打滑，
 *        这个类能让速度平滑加速、保持、平滑减速，保护电机和齿轮。
 */

namespace vxmath {

class TrapezoidProfile {
public:
    TrapezoidProfile();

    /**
     * @param distance 目标总距离
     * @param maxVel 最大速度限制
     * @param accel 加速度（决定起步有多柔和）
     */
    TrapezoidProfile(double distance, double maxVel, double accel);

    // 运行期重新配置（用于非阻塞状态机复用同一个 profile 对象）
    void configure(double distance, double maxVel, double accel);

    // 根据已经走过的距离，计算当前应该给出的目标速度
    double getVelocity(double traveled) const;

    // 返回 0.0 ~ 1.0 的速度比例，用于直接乘上电机最大输出
    double getVelocityRatio(double traveled) const;

    bool isDone(double traveled) const;

private:
    double m_d;
    double m_vmax;
    double m_a;
    double m_da; // 加速段距离
    double m_tc; // 匀速段距离
};

} // namespace vxmath
