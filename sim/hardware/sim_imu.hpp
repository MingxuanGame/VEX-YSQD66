#pragma once
#include <cstdint>
#include "sim/core/sim_rng.hpp"

/**
 * @file sim_imu.hpp
 * @brief 虚拟 V5 IMU。
 *        基于底盘的实时角速度进行物理积分，并注入真实的白噪声与漂移，用于测试滤波和 PID。
 */
namespace sim {

class Imu {
public:
    /**
     * @param port 虚拟端口
     */
    Imu(int port);

    // ==================== 业务控制接口 ====================
    /**
     * @brief 重置并校准 IMU
     * @param wait_for_calibration 是否阻塞等待校准完成（仿真里瞬间完成）
     * @note 会自动清空当前的航向角，通常用于自动赛开始前。
     */
    void reset(bool wait_for_calibration = true);
    bool is_calibrating() const;

    // ==================== 传感器读取接口 ====================
    double get_heading() const;            // 0 ~ 360 度
    double get_rotation() const;           // -inf ~ inf 累积角度
    double get_gyro_rate(char axis) const; // 角速度，仅支持 'z' 轴

    // ==================== 物理引擎注入接口 ====================
    /**
     * @brief 物理步进更新（由底盘物理引擎每 dt 秒调用）
     * @param dt_sec 步进时间
     * @param true_angular_vel_dps 真实的底盘角速度（度/秒），由左右轮差速计算得出
     */
    void physics_update(double dt_sec, double true_angular_vel_dps);

    /**
     * @brief 设置 IMU 的噪声级别
     * @param noise_deg 标准差（度数）。设为 0 即为无噪声的完美传感器。
     */
    void setNoiseLevel(double noise_deg);

    // 兼容旧仿真代码；新代码使用 setNoiseLevel。
    void set_noise_level(double noise_deg);

    // 固定噪声序列，便于复现同一场景。
    void setRandomSeed(uint32_t seed);

private:
    int m_port;
    bool m_calibrating;

    double m_true_heading_deg;     // 真实的物理朝向（上帝视角）
    double m_true_angular_vel_dps; // 真实的物理角速度
    double m_accumulated_drift;    // 累积漂移量

    // 传感器特性的模拟参数
    double m_noise_std_dev_deg = 0.1;         // 默认 0.1 度的白噪声
    const double kDriftRateDegPerSec = 0.001; // 每秒漂移 0.001 度
    mutable XorShift32 m_rng;

    // 辅助函数：生成高斯白噪声
    double generateGaussianNoise(double mean, double stddev) const;
};

} // namespace sim
