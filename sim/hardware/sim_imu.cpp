#include "sim_imu.hpp"
#include <cmath>

namespace sim {

Imu::Imu(int port)
    : m_port(port), m_calibrating(false), m_true_heading_deg(0), m_true_angular_vel_dps(0),
      m_accumulated_drift(0), m_rng(static_cast<uint32_t>(port) * 2246822519u + 0x9E3779B9u) {}

void Imu::reset(bool wait_for_calibration) {
    // 仿真里校准是瞬间完成的；wait_for_calibration 仅为接口兼容。
    (void)wait_for_calibration;
    m_calibrating = true;
    m_true_heading_deg = 0.0;
    m_accumulated_drift = 0.0;
    m_true_angular_vel_dps = 0.0;
    m_calibrating = false;
}

bool Imu::is_calibrating() const {
    return m_calibrating;
}

double Imu::get_heading() const {
    // 真实朝向加上由于积分产生的漂移
    double raw_heading = m_true_heading_deg + m_accumulated_drift;

    // 叠加传感器底噪（每次读取都抖动一下）
    double noisy_heading = raw_heading + generateGaussianNoise(0.0, m_noise_std_dev_deg);

    // 归一化到 0~360
    noisy_heading = std::fmod(noisy_heading, 360.0);
    if (noisy_heading < 0)
        noisy_heading += 360.0;
    return noisy_heading;
}

double Imu::get_rotation() const {
    // 累积角度通常是 -inf 到 inf，不需要归一化，但同样要加噪声和漂移
    double raw_rotation = m_true_heading_deg + m_accumulated_drift;
    return raw_rotation + generateGaussianNoise(0.0, m_noise_std_dev_deg);
}

double Imu::get_gyro_rate(char axis) const {
    if (axis != 'z' && axis != 'Z')
        return 0.0;
    // 角速度同样带有噪声
    return m_true_angular_vel_dps + generateGaussianNoise(0.0, m_noise_std_dev_deg * 10.0);
}

void Imu::physics_update(double dt_sec, double true_angular_vel_dps) {
    m_true_angular_vel_dps = true_angular_vel_dps;

    // 积分计算真实朝向：角度 += 角速度 * 时间
    m_true_heading_deg += m_true_angular_vel_dps * dt_sec;

    // 模拟缓慢的物理漂移（温度变化、积分误差）
    m_accumulated_drift += kDriftRateDegPerSec * dt_sec;
}

void Imu::setNoiseLevel(double noise_deg) {
    m_noise_std_dev_deg = noise_deg;
}

void Imu::set_noise_level(double noise_deg) {
    setNoiseLevel(noise_deg);
}

void Imu::setRandomSeed(uint32_t seed) {
    m_rng.seed(seed);
}

// 使用成员 RNG 生成高斯白噪声，避免全局 rand 和静态缓存破坏可复现性。
double Imu::generateGaussianNoise(double mean, double stddev) const {
    return m_rng.gaussian(mean, stddev);
}

} // namespace sim
