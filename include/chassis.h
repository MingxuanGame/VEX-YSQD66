#pragma once

#include <array>
#include <cstdint>

namespace chassis {

// 最近一次传感器初始化、清零或读取失败的诊断快照。
struct SensorDiagnostics {
    bool has_error = false;
    const char *stage = "OK";
    bool imu_calibrated = false;
    int imu_status = 0;
    int imu_errno = 0;
    int imu_reset_ret = 1;
    std::uint32_t calib_ms = 0;
    double imu_rotation_deg = 0.0;
    std::array<double, 4> motor_position_deg{};
    std::array<int, 4> motor_errno{};
    int left_tare_errno = 0;
    int right_tare_errno = 0;
};

// 初始化四电机底盘与端口 5 的 IMU；停车校准最多等待 3s，不驱动车辆。
void initialize();

// 停止输出、保持刹车模式，清零四电机编码器和 IMU 转角参考，不重新校准或机械归零。
void reset_state();

// 非阻塞读取左前/左后/右前/右后电机轴角度及 IMU 连续转角（度，顺时针为正）。
// 未校准、正在校准或任一读取失败时返回 false，输出不可用。
bool read_feedback(std::array<double, 4> &motor_position_deg, double &imu_rotation_deg);

// 返回最近一次传感器状态或失败诊断信息。
const SensorDiagnostics &sensor_diagnostics();

// 应用左右侧开环功率 [-127, 127]，按补偿偏移限幅；禁用或自动阶段下保持停止。
void set_power(int left_power, int right_power);

// 自动专用开环功率 [-127, 127]，只在场控启用的自动时段输出。
void set_autonomous_power(int left_power, int right_power);

// 立即刹车停止四个驱动电机。
void stop();

} // namespace chassis
