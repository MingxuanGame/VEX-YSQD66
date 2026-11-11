#pragma once
#include <cstdint>

namespace io {
struct InputState;
} // namespace io

namespace robot {
class Chassis;
class Imu;
class Battery;
} // namespace robot

namespace control {

struct DriveConfig {
    int deadzone = 5;
    double expo = 1.3;
    int max_output = 100; // 蓝齿轮安全上限

    int anti_push_current_ma = 2000;
    int anti_push_velocity_rpm = 10;
    int anti_push_time_ms = 500;
    int anti_push_output = -40;   // 卸力时的反向输出
    int hard_push_threshold = 60; // 操作手推杆超过此值表示要硬刚

    int escape_max_output = 60; // 脱困输出不超过 60%
    int escape_phase_ms = 150;
    int tune_max_output = 35; // Tune 模式下按住 L1 的低速点动上限

    // 全局电流预算：四轮 + 机构的总电流不要顶到主控 20A 限流。
    double max_total_current_ma = 20000.0;
    double aux_current_reserve_ma = 3000.0; // 抬升/夹爪预留

    // 航向保持 Assist：按住 X 时锁住按下瞬间的航向，自动修 turn。
    double assist_kp = 2.0;
    int assist_max_turn = 40;
};

class DriveController {
public:
    explicit DriveController(robot::Chassis &chassis, const DriveConfig &config = {});

    // 手动驾驶一帧：断联 -> 急停；L1+L2 -> 脱困；否则正常差速。
    void update(const io::InputState &input, uint32_t now_ms);
    void stop();

    void setConfig(const DriveConfig &config);
    const DriveConfig &getConfig() const;
    void setMaxOutput(int max_output);

    void setImu(robot::Imu *imu);
    void setBattery(robot::Battery *battery);
    void setAuxCurrentMa(double current_ma);

    // 视觉/距离传感器检测到锥体后，可把目标航向注入 Assist。
    // 不注入时，按住 X 会锁住按下瞬间的当前航向。
    void setAssistTargetHeading(double heading_deg);
    void clearAssistTarget();

    int getLastLeftOutput() const;
    int getLastRightOutput() const;
    bool isAssistActive() const;
    double getLastCurrentLimitScale() const;

private:
    void updateEscape(uint32_t now_ms);
    robot::Chassis *m_chassis;
    robot::Imu *m_imu;
    robot::Battery *m_battery;
    DriveConfig m_config;

    bool m_escape_active;
    uint32_t m_escape_start_ms;
    uint32_t m_push_start_ms;
    int m_last_left;
    int m_last_right;
    int m_active_max_output;

    bool m_assist_active;
    double m_assist_target_deg;
    bool m_assist_target_valid;

    double m_aux_current_ma;
    double m_last_current_scale;
};

} // namespace control
