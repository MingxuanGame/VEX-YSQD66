#include "control/drive_controller.hpp"
#include "config/motion_config.hpp"
#include "io/input_mapping.hpp"
#include "io/input_state.hpp"
#include "robot/battery.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"
#include "vxmath/math_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace control {

namespace {

int clampOutput(int value, int limit) {
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

int limitDelta(int target, int previous, int max_delta) {
    if (max_delta <= 0)
        return target;
    const int delta = std::clamp(target - previous, -max_delta, max_delta);
    return previous + delta;
}

bool validBatteryMv(int voltage_mv) {
    // V5 电池的正常工作范围；超出时宁可不补偿，也不要把异常值放大成输出。
    return voltage_mv >= 6000 && voltage_mv <= 16000;
}

} // namespace

DriveController::DriveController(robot::Chassis &chassis, const DriveConfig &config)
    : m_chassis(&chassis), m_imu(nullptr), m_battery(nullptr), m_config(config),
      m_escape_active(false), m_escape_start_ms(0), m_push_start_ms(0), m_last_left(0),
      m_last_right(0), m_active_max_output(config.max_output), m_assist_active(false),
      m_assist_target_deg(0.0), m_assist_target_valid(false), m_aux_current_ma(0.0),
      m_last_current_scale(1.0) {}

void DriveController::update(const io::InputState &input, uint32_t now_ms) {
    if (!m_chassis)
        return;

    if (!input.isConnected) {
        stop();
        return;
    }
    if (!m_chassis->isHealthy()) {
        stop();
        return;
    }

    // 物理底牌：按住 L1+L2 时无视摇杆，执行受限的脱困摆动。
    if (input.l1.pressed() && input.l2.pressed()) {
        updateEscape(now_ms);
        return;
    }
    m_escape_active = false;
    m_escape_start_ms = 0;

    const int forward = io::processJoystick(input.leftY, m_config.deadzone, m_config.expo);
    int turn = io::processJoystick(input.rightX, m_config.deadzone, m_config.expo);

    // 航向保持 Assist：按住 X 时忽略手动摇杆的 turn，锁住按下瞬间的航向。
    // 注意：这里做的是“保持当前航向”，真正的锥体目标航向应由视觉/距离传感器提供。
    if (m_imu && input.x.pressed() && m_imu->isHealthy()) {
        const double current_heading = m_imu->getHeading();
        if (vxmath::isValidAngle(current_heading)) {
            if (!m_assist_active) {
                if (!m_assist_target_valid) {
                    m_assist_target_deg = current_heading;
                }
                m_assist_active = true;
            }
            const double error = vxmath::getError(current_heading, m_assist_target_deg);
            const double correction = vxmath::clampVal(
                m_config.assist_kp * error, -static_cast<double>(m_config.assist_max_turn),
                static_cast<double>(m_config.assist_max_turn));
            // 差速：turn > 0 会让左轮更快、航向减小，所以 assist 要取反。
            turn = -static_cast<int>(correction);
        }
    } else {
        m_assist_active = false;
        m_assist_target_valid = false;
    }

    int left = forward + turn;
    int right = forward - turn;

    // 左右轮先按 forward±turn 合成。若峰值超过安全上限，必须让两轮
    // 除以同一个 peak 做等比例缩放，而不能分别 clamp：
    // 分别限幅会改变 left/right 的差值，差速比一变，转向半径和轨迹就会变形。
    const int peak = (std::abs(left) > std::abs(right)) ? std::abs(left) : std::abs(right);
    if (peak > m_active_max_output) {
        left = left * m_active_max_output / peak;
        right = right * m_active_max_output / peak;
    }

    // 防推挤：操作手没有明显推杆时才卸力；一旦猛推，立刻交还控制权。
    const bool operator_hard = std::abs(input.leftY) > m_config.hard_push_threshold ||
                               std::abs(input.rightX) > m_config.hard_push_threshold;

    // 反馈每周期只采样一次；无效值不参与防推挤和电流预算。
    const double raw_left_current = m_chassis->getLeftCurrent();
    const double raw_right_current = m_chassis->getRightCurrent();
    const double raw_left_velocity = m_chassis->getLeftVelocity();
    const double raw_right_velocity = m_chassis->getRightVelocity();
    const double left_current = std::isfinite(raw_left_current) ? raw_left_current : 0.0;
    const double right_current = std::isfinite(raw_right_current) ? raw_right_current : 0.0;
    const double left_velocity = std::isfinite(raw_left_velocity) ? raw_left_velocity : 0.0;
    const double right_velocity = std::isfinite(raw_right_velocity) ? raw_right_velocity : 0.0;
    const double current = (left_current + right_current) / 2.0;
    const double velocity = (left_velocity + right_velocity) / 2.0;

    bool anti_push = false;
    if (!operator_hard && current > m_config.anti_push_current_ma &&
        std::abs(velocity) < m_config.anti_push_velocity_rpm) {
        if (m_push_start_ms == 0) {
            m_push_start_ms = (now_ms == 0) ? 1 : now_ms;
        } else if (static_cast<int32_t>(now_ms - m_push_start_ms) > m_config.anti_push_time_ms) {
            anti_push = true;
        }
    } else {
        m_push_start_ms = 0;
    }

    if (anti_push) {
        left = m_config.anti_push_output;
        right = m_config.anti_push_output;
    }

    // 手动模式也做电压补偿：低电量时按 12V/实际电压放大输出。
    // 先补偿再做电流预算，避免补偿后的输出绕过总电流限制。
    if (m_battery) {
        const int voltage_mv = m_battery->getVoltageMv();
        if (validBatteryMv(voltage_mv)) {
            double scale = static_cast<double>(cfg::kNominalBatteryMv) / voltage_mv;
            if (scale < 1.0)
                scale = 1.0;
            if (scale > cfg::kMaxBatteryCompensation) {
                scale = cfg::kMaxBatteryCompensation;
            }
            left = static_cast<int>(std::lround(left * scale));
            right = static_cast<int>(std::lround(right * scale));
        }
    }

    // 全局电流预算：四轮电流 + 机构预留。超预算时等比例缩小两轮，
    // 保持左右差速比不变；机构电流由 Mechanism::getCurrentMa() 提供。
    m_last_current_scale = 1.0;
    if (m_config.max_total_current_ma > 0.0) {
        const double drive_current = std::abs(left_current) + std::abs(right_current);
        const double aux_current =
            std::isfinite(m_aux_current_ma) && m_aux_current_ma > 0.0 ? m_aux_current_ma : 0.0;
        const double effective_aux =
            std::max(aux_current,
                     m_config.aux_current_reserve_ma > 0.0 ? m_config.aux_current_reserve_ma : 0.0);
        const double total_current = drive_current + effective_aux;
        if (total_current > m_config.max_total_current_ma) {
            m_last_current_scale = m_config.max_total_current_ma / total_current;
            left = static_cast<int>(left * m_last_current_scale);
            right = static_cast<int>(right * m_last_current_scale);
        }
    }

    const int target_left = clampOutput(left, m_active_max_output);
    const int target_right = clampOutput(right, m_active_max_output);
    m_last_left = limitDelta(target_left, m_last_left, m_config.max_delta_per_update);
    m_last_right = limitDelta(target_right, m_last_right, m_config.max_delta_per_update);
    m_chassis->setDrive(m_last_left, m_last_right);
}

void DriveController::updateEscape(uint32_t now_ms) {
    if (!m_chassis)
        return;

    if (!m_escape_active) {
        m_escape_active = true;
        m_escape_start_ms = (now_ms == 0) ? 1 : now_ms;
    }

    const int speed = clampOutput(m_config.escape_max_output, 127);
    const int phase_ms = (m_config.escape_phase_ms > 0) ? m_config.escape_phase_ms : 150;

    // 三相循环：0=小幅后退，1=左反/右正甩尾，2=左正/右反甩尾。
    // 150ms 是电机正反转的折中：太短会频繁反向、电流冲击大且发热，
    // 太长又拔不出来；用 now 取模切相，而不是 delay，保证主循环不阻塞。
    const int phase =
        static_cast<int>((now_ms - m_escape_start_ms) / static_cast<uint32_t>(phase_ms)) % 3;

    int left = 0;
    int right = 0;
    switch (phase) {
    case 0:
        // 小幅后退
        left = -speed;
        right = -speed;
        break;
    case 1:
        // 反向甩尾
        left = -speed;
        right = speed;
        break;
    default:
        // 反向甩尾
        left = speed;
        right = -speed;
        break;
    }

    m_last_left = left;
    m_last_right = right;
    m_chassis->setDrive(left, right);
}

void DriveController::stop() {
    if (m_chassis)
        m_chassis->stop(true);
    m_escape_active = false;
    m_escape_start_ms = 0;
    m_push_start_ms = 0;
    m_last_left = 0;
    m_last_right = 0;
    m_assist_active = false;
    m_assist_target_valid = false;
    m_aux_current_ma = 0.0;
    m_last_current_scale = 1.0;
}

void DriveController::setConfig(const DriveConfig &config) {
    m_config = config;
    m_active_max_output = config.max_output;
}

const DriveConfig &DriveController::getConfig() const {
    return m_config;
}

void DriveController::setMaxOutput(int max_output) {
    if (max_output < 0)
        max_output = 0;
    if (max_output > 127)
        max_output = 127;
    m_active_max_output = max_output;
}

void DriveController::setImu(robot::Imu *imu) {
    m_imu = imu;
    m_assist_active = false;
    m_assist_target_valid = false;
}

void DriveController::setBattery(robot::Battery *battery) {
    m_battery = battery;
}

void DriveController::setAuxCurrentMa(double current_ma) {
    m_aux_current_ma = (std::isfinite(current_ma) && current_ma > 0.0) ? current_ma : 0.0;
}

void DriveController::setAssistTargetHeading(double heading_deg) {
    m_assist_target_deg = heading_deg;
    m_assist_target_valid = true;
}

void DriveController::clearAssistTarget() {
    m_assist_target_valid = false;
    m_assist_active = false;
}

int DriveController::getLastLeftOutput() const {
    return m_last_left;
}

int DriveController::getLastRightOutput() const {
    return m_last_right;
}

bool DriveController::isAssistActive() const {
    return m_assist_active;
}

double DriveController::getLastCurrentLimitScale() const {
    return m_last_current_scale;
}

} // namespace control
