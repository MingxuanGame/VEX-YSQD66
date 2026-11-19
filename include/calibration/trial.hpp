#pragma once

#include "calibration/measurements.hpp"
#include "macro/engine.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace calibration {

enum class TrialState { Idle, Running, Settling, Complete, Fault };

// 非阻塞练习测量。由手动调度层检查场控、手柄和按住运行许可，每 20ms 服务一次。
class Trial {
public:
    explicit Trial(macrorec::Hardware &hardware);

    // 读取当前位置作为角度参考，启动配置中指定的测量。时间为系统毫秒。
    void start(const TrialConfig &config, std::uint32_t now_ms);
    void update(std::uint32_t now_ms);
    // 立即刹车，取消读取和输出。
    void cancel();

    TrialState state() const;
    // Complete 时结果可用于标定，运行中可显示当前累计角度和估计行程。
    const TrialResult &result() const;
    const char *status() const;

private:
    struct Stamp {
        macrorec::Feedback feedback{};
        std::uint32_t time_ms = 0;
    };

    void fail(const char *message);
    void settle(const Stamp &sample);
    void read_motion(const Stamp &sample);
    void update_settling(const Stamp &sample);
    void update_breakaway(const Stamp &sample);
    void update_speed_window(const Stamp &sample, const std::array<std::uint32_t, 3> &marks_ms);

    macrorec::Hardware &m_hardware;
    TrialConfig m_config{};
    TrialResult m_result{};
    TrialState m_state = TrialState::Idle;
    const char *m_status = "Ready";
    Stamp m_origin{};
    Stamp m_previous{};
    // 起动力档位基准。进入停车阶段后用作静止窗口的基准。
    Stamp m_window_start{};
    std::uint32_t m_stop_ms = 0;
    int m_breakaway_power = 1;
    std::array<Stamp, 3> m_speed_marks{};
    std::size_t m_speed_mark_count = 0;
};

} // namespace calibration
