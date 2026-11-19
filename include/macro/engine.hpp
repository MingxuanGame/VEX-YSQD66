#pragma once

#include "math/pid.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace macrorec {

constexpr std::uint32_t kPeriodMs = 20;
constexpr std::uint32_t kCountdownMs = 3000;
// Override 2.1 Match 定义：15s 自动窗，末尾留出 500ms 停止与状态重置。
constexpr std::uint32_t kRecordMs = 14500;
constexpr std::uint32_t kResetMs = 500;
constexpr std::size_t kFrameCount = kRecordMs / kPeriodMs;
constexpr std::uint16_t kButtonMask = 0x0fff;
constexpr std::uint16_t kLiftDownButton = 1U << 5;   // B
constexpr std::uint16_t kLiftUpButton = 1U << 6;     // X
constexpr std::uint16_t kCheckpointButton = 1U << 8; // L1：按下沿标记检查点

// 手柄原始输入快照：四轴 [-127, 127]，按键为 12 位掩码。
struct Input {
    int left_x = 0;
    int left_y = 0;
    int right_x = 0;
    int right_y = 0;
    std::uint16_t buttons = 0;
    bool connected = false;
};

// 下发给执行器组件的最终指令，单位为 PROS move() 功率 [-127, 127]。
struct Command {
    int chassis_left_power = 0;
    int chassis_right_power = 0;
    int lift_power = 0;
};

// 帧开始、下发指令前的反馈；相对宏起点，电机依次为左前/左后/右前/右后。
// 电机为电机轴角度，IMU 为连续转角（度，顺时针为正，不折返到 0..360）。
struct Feedback {
    std::array<double, 4> motor_position_deg{};
    double imu_rotation_deg = 0.0;
};

struct Frame {
    Input input{};
    Command command{};
    Feedback feedback{};
    bool checkpoint = false;
};

// 传感器失败时的多行屏幕调试文本（单行按 57 字符以内格式化）。
struct DebugInfo {
    bool active = false;
    std::array<char, 64> summary{};
    std::array<char, 64> imu{};
    std::array<char, 64> encoders_l{};
    std::array<char, 64> encoders_r{};
    std::array<char, 64> encoders_all{};
};

using Clip = std::array<Frame, kFrameCount>;

// Arcade 混控和升降按键映射；断联时输出零。
Command map_input(const Input &input);

// 检查加载帧的轴范围、按键掩码、组件输出与反馈有限性。
bool valid_frame(const Frame &frame);

// 宏执行器依赖的硬件抽象接口；具体实现由适配层提供。
class Hardware {
public:
    virtual ~Hardware() = default;
    // 返回可写入宏帧的指令，升降功率包含限位和降速的结果。
    virtual Command apply(Command command) = 0;
    virtual void stop() = 0;
    virtual void reset() = 0;
    // 非阻塞读取原始反馈（度）；校准未完成或读取失败返回 false，不允许驱动宏。
    virtual bool read_feedback(Feedback &feedback) = 0;
    // 读取最近一次传感器失败诊断信息；无诊断数据时返回 false。
    virtual bool read_debug_info(DebugInfo &info) const;
};

enum class State {
    Idle,
    Countdown,
    Recording,
    Replaying,
    CheckpointPause,
    Resetting,
    Complete,
    Cancelled,
    Fault
};

// 非阻塞宏录放状态机：
// 3s 倒计时 -> 725 帧（14.5s，每帧 20ms）录制/回放 -> 0.5s 停车重置；
// 支持从检查点回放前缀、停车 3s 后再续录剩余帧。
class Engine {
public:
    explicit Engine(Hardware &hardware);

    bool start_recording(std::uint32_t now_ms);
    // frame_count 为检查点结束帧数；等于 kFrameCount 时只完整回放不续录。
    bool start_from_checkpoint(const Clip &clip, std::size_t frame_count, std::uint32_t now_ms);
    void update(std::uint32_t now_ms, bool allowed, const Input &input = {});
    void cancel();

    bool busy() const;
    State state() const;
    bool recording() const;
    std::size_t frame_count() const;
    const Clip &clip() const;
    std::uint32_t elapsed_ms(std::uint32_t now_ms) const;
    const char *status() const;
    const DebugInfo &debug_info() const;

private:
    bool start(std::uint32_t now_ms, bool recording);
    void fail(const char *reason);
    void capture_sensor_debug(const char *phase, std::size_t frame_index);
    void reset_pid();
    bool apply_replay(const Frame &frame, const Feedback &feedback);

    Hardware &m_hardware;
    Feedback m_origin{};
    math::Pid m_left_pid;
    math::Pid m_right_pid;
    math::Pid m_heading_pid;
    Clip m_clip{};
    DebugInfo m_debug_info{};
    State m_state = State::Idle;
    bool m_recording = false;
    bool m_resuming = false;
    bool m_checkpoint_button_down = false;
    std::size_t m_resume_frame_count = 0;
    std::size_t m_frame_count = 0;
    std::uint32_t m_phase_start_ms = 0;
    std::uint32_t m_last_frame_ms = 0;
    const char *m_status = "READY";
};

} // namespace macrorec
