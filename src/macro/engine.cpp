#include "macro/engine.hpp"

#include <algorithm>
#include <cstdlib>

namespace macrorec {

Command map_input(const Input &input) {
    // TODO：其他机构接入后在此增加经限位处理的组件指令，并升级宏协议/映射版本。
    // 当前 left_x、right_y 及全部按键仅作为原始记录，不控制未安装的机构。
    if (!input.connected) {
        return {};
    }
    const int forward = -std::clamp(input.left_y, -127, 127);
    const int turn = -std::clamp(input.right_x, -127, 127);
    const int left = forward + turn;
    const int right = forward - turn;
    const int scale = std::max({127, std::abs(left), std::abs(right)});
    return {left * 127 / scale, right * 127 / scale};
}

bool valid_frame(const Frame &frame) {
    const auto in_range = [](int value) { return value >= -127 && value <= 127; };
    return in_range(frame.input.left_x) && in_range(frame.input.left_y) &&
           in_range(frame.input.right_x) && in_range(frame.input.right_y) &&
           (frame.input.buttons & ~kButtonMask) == 0 &&
           in_range(frame.command.chassis_left_power) &&
           in_range(frame.command.chassis_right_power);
}

Engine::Engine(Hardware &hardware) : m_hardware(hardware) {}

bool Engine::start(std::uint32_t now_ms, bool recording) {
    if (busy()) {
        return false;
    }
    m_hardware.stop();
    m_recording = recording;
    m_started_ms = now_ms;
    m_last_update_ms = now_ms;
    m_frame_count = 0;
    m_state = State::Countdown;
    m_status = "INPUT LOCKED";
    return true;
}

bool Engine::start_recording(std::uint32_t now_ms) {
    return start(now_ms, true);
}

bool Engine::start_replay(const Clip &clip, std::uint32_t now_ms) {
    if (busy()) {
        return false;
    }
    for (const auto &frame : clip) {
        if (!valid_frame(frame)) {
            fail("INVALID FRAME");
            return false;
        }
    }
    m_clip = clip;
    return start(now_ms, false);
}

void Engine::fail(const char *reason) {
    m_hardware.stop();
    m_state = State::Fault;
    m_status = reason;
}

void Engine::update(std::uint32_t now_ms, bool allowed, const Input &input) {
    if (!busy()) {
        return;
    }
    if (!allowed) {
        fail("DISABLED / MODE CHANGED");
        return;
    }
    if (now_ms - m_last_update_ms > 2 * kPeriodMs) {
        fail("CONTROL LOOP STALLED");
        return;
    }
    m_last_update_ms = now_ms;
    const auto elapsed = now_ms - m_started_ms;
    if (elapsed < kCountdownMs) {
        m_hardware.stop();
        return;
    }
    auto active_ms = elapsed - kCountdownMs;
    if (m_state == State::Countdown) {
        if (active_ms >= kPeriodMs) {
            fail("MISSED START FRAME");
            return;
        }
        // 倒计时跨过边界的首个 20ms 调度点作为帧零，避免点击发生于周期中间造成相位偏差。
        m_started_ms = now_ms - kCountdownMs;
        active_ms = 0;
    }
    if (active_ms >= kRecordMs) {
        m_hardware.stop();
        if (m_frame_count != kFrameCount) {
            fail("MISSED FRAME");
            return;
        }
        m_state = active_ms < kRecordMs + kResetMs ? State::Resetting : State::Complete;
        m_status = m_state == State::Resetting ? "RESET: OUTPUT ZERO" : "COMPLETE";
        return;
    }
    if (m_frame_count == kFrameCount) {
        return;
    }
    const int phase_error_ms =
        static_cast<int>(active_ms) - static_cast<int>(m_frame_count * kPeriodMs);
    if (phase_error_ms >= static_cast<int>(kPeriodMs)) {
        // 不能靠补写同一输入伪造丢失的帧，也不能快速追赶回放使动作时间失真。
        fail("MISSED FRAME");
        return;
    }
    // PROS 的周期唤醒及手柄读取可能有数 ms 抖动；容许最多 5ms 提前，仍然每帧只执行一次。
    constexpr int kEarlyToleranceMs = 5;
    if (phase_error_ms < -kEarlyToleranceMs) {
        return;
    }
    const auto index = m_frame_count;
    if (m_recording) {
        if (!input.connected || !valid_frame({input, {}})) {
            fail("CONTROLLER LOST / INVALID");
            return;
        }
        m_clip[index] = {input, map_input(input)};
        m_state = State::Recording;
        m_status = "RECORDING";
    } else {
        m_state = State::Replaying;
        m_status = "REPLAYING";
    }
    if (!m_hardware.apply(m_clip[index].command)) {
        fail("COMPONENT WRITE FAILED");
        return;
    }
    ++m_frame_count;
}

void Engine::cancel() {
    m_hardware.stop();
    m_state = State::Cancelled;
    m_frame_count = 0;
    m_status = "STOPPED";
}

bool Engine::busy() const {
    return m_state == State::Countdown || m_state == State::Recording ||
           m_state == State::Replaying || m_state == State::Resetting;
}

State Engine::state() const {
    return m_state;
}

bool Engine::recording() const {
    return m_recording;
}

std::size_t Engine::frame_count() const {
    return m_frame_count;
}

const Clip &Engine::clip() const {
    return m_clip;
}

std::uint32_t Engine::elapsed_ms(std::uint32_t now_ms) const {
    const auto elapsed = now_ms - m_started_ms;
    if (m_state == State::Countdown) {
        return std::min(elapsed, kCountdownMs);
    }
    if (m_state == State::Resetting || m_state == State::Complete) {
        return std::min(elapsed - kCountdownMs - kRecordMs, kResetMs);
    }
    return std::min(elapsed - kCountdownMs, kRecordMs);
}

const char *Engine::status() const {
    return m_status;
}

} // namespace macrorec
