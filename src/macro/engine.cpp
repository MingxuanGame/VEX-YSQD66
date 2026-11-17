#include "macro/engine.hpp"

#include <algorithm>
#include <cstdlib>

namespace macrorec {

Command map_input(const Input &input) {
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
    m_hardware.reset();
    if (recording) {
        m_clip.fill(Frame{});
    }
    m_recording = recording;
    m_resuming = false;
    m_checkpoint_button_down = false;
    m_resume_frame_count = 0;
    m_frame_count = 0;
    m_phase_start_ms = now_ms;
    m_last_frame_ms = 0;
    m_state = State::Countdown;
    m_status = "INPUT LOCKED";
    return true;
}

bool Engine::start_recording(std::uint32_t now_ms) {
    return start(now_ms, true);
}

bool Engine::start_from_checkpoint(const Clip &clip, std::size_t frame_count,
                                   std::uint32_t now_ms) {
    if (frame_count == 0 || frame_count > kFrameCount ||
        (frame_count < kFrameCount && !clip[frame_count - 1].checkpoint)) {
        return false;
    }
    m_clip = clip;
    if (!start(now_ms, false)) {
        return false;
    }
    if (frame_count < kFrameCount) {
        m_recording = true;
        m_resuming = true;
        m_resume_frame_count = frame_count;
        std::fill(m_clip.begin() + frame_count, m_clip.end(), Frame{});
    }
    return true;
}

void Engine::fail(const char *reason) {
    m_hardware.reset();
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
    if (m_state == State::Countdown) {
        m_hardware.stop();
        if (now_ms - m_phase_start_ms < kCountdownMs) {
            return;
        }
        m_hardware.reset();
        m_state = (m_recording && !m_resuming) ? State::Recording : State::Replaying;
        m_last_frame_ms = now_ms - kPeriodMs;
    }
    if (m_state == State::CheckpointPause) {
        m_hardware.stop();
        if (now_ms - m_phase_start_ms < kCountdownMs) {
            return;
        }
        // 在恢复录制的首帧重建 L1 电平基准，暂停期间按住不会误触发检查点。
        m_checkpoint_button_down = (input.buttons & kCheckpointButton) != 0;
        m_state = State::Recording;
        m_last_frame_ms = now_ms - kPeriodMs;
    }
    if (m_state == State::Resetting) {
        m_hardware.stop();
        if (now_ms - m_phase_start_ms >= kResetMs) {
            m_state = State::Complete;
            m_status = "COMPLETE";
        }
        return;
    }

    const auto elapsed_frame = now_ms - m_last_frame_ms;
    if (elapsed_frame < kPeriodMs) {
        return;
    }
    if (elapsed_frame > 2 * kPeriodMs) {
        fail("CONTROL LOOP STALLED");
        return;
    }

    if (m_resuming && m_state == State::Replaying && m_frame_count == m_resume_frame_count) {
        m_hardware.stop();
        m_phase_start_ms = now_ms;
        m_state = State::CheckpointPause;
        m_status = "CHECKPOINT: WAIT 3s";
        return;
    }
    if (m_frame_count == kFrameCount) {
        m_hardware.stop();
        m_hardware.reset();
        m_phase_start_ms = now_ms;
        m_state = State::Resetting;
        m_status = "RESET: OUTPUT ZERO";
        return;
    }

    const auto index = m_frame_count;
    if (m_state == State::Recording) {
        if (!input.connected) {
            fail("CONTROLLER LOST");
            return;
        }
        const bool l1_down = (input.buttons & kCheckpointButton) != 0;
        m_clip[index] = {input, map_input(input), l1_down && !m_checkpoint_button_down};
        m_checkpoint_button_down = l1_down;
        m_status = "RECORDING | L1: CHECKPOINT";
    } else {
        m_status = "REPLAYING";
    }
    m_hardware.apply(m_clip[index].command);
    m_last_frame_ms = (index == 0 || (m_resuming && index == m_resume_frame_count))
                          ? now_ms
                          : m_last_frame_ms + kPeriodMs;
    ++m_frame_count;
}

void Engine::cancel() {
    m_frame_count = 0;
    m_recording = false;
    m_resuming = false;
    m_checkpoint_button_down = false;
    m_resume_frame_count = 0;
    m_phase_start_ms = 0;
    m_last_frame_ms = 0;
    m_hardware.reset();
    m_state = State::Cancelled;
    m_status = "STOPPED";
}

bool Engine::busy() const {
    return m_state == State::Countdown || m_state == State::Recording ||
           m_state == State::Replaying || m_state == State::CheckpointPause ||
           m_state == State::Resetting;
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
    if (m_state == State::Countdown || m_state == State::CheckpointPause) {
        return std::min(now_ms - m_phase_start_ms, kCountdownMs);
    }
    if (m_state == State::Resetting) {
        return std::min(now_ms - m_phase_start_ms, kResetMs);
    }
    if (m_state == State::Complete) {
        return kResetMs;
    }
    return std::min(static_cast<std::uint32_t>(m_frame_count) * kPeriodMs, kRecordMs);
}

const char *Engine::status() const {
    return m_status;
}

} // namespace macrorec
