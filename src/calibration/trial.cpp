#include "calibration/trial.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace calibration {
namespace {

// 这些值限制练习动作和取样窗口，需在实车复核，不作为底盘标定结果。
constexpr std::uint32_t kSampleTimeoutMs = 150;
constexpr std::uint32_t kTravelMs = 1500;
constexpr std::uint32_t kTurnMs = 450;
constexpr std::uint32_t kDistanceTimeoutMs = 2800;
constexpr std::uint32_t kPowerWindowMs = 120;
constexpr std::uint32_t kBreakawayTimeoutMs = 5500;
constexpr std::uint32_t kQuietMs = 250;
constexpr std::uint32_t kSettleTimeoutMs = 1200;
constexpr int kMaxBreakawayPower = 40;
constexpr double kMaxStraightYawDeg = 12.0;
constexpr double kTurnStopYawDeg = 25.0;
constexpr double kMaxTravelSampleDeg = 720.0;
constexpr double kMaxCalibratedTravelIn = 42.0;
constexpr double kMinTravelSampleDeg = 80.0;
constexpr double kBreakawayTravelDeg = 5.0;
constexpr std::array<std::uint32_t, 3> kSteadyMarksMs{800, 1200, 1600};
constexpr std::array<std::uint32_t, 3> kAccelerationMarksMs{60, 180, 300};

using Motors = std::array<double, 4>;
using Sides = std::array<double, 2>;

Motors motor_delta_deg(const macrorec::Feedback &from, const macrorec::Feedback &to) {
    Motors delta{};
    for (std::size_t i = 0; i < delta.size(); ++i) {
        delta[i] = to.motor_position_deg[i] - from.motor_position_deg[i];
    }
    return delta;
}

double max_abs_deg(const Motors &motor_deg) {
    double largest_deg = 0;
    for (const double angle_deg : motor_deg) {
        largest_deg = std::max(largest_deg, std::abs(angle_deg));
    }
    return largest_deg;
}

Sides side_mean_deg(const Motors &motor_deg) {
    return {std::midpoint(motor_deg[0], motor_deg[1]), std::midpoint(motor_deg[2], motor_deg[3])};
}

std::uint32_t elapsed_ms(const std::uint32_t from_ms, const std::uint32_t to_ms) {
    return to_ms - from_ms;
}

bool stable_speed(double first_in_s, double second_in_s) {
    if (std::min(first_in_s, second_in_s) < 0.2) {
        return false;
    }
    const double spread = std::abs(first_in_s - second_in_s);
    return spread <= std::max(0.3, 0.15 * std::midpoint(first_in_s, second_in_s));
}

} // namespace

Trial::Trial(macrorec::Hardware &hardware) : m_hardware(hardware) {}

void Trial::start(const TrialConfig &config, std::uint32_t now_ms) {
    m_hardware.stop();
    m_config = config;
    m_result = {};
    m_origin.time_ms = now_ms;
    if (!m_hardware.read_feedback(m_origin.feedback)) {
        fail("IMU or encoder unavailable");
        return;
    }
    m_previous = m_origin;
    m_window_start = m_origin;
    m_breakaway_power = 1;
    m_speed_mark_count = 0;
    m_state = TrialState::Running;
    switch (config.kind) {
    case TrialKind::None:
        fail("No powered trial selected");
        break;
    case TrialKind::Breakaway:
        m_status = "Increasing start power";
        m_hardware.apply({m_breakaway_power, m_breakaway_power});
        break;
    case TrialKind::Turn:
        m_status = "Measuring clockwise turn";
        m_hardware.apply({config.power, -config.power});
        break;
    default:
        m_status = "Measuring straight drive";
        m_hardware.apply({config.power, config.power});
        break;
    }
}

void Trial::update(std::uint32_t now_ms) {
    if (m_state != TrialState::Running && m_state != TrialState::Settling) {
        return;
    }
    const auto delta_ms = elapsed_ms(m_previous.time_ms, now_ms);
    if (!delta_ms) {
        return;
    }
    if (delta_ms > kSampleTimeoutMs) {
        fail("Feedback interval too long");
        return;
    }
    Stamp sample{.time_ms = now_ms};
    if (!m_hardware.read_feedback(sample.feedback)) {
        fail("IMU or encoder unavailable");
        return;
    }
    // 蓝齿轮组额定 600 rpm。只拦截远超物理上限的跳变或传感器清零。
    const double delta_s = delta_ms / 1000.0;
    if (max_abs_deg(motor_delta_deg(m_previous.feedback, sample.feedback)) > 4200 * delta_s + 8 ||
        std::abs(sample.feedback.imu_rotation_deg - m_previous.feedback.imu_rotation_deg) >
            1400 * delta_s + 5) {
        fail("Encoder or IMU readings jumped");
        return;
    }
    m_previous = sample;
    read_motion(sample);
    if (m_state == TrialState::Settling) {
        update_settling(sample);
        return;
    }

    const auto run_ms = elapsed_ms(m_origin.time_ms, now_ms);
    const auto motor_deg = max_abs_deg(m_result.delta_deg);
    const auto yaw_deg = m_result.yaw_deg;
    if (m_config.kind == TrialKind::Turn) {
        if (yaw_deg < -3.0) {
            fail("Check clockwise turn direction");
        } else if (motor_deg >= kMaxTravelSampleDeg) {
            fail("Turn exceeded the wheel travel limit");
        } else if (yaw_deg >= kTurnStopYawDeg || run_ms >= kTurnMs) {
            settle(sample);
        }
        return;
    }

    if (std::abs(yaw_deg) > kMaxStraightYawDeg ||
        *std::min_element(m_result.delta_deg.begin(), m_result.delta_deg.end()) < -12.0) {
        fail("Drive veered or a wheel ran backward");
        return;
    }
    if (m_result.travel_in > kMaxCalibratedTravelIn ||
        (m_config.kind != TrialKind::Travel && motor_deg > kMaxTravelSampleDeg * 4)) {
        fail("Drive exceeded the travel limit");
        return;
    }

    switch (m_config.kind) {
    case TrialKind::Travel:
        if (motor_deg >= kMaxTravelSampleDeg || run_ms >= kTravelMs) {
            settle(sample);
        }
        break;
    case TrialKind::Breakaway:
        update_breakaway(sample);
        break;
    case TrialKind::Steady:
        update_speed_window(sample, kSteadyMarksMs);
        break;
    case TrialKind::Accelerate:
        update_speed_window(sample, kAccelerationMarksMs);
        break;
    case TrialKind::StopAtDistance:
        if (m_result.travel_in >= m_config.target_in) {
            settle(sample);
        } else if (run_ms >= kDistanceTimeoutMs) {
            fail("Target not reached before timeout");
        }
        break;
    case TrialKind::None:
    case TrialKind::Turn:
        break;
    }
}

void Trial::cancel() {
    m_hardware.stop();
    m_state = TrialState::Idle;
    m_status = "Ready";
}

TrialState Trial::state() const {
    return m_state;
}

const TrialResult &Trial::result() const {
    return m_result;
}

const char *Trial::status() const {
    return m_status;
}

void Trial::fail(const char *message) {
    m_hardware.stop();
    m_state = TrialState::Fault;
    m_status = message;
}

void Trial::settle(const Stamp &sample) {
    m_hardware.stop();
    m_state = TrialState::Settling;
    m_status = "Braking. Wait for a stable reading";
    m_stop_ms = sample.time_ms;
    m_window_start = sample;
}

void Trial::read_motion(const Stamp &sample) {
    m_result.delta_deg = motor_delta_deg(m_origin.feedback, sample.feedback);
    m_result.yaw_deg = sample.feedback.imu_rotation_deg - m_origin.feedback.imu_rotation_deg;
    const auto sides_deg = side_mean_deg(m_result.delta_deg);
    m_result.travel_in =
        std::midpoint(sides_deg[0], sides_deg[1]) * m_config.travel_per_rev / 360.0;
}

void Trial::update_settling(const Stamp &sample) {
    // 使用累计位移的静止窗口，避免极慢滑行在相邻帧中被误判为静止。
    if (max_abs_deg(motor_delta_deg(m_window_start.feedback, sample.feedback)) > 3.0 ||
        std::abs(sample.feedback.imu_rotation_deg - m_window_start.feedback.imu_rotation_deg) >
            0.5) {
        m_window_start = sample;
    }
    if (elapsed_ms(m_stop_ms, sample.time_ms) >= kSettleTimeoutMs) {
        fail("Vehicle did not settle");
        return;
    }
    if (elapsed_ms(m_window_start.time_ms, sample.time_ms) < kQuietMs) {
        return;
    }
    if (m_config.kind == TrialKind::Travel &&
        (*std::min_element(m_result.delta_deg.begin(), m_result.delta_deg.end()) <
             kMinTravelSampleDeg ||
         std::abs(m_result.yaw_deg) > kMaxStraightYawDeg)) {
        fail("Travel needs more straight wheel motion");
        return;
    }
    if (m_config.kind == TrialKind::Turn && m_result.yaw_deg < 2.0) {
        fail("No measurable clockwise turn");
        return;
    }
    m_state = TrialState::Complete;
    m_status = "Measurement complete";
}

void Trial::update_breakaway(const Stamp &sample) {
    if (elapsed_ms(m_origin.time_ms, sample.time_ms) >= kBreakawayTimeoutMs) {
        fail("Start-power measurement timed out");
        return;
    }
    if (elapsed_ms(m_window_start.time_ms, sample.time_ms) < kPowerWindowMs) {
        return;
    }
    const auto delta = motor_delta_deg(m_window_start.feedback, sample.feedback);
    if (*std::min_element(delta.begin(), delta.end()) >= kBreakawayTravelDeg) {
        m_result.breakaway_power = m_breakaway_power;
        settle(sample);
        return;
    }
    if (m_breakaway_power >= kMaxBreakawayPower) {
        fail("No sustained motion within web kS range");
        return;
    }
    ++m_breakaway_power;
    m_window_start = sample;
    m_hardware.apply({m_breakaway_power, m_breakaway_power});
}

void Trial::update_speed_window(const Stamp &sample, const std::array<std::uint32_t, 3> &marks_ms) {
    if (elapsed_ms(m_origin.time_ms, sample.time_ms) < marks_ms[m_speed_mark_count]) {
        return;
    }
    m_speed_marks[m_speed_mark_count++] = sample;
    if (m_speed_mark_count < m_speed_marks.size()) {
        return;
    }

    const auto &first = m_speed_marks[0];
    const auto &middle = m_speed_marks[1];
    const auto &last = m_speed_marks[2];
    const auto speed = [this](const Stamp &a, const Stamp &b) {
        const auto sides_deg = side_mean_deg(motor_delta_deg(a.feedback, b.feedback));
        const double dt_s = elapsed_ms(a.time_ms, b.time_ms) / 1000.0;
        const double scale = m_config.travel_per_rev / (360.0 * dt_s);
        return Sides{sides_deg[0] * scale, sides_deg[1] * scale};
    };
    const auto early = speed(first, middle);
    const auto late = speed(middle, last);
    if (m_config.kind == TrialKind::Steady) {
        if (!stable_speed(early[0], late[0]) || !stable_speed(early[1], late[1])) {
            fail("Speed varied. Check traction and retry");
            return;
        }
        const auto steady = speed(first, last);
        m_result.left_in_s = steady[0];
        m_result.right_in_s = steady[1];
    } else {
        m_result.accel_start_in_s = std::midpoint(early[0], early[1]);
        m_result.accel_end_in_s = std::midpoint(late[0], late[1]);
        // 两段均速代表各自区间中点，使用实际采样间隔。
        m_result.accel_time_s = elapsed_ms(first.time_ms, last.time_ms) / 2000.0;
        if (m_result.accel_start_in_s < 0 ||
            m_result.accel_end_in_s - m_result.accel_start_in_s < 0.15) {
            fail("No clear acceleration in sampled window");
            return;
        }
    }
    settle(sample);
}

} // namespace calibration
