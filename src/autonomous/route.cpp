#include "autonomous/route.hpp"

#include "autonomous/odometry.hpp"
#include "autonomous/path.hpp"
#include "chassis.h"
#include "math/pid.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace auto_route {
namespace {

// Override 2.1: Match、SC1、GG12/GG13、R9。预留 200 ms 给场控停车。
constexpr std::uint32_t kStopMs = 14800;
constexpr std::uint32_t kPeriodMs = 20;

bool enabled() {
    return pros::competition::is_autonomous() && !pros::competition::is_disabled();
}

struct Window {
    std::uint32_t start_ms;
    std::uint32_t step_ms;
    std::uint32_t limit_ms;

    bool active() const {
        const auto now_ms = pros::millis();
        return enabled() && now_ms - start_ms < kStopMs && now_ms - step_ms < limit_ms;
    }
};

double signed_static(double command, double amount) {
    return command == 0.0 ? 0.0 : std::copysign(amount, command);
}

void drive_power(double forward, double steer, double trim, int max_power) {
    const double left = forward + steer + trim;
    const double right = forward - steer - trim;
    const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)) / max_power);
    chassis::set_autonomous_power(std::lround(left / scale), std::lround(right / scale));
}

math::Pid heading_pid(const DriveConfig &config, int max_power) {
    return math::Pid({config.heading_p, config.heading_i, config.heading_d, 30.0,
                      static_cast<double>(max_power)});
}

bool align(Odometry &odom, const DriveConfig &config, Point position, double heading_deg,
           int max_power, const Window &window) {
    auto controller = heading_pid(config, max_power);
    int settled = 0;
    while (window.active()) {
        if (!odom.update() ||
            distance_in(odom.pose().position, position) > config.tracking_limit_in) {
            return false;
        }
        const double error_deg = angle_error_deg(heading_deg, odom.pose().heading_deg);
        if (std::abs(error_deg) <= config.heading_tolerance_deg) {
            chassis::stop();
            if (++settled == 2) {
                return true;
            }
        } else {
            settled = 0;
            const double steer = controller.update(error_deg, odom.dt_s());
            drive_power(0.0, steer + signed_static(steer, config.static_power), 0.0, max_power);
        }
        pros::delay(kPeriodMs);
    }
    return false;
}

bool follow_curve(Odometry &odom, const DriveConfig &config, const Step &step,
                  const std::vector<Sample> &path, const Window &window) {
    if (path.size() < 2 || !align(odom, config, path.front().position, path.front().heading_deg,
                                  step.max_power, window)) {
        return false;
    }
    auto steering = heading_pid(config, step.max_power);
    math::Pid speed({config.speed_p, config.speed_i, config.speed_d, 12.0,
                     static_cast<double>(step.max_power)});
    const double direction = step.reverse ? -1.0 : 1.0;
    std::size_t nearest = 0;
    double previous_target_in_s = 0.0;
    while (window.active()) {
        if (!odom.update()) {
            return false;
        }
        const auto pose = odom.pose();
        nearest = advance_nearest(path, nearest, pose.position, config.lookahead_in * 2.0 + 1.0);
        if (distance_in(pose.position, path[nearest].position) > config.tracking_limit_in) {
            return false;
        }
        const auto &last = path.back();
        const double end_distance_in = distance_in(pose.position, last.position);
        const double remaining_in = last.distance_in - path[nearest].distance_in;
        if (remaining_in <= config.position_tolerance_in &&
            end_distance_in <= config.position_tolerance_in) {
            chassis::stop();
            return align(odom, config, last.position, last.heading_deg, step.max_power, window) &&
                   distance_in(odom.pose().position, last.position) <= config.position_tolerance_in;
        }

        // 前视点提供横向修正，末段切线约束避免在接近终点时急转。
        const auto target = lookahead_point(path, nearest, config.lookahead_in);
        const double aim_deg = bearing_deg(pose.position, target) + (step.reverse ? 180.0 : 0.0);
        const double tangent_error_deg =
            angle_error_deg(path[nearest].heading_deg, pose.heading_deg);
        const double aim_offset_deg = angle_error_deg(aim_deg, path[nearest].heading_deg);
        // 限制前视修正幅度。越过末端时停机，避免在靠近装置时掉头找点。
        if (std::abs(tangent_error_deg) > 70.0 ||
            (remaining_in < config.position_tolerance_in && std::abs(aim_offset_deg) > 90.0)) {
            return false;
        }
        const double steer_error_deg =
            tangent_error_deg + 0.80 * std::clamp(aim_offset_deg, -50.0, 50.0);
        const double steer = steering.update(steer_error_deg, odom.dt_s());
        const double braking_in_s =
            std::sqrt(2.0 * config.acceleration_in_s2 * std::max(remaining_in, end_distance_in));
        const double ramp_in_s =
            std::abs(previous_target_in_s) + config.acceleration_in_s2 * odom.dt_s();
        const double available_in_s =
            std::max(0.0, (step.max_power - config.static_power) / config.velocity_gain);
        const double target_in_s =
            direction * std::min({config.cruise_in_s, braking_in_s, ramp_in_s, available_in_s}) *
            std::max(0.0, std::cos(steer_error_deg * std::numbers::pi / 180.0));
        const double acceleration_in_s2 = (target_in_s - previous_target_in_s) / odom.dt_s();
        previous_target_in_s = target_in_s;
        const double feedforward = signed_static(target_in_s, config.static_power) +
                                   config.velocity_gain * target_in_s +
                                   config.acceleration_gain * acceleration_in_s2;
        const double feedback = speed.update(target_in_s - odom.speed_in_s(), odom.dt_s());
        const double trim = config.left_trim * target_in_s / config.cruise_in_s;
        drive_power(feedforward + feedback, steer, trim, step.max_power);
        pros::delay(kPeriodMs);
    }
    return false;
}

bool dock(Odometry &odom, const DriveConfig &config, const Step &step, Point from,
          const Window &window) {
    const double heading_rad = step.heading_deg * std::numbers::pi / 180.0;
    const double dx = std::sin(heading_rad);
    const double dy = std::cos(heading_rad);
    auto steer = heading_pid(config, step.max_power);
    double previous_in = 0.0;
    std::uint32_t still_ms = pros::millis();
    while (window.active()) {
        if (!odom.update()) {
            return false;
        }
        const auto pose = odom.pose();
        const double x_in = pose.position.x - step.end.x;
        const double y_in = pose.position.y - step.end.y;
        const double along_in = x_in * dx + y_in * dy;
        const double across_in = x_in * dy - y_in * dx;
        const double error_deg = angle_error_deg(step.heading_deg, pose.heading_deg);
        if (std::abs(across_in) > 0.7 || std::abs(error_deg) > 7.0 || along_in > 0.5 ||
            distance_in(pose.position, from) > 5.6) {
            return false;
        }
        const auto now_ms = pros::millis();
        if (std::abs(along_in - previous_in) > 0.12) {
            previous_in = along_in;
            still_ms = now_ms;
        }
        // 编码器静止仅在预期接触区内可视为顶住；其余堵转立即终止整条路线。
        if (now_ms - still_ms >= 180) {
            return along_in >= -0.65;
        }
        const double correction = steer.update(error_deg, odom.dt_s());
        drive_power(step.max_power, correction, 0.0, step.max_power);
        pros::delay(kPeriodMs);
    }
    return false;
}

bool wait(Odometry &odom, const Window &window) {
    while (window.active()) {
        if (!odom.update()) {
            return false;
        }
        pros::delay(kPeriodMs);
    }
    return true;
}

} // namespace

void stop() {
    chassis::stop();
}

void run(const Config &config, std::span<const Step> steps) {
    const auto start_ms = pros::millis();
    stop();
    if (!enabled()) {
        return;
    }
    Odometry odom(config.start, config.drive.travel_in_per_motor_rev);
    if (!odom.begin()) {
        std::puts("AUTO stopped: sensor not ready");
        return;
    }
    auto planned = config.start;
    for (const Step &step : steps) {
        const Window window{start_ms, pros::millis(), step.limit_ms};
        if (!window.active()) {
            break;
        }
        bool success = false;
        switch (step.action) {
        case Action::drive: {
            const auto path = sample_curve(planned, step);
            success = follow_curve(odom, config.drive, step, path, window);
            planned = {step.end, path.back().heading_deg};
            break;
        }
        case Action::turn:
            success = align(odom, config.drive, planned.position, step.heading_deg, step.max_power,
                            window);
            planned.heading_deg = step.heading_deg;
            break;
        case Action::dock:
            success = dock(odom, config.drive, step, planned.position, window);
            planned = {step.end, step.heading_deg};
            break;
        case Action::wait:
            success = wait(odom, window);
            break;
        }
        stop();
        const auto elapsed_ms = pros::millis() - start_ms;
        std::printf("AUTO %u %s %lu ms\n", step.checkpoint,
                    success && enabled() && elapsed_ms < kStopMs ? "OK" : "STOP",
                    static_cast<unsigned long>(elapsed_ms));
        if (!success || !enabled() || elapsed_ms >= kStopMs) {
            break;
        }
    }
}

} // namespace auto_route
