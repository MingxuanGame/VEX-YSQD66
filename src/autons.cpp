#include "autons.hpp"

#include "brain_ui.hpp"
#include "chassis.h"
#include "claw.hpp"
#include "toggle.hpp"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "lifting.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// 在 static/ 放入 JerryIO / LemLib Path Planner 导出的路径文本后取消注释：
// ASSET(example_txt);

namespace autons {
namespace {

// 场地坐标以场地中心为原点，θ 以 +y 为 0°、顺时针为正（LemLib 约定）。
struct RoutePoint {
    float x;
    float y;
    float theta;
};

float wrap_deg(float deg) {
    while (deg > 180.0F) {
        deg -= 360.0F;
    }
    while (deg <= -180.0F) {
        deg += 360.0F;
    }
    return deg;
}

RoutePoint as_is(RoutePoint point) {
    return point;
}

// 以场地原点中心对称：Red Left → Blue Right。
RoutePoint rotate_180(RoutePoint point) {
    return {-point.x, -point.y, wrap_deg(point.theta + 180.0F)};
}

// 以红方左侧与下方之间的单实线（对角线 y = x）镜像：Red Left → Red Down。
RoutePoint mirror_solid_line(RoutePoint point) {
    return {point.y, point.x, wrap_deg(90.0F - point.theta)};
}

// Red Down 再以场地原点中心对称：→ Blue Up。
RoutePoint mirror_solid_line_rotate_180(RoutePoint point) {
    return {-point.y, -point.x, wrap_deg(270.0F - point.theta)};
}

// 四个预设共用同一条 Red Left 路线，只按场地镜像变换每个点位与朝向；
// 夹爪、升降、Toggle 动作与路径形状保持不变。
void run_route(bool allow_driver, RoutePoint (*transform)(RoutePoint)) {
    // 路线开始时夹紧，固定预装 Pin。
    claw::close();
    // Override 2.1 第 18 页：对抗赛自动时段为 15 秒；路线最多运行 14.9 秒，留 100 ms 停车余量。
    constexpr std::uint32_t kRouteBudgetMs = 14900;
    constexpr std::uint32_t kPollMs = 10;
    constexpr float kPlacementToleranceIn = 3.0F; // 沿用底盘位置环的大误差范围。

    lemlib::Timer route_timer(kRouteBudgetMs);
    const auto can_run = [&] {
        // Override 2.1 <R9>：禁用或切换阶段后立即停止动作。
        // allow_driver 供手动阶段的按 A 入口运行同一条路线。
        return !pros::competition::is_disabled() &&
               (allow_driver || pros::competition::is_autonomous()) && !route_timer.isDone();
    };
    if (!can_run()) {
        return;
    }

    auto &chassis = chassis::controller();
    const auto at = [&](float x, float y, float theta) { return transform({x, y, theta}); };
    const auto move_to_point = [&](float x, float y, int timeout_in,
                                   lemlib::MoveToPointParams params = {}) {
        const auto point = at(x, y, 0.0F);
        chassis.moveToPoint(point.x, point.y, timeout_in, params);
    };
    const auto move_to_pose = [&](float x, float y, float theta, int timeout_in,
                                  lemlib::MoveToPoseParams params = {}) {
        const auto point = at(x, y, theta);
        chassis.moveToPose(point.x, point.y, point.theta, timeout_in, params);
    };
    const auto wait_for_drive = [&] {
        while (can_run() && chassis.isInMotion()) {
            pros::delay(kPollMs);
        }
        return can_run();
    };
    const auto wait_at_point = [&](float x, float y) {
        if (!wait_for_drive()) {
            return false;
        }
        const auto point = at(x, y, 0.0F);
        const auto pose = chassis.getPose();
        return std::hypot(pose.x - point.x, pose.y - point.y) <= kPlacementToleranceIn;
    };
    // 离目标还剩约 lead_ms 的行驶时间时先合爪，不再等停稳；一直没触发则在运动结束后合。
    // 接近速度用约 50 ms 的位姿变化估算，速度太低时只用“运动结束”兜底。
    const auto close_before_stop = [&](float x, float y, std::uint32_t lead_ms) {
        const auto point = at(x, y, 0.0F);
        constexpr std::uint32_t kSpeedWindowMs = 50;
        auto window_distance =
            std::hypot(chassis.getPose().x - point.x, chassis.getPose().y - point.y);
        auto window_ms = pros::millis();
        while (can_run() && chassis.isInMotion()) {
            pros::delay(kPollMs);
            const auto pose = chassis.getPose();
            const float distance = std::hypot(pose.x - point.x, pose.y - point.y);
            const auto now_ms = pros::millis();
            const auto elapsed_ms = now_ms - window_ms;
            if (elapsed_ms < kSpeedWindowMs) {
                continue;
            }
            const float speed_in_s =
                (window_distance - distance) * 1000.0F / static_cast<float>(elapsed_ms);
            if (speed_in_s > 0.5F &&
                distance <= speed_in_s * static_cast<float>(lead_ms) / 1000.0F) {
                break;
            }
            window_distance = distance;
            window_ms = now_ms;
        }
        if (can_run()) {
            claw::close();
        }
        return wait_at_point(x, y);
    };

    const auto start = at(-62.5F, -7.5F, 90.0F);
    chassis.setPose(start.x, start.y, start.theta);
    // 开局用后臂撞己方 Toggle：前进顶到 Toggle 就下放，倒车退出后抬起，连做两趟再走后续路线。
    // 顶住 Toggle 时不一定能到点，每段只等运动结束，不检查是否到位。
    constexpr int kTogglePasses = 2;
    for (int pass = 0; pass < kTogglePasses; ++pass) {
        move_to_point(-50.0F, -7.5F, 1000);
        if (!wait_for_drive()) {
            return;
        }
        toggle::down();
        move_to_point(-67.0F, -7.5F, 1350, {.forwards = false});
        if (!wait_for_drive()) {
            return;
        }
        toggle::up();
    }
    // 第一次放 Pin：驶向红 Goal，顶进去后开爪放下预装 Pin。
    // 实车在碰到 Goal 前就停住，目标点沿车头方向再顶 3 in，timeout 加 300 ms。
    move_to_pose(-49.0F, -22.3F, 150.0F, 1800, {.lead = 0.5F});
    if (!wait_at_point(-49.0F, -22.3F)) {
        return;
    }
    claw::open();
    // 倒车退出红 Goal。
    move_to_pose(-60.0F, -8.0F, 95.0F, 1850, {.forwards = false, .lead = 0.2F});
    if (!wait_for_drive()) {
        return;
    }

    // 夹黄 Pin：离目标还剩约 100 ms 行程时合爪，不等停稳。
    constexpr std::uint32_t kGrabLeadMs = 100;
    move_to_pose(-27.8F, -21.3F, 128.0F, 2000, {.lead = 0.4F});
    if (!close_before_stop(-27.8F, -21.3F, kGrabLeadMs)) {
        return;
    }

    // 转回红 Goal，把黄 Pin 放到同一个 Goal 上；升降比第一次高一些。
    // 实车调头后方向偏左，回程朝向往右收 10°（-100 → -90）。
    const float return_heading = at(0.0F, 0.0F, -90.0F).theta;
    chassis.turnToHeading(return_heading, 1300);
    if (!wait_for_drive()) {
        return;
    }
    move_to_pose(-42.0F, -19.6F, -83.0F, 1200);
    constexpr std::uint32_t kLiftTimeoutMs = 1800;
    constexpr std::int32_t kLiftMaxVelocityRpm = 100; // 带载待实车验证。
    constexpr double kSecondPinLiftDeg = -700.0;      // 第二次放 Pin 的抬升高度，待实车确认。
    lifting::move_to({kSecondPinLiftDeg, kSecondPinLiftDeg}, kLiftMaxVelocityRpm,
                     std::min(kLiftTimeoutMs, route_timer.getTimeLeft()), allow_driver);
    if (!wait_at_point(-42.0F, -19.6F)) {
        return;
    }
    claw::open();

    // 倒车去场地中央，结束自动路线。
    move_to_pose(-9.8F, -4.9F, 240.0F, 2650, {.forwards = false, .lead = 0.6F});
    wait_for_drive();
}

void run_red_left(bool allow_driver) {
    run_route(allow_driver, as_is);
}

void run_red_down(bool allow_driver) {
    run_route(allow_driver, mirror_solid_line);
}

void run_blue_right(bool allow_driver) {
    run_route(allow_driver, rotate_180);
}

void run_blue_up(bool allow_driver) {
    run_route(allow_driver, mirror_solid_line_rotate_180);
}

// Plan 2 的路线动作待填写。
void run_red_left_2(bool) {}

void run_red_down_2(bool) {}

void run_blue_right_2(bool) {}

void run_blue_up_2(bool) {}

void run_impl(bool allow_driver) {
    claw::unlock_startup();
    const auto preset = brain_ui::selected_auton();
    const bool plan_2 = brain_ui::selected_plan() == brain_ui::AutonPlan::Plan2;
    switch (preset) {
    case brain_ui::AutonPreset::RedLeft:
        (plan_2 ? run_red_left_2 : run_red_left)(allow_driver);
        break;
    case brain_ui::AutonPreset::RedDown:
        (plan_2 ? run_red_down_2 : run_red_down)(allow_driver);
        break;
    case brain_ui::AutonPreset::BlueRight:
        (plan_2 ? run_blue_right_2 : run_blue_right)(allow_driver);
        break;
    case brain_ui::AutonPreset::BlueUp:
        (plan_2 ? run_blue_up_2 : run_blue_up)(allow_driver);
        break;
    case brain_ui::AutonPreset::Count:
        break;
    }

    chassis::stop();
}

} // namespace

void run() {
    if (!pros::competition::is_autonomous() || pros::competition::is_disabled()) {
        return;
    }
    run_impl(false);
}

void run_in_driver() {
    run_impl(true);
}

} // namespace autons
