#include "autons.hpp"

#include "brain_ui.hpp"
#include "chassis.h"
#include "claw.hpp"
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

void run_red_left(bool allow_driver) {
    // Override 2.1 第 18 页：对抗赛自动时段为 15 秒，预留停车时间。
    constexpr std::uint32_t kRouteBudgetMs = 14500;
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
    const auto wait_for_drive = [&] {
        while (can_run() && chassis.isInMotion()) {
            pros::delay(kPollMs);
        }
        return can_run();
    };
    const auto wait_at_point = [&](float x_in, float y_in) {
        if (!wait_for_drive()) {
            return false;
        }
        const auto pose = chassis.getPose();
        return std::hypot(pose.x - x_in, pose.y - y_in) <= kPlacementToleranceIn;
    };

    chassis.setPose(-62.5F, -7.5F, 90.0F);
    // 驶向 Red Goal，撞上得分座后松开夹爪放下 Pin，再倒车退出。
    chassis.moveToPoint(-39.2F, -17.5F, 1900);
    if (!wait_at_point(-39.2F, -17.5F)) {
        return;
    }
    claw::open();
    chassis.moveToPoint(-46.5F, -13.9F, 1050, {.forwards = false});
    if (!wait_for_drive()) {
        return;
    }

    // 驶向分界线外侧的黄 Pin，到点夹紧后再运往 Neutral Goal。
    chassis.moveToPose(-23.0F, -16.6F, 116.05F, 1650);
    if (!wait_at_point(-23.0F, -16.6F)) {
        return;
    }
    claw::close();

    // 底盘异步驶向 Neutral Goal。升降恢复时在行进中抬升，两者都到位后再放下。
    chassis.moveToPose(-37.8F, 19.2F, -60.45F, 3500);
    constexpr std::uint32_t kLiftTimeoutMs = 4000;
    constexpr std::int32_t kLiftMaxVelocityRpm = 100; // 带载待实车验证。
    if (!lifting::move_to({850,850}, kLiftMaxVelocityRpm,
                          std::min(kLiftTimeoutMs, route_timer.getTimeLeft()))) {
        wait_at_point(-37.8F, 19.2F);
    }
    // 放下黄 Pin
    claw::open();
}

void run_red_down() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：红方下路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
}

void run_blue_right() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：蓝方右路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
}

void run_blue_up() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：蓝方上路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
}

void run_impl(bool allow_driver) {
    const auto preset = brain_ui::selected_auton();
    switch (preset) {
    case brain_ui::AutonPreset::RedLeft:
        run_red_left(allow_driver);
        break;
    case brain_ui::AutonPreset::RedDown:
        run_red_down();
        break;
    case brain_ui::AutonPreset::BlueRight:
        run_blue_right();
        break;
    case brain_ui::AutonPreset::BlueUp:
        run_blue_up();
        break;
    case brain_ui::AutonPreset::Count:
        break;
    }

    chassis::stop();
}

} // namespace

void run() {
    run_impl(false);
}

void run_in_driver() {
    run_impl(true);
}

} // namespace autons
