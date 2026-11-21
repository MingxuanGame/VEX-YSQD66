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

void run_red_left(bool allow_driver) {
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
    // 先退到左侧再用后臂推翻己方 Toggle；推住时不一定能到点，只等运动结束。
    chassis.moveToPoint(-50.0F, -7.5F, 1000);
    if (!wait_for_drive()) {
        return;
    }
    toggle::down();
    chassis.moveToPoint(-67.0F, -7.5F, 1050, {.forwards = false});
    if (!wait_for_drive()) {
        return;
    }

    // 驶向 Red Goal，撞上得分座后松开夹爪放下 Pin，再倒车退出。
    chassis.moveToPose(-50.5F, -18.5F, 158.0F, 2050);
    if (!wait_at_point(-50.5F, -18.5F)) {
        return;
    }
    claw::open();
    chassis.moveToPose(-60.0F, -8.0F, 95.0F, 1650, {.forwards = false, .lead = 0.2F});
    if (!wait_for_drive()) {
        return;
    }

    // 驶向分界线外侧的黄 Pin，到点夹紧后再运往 Neutral Goal。
    chassis.moveToPose(-28.0F, -21.4F, 120.0F, 2650);
    if (!wait_at_point(-28.0F, -21.4F)) {
        return;
    }
    claw::close();

    // 底盘异步驶向 Neutral Goal。升降在途中抬升，两者都到位后再放下黄 Pin。
    chassis.moveToPose(-46.6F, 16.0F, -14.1F, 3400, {.lead = 0.2F});
    constexpr std::uint32_t kLiftTimeoutMs = 4000;
    constexpr std::int32_t kLiftMaxVelocityRpm = 100; // 带载待实车验证。
    lifting::move_to({850, 850}, kLiftMaxVelocityRpm,
                     std::min(kLiftTimeoutMs, route_timer.getTimeLeft()), allow_driver);
    if (!wait_at_point(-46.6F, 16.0F)) {
        return;
    }
    // 放下黄 Pin
    claw::open();

    // 倒车退向场地中央，结束自动路线。
    chassis.moveToPose(-12.0F, -0.7F, -90.0F, 2900, {.forwards = false, .lead = 0.3F});
    wait_for_drive();
}

void run_red_down(bool) {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：红方下路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
}

void run_blue_right(bool) {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：蓝方右路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
}

void run_blue_up(bool) {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：蓝方上路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
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
