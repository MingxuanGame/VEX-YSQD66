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

void run_red_left() {
    // Override 2.1 第 18 页：对抗赛自动时段为 15 秒，预留停车时间。
    constexpr std::uint32_t kRouteBudgetMs = 14500;
    constexpr std::uint32_t kPollMs = 10;
    constexpr float kPlacementToleranceIn = 3.0F; // 沿用底盘位置环的大误差范围。

    lemlib::Timer route_timer(kRouteBudgetMs);
    const auto can_run = [&] {
        // Override 2.1 <R9>：仅在自动模式启用时继续动作。
        return !pros::competition::is_disabled() && pros::competition::is_autonomous() &&
               !route_timer.isDone();
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

    chassis.setPose(-62.5F, 6.3F, 90.0F);
    // 驶向 Red Goal，到点松开夹爪放下 Pin，再倒车退出。
    chassis.moveToPoint(-42.6F, -18.6F, 2200);
    if (!wait_at_point(-42.6F, -18.6F)) {
        return;
    }
    claw::open();
    chassis.moveToPoint(-50.6F, -9.9F, 1250, {.forwards = false});
    if (!wait_for_drive()) {
        return;
    }

    // 驶向分界线外侧的黄 Pin，到点夹紧后再驶向 Neutral Goal。
    chassis.moveToPoint(-24.3F, -16.8F, 1950);
    if (!wait_at_point(-24.3F, -16.8F)) {
        return;
    }
    claw::close();

    // 底盘异步驶向 Neutral Goal。升降恢复时在行进中抬升，两者都到位后再松开。
    chassis.moveToPoint(-46.1F, 21.8F, 2800);
    // constexpr std::uint32_t kLiftTimeoutMs = 4000;
    // constexpr std::int32_t kLiftMaxVelocityRpm = 100; // 带载待实车验证。
    // if (!lifting::move_to({lifting::kUpperLimitDeg, lifting::kUpperLimitDeg}, kLiftMaxVelocityRpm,
    //                       std::min(kLiftTimeoutMs, route_timer.getTimeLeft()))) {
    //     return;
    // }
    if (!wait_at_point(-46.1F, 21.8F)) {
        return;
    }
    // 放下黄 Pin
    claw::open();

    // 倒车进入中央公共区域并对准 Toggle，随后停车。
    chassis.moveToPose(-21.0F, -1.9F, 271.77F, 2700, {.forwards = false});
    wait_for_drive();
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

} // namespace

void run() {
    const auto preset = brain_ui::selected_auton();
    switch (preset) {
    case brain_ui::AutonPreset::RedLeft:
        run_red_left();
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

} // namespace autons
