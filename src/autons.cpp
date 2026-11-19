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
#include <cstdint>

// 在 static/ 放入 JerryIO / LemLib Path Planner 导出的路径文本后取消注释：
// ASSET(example_txt);

namespace autons {
namespace {

// 已知缺口：当前只有 run_red_left 有完整路线；red_down / blue_right / blue_up 只重置 pose。
void run_red_left() {
    // Override 2.1 第 18 页：对抗赛自动时段为 15 秒，预留停车时间。
    constexpr std::uint32_t kRouteBudgetMs = 14500;
    constexpr std::uint32_t kPollMs = 10;
    // 已知依赖：lifting::limits() 为空时整条路线直接返回；需先 SET UP 或 USE SAVED。
    // move_to 使用相对角度，0 对应 Brain 菜单实测的 SET LOW。
    constexpr double kReleaseOffsetDeg = 0.0;
    // 待实机确认机构动作时间。
    constexpr std::uint32_t kLiftTimeoutMs = 900;
    constexpr std::uint32_t kClawTravelMs = 250;
    constexpr int kLiftMaxPower = 80;

    lemlib::Timer route_timer(kRouteBudgetMs);
    const auto can_run = [&] {
        // Override 2.1 <R9>：仅在自动模式启用时继续动作。
        return !pros::competition::is_disabled() && pros::competition::is_autonomous() &&
               !route_timer.isDone();
    };
    const auto &limits = lifting::limits();
    if (!can_run() || !limits) {
        return;
    }
    // SET UP 保存两路上限；取较短的相对行程，确保两路都在限位内。
    const double pickup_offset_deg = std::min(limits->upper_deg[0] - limits->lower_deg[0],
                                              limits->upper_deg[1] - limits->lower_deg[1]);

    auto &chassis = chassis::controller();
    // LemLib 默认异步执行。每段结束后再下发下一段或操作机构。
    const auto wait_for_drive = [&] {
        while (can_run() && chassis.isInMotion()) {
            pros::delay(kPollMs);
        }
        return can_run();
    };
    const auto move_lift_and_claw = [&](double offset_deg, void (*actuate_claw)()) {
        const auto timeout_ms = std::min(kLiftTimeoutMs, route_timer.getTimeLeft());
        if (!lifting::move_to(offset_deg, timeout_ms, kLiftMaxPower) || !can_run()) {
            return false;
        }
        actuate_claw();
        // 让气缸走完行程；夹紧状态需保持到放置点。
        lemlib::Timer travel(kClawTravelMs);
        while (can_run() && !travel.isDone()) {
            pros::delay(kPollMs);
        }
        return can_run();
    };

    chassis.setPose(-60.4F, 7.7F, 90.0F);

    chassis.moveToPoint(-49.9F, -13.4F, 1650); // Red Goal
    if (!wait_for_drive() || !move_lift_and_claw(kReleaseOffsetDeg, claw::open_autonomous)) {
        return;
    }
    chassis.moveToPoint(-54.2F, -5.8F, 1000, {.forwards = false}); // 倒出 Red Goal
    if (!wait_for_drive()) {
        return;
    }

    chassis.turnToPoint(-38.2F, -14.9F, 950); // 转向外侧 Cup + Pin
    if (!wait_for_drive()) {
        return;
    }
    chassis.moveToPoint(-31.3F, -19.9F, 1700);
    if (!wait_for_drive() || !move_lift_and_claw(pickup_offset_deg, claw::close_autonomous)) {
        return;
    }

    chassis.turnToHeading(120.63F, 700);
    if (!wait_for_drive()) {
        return;
    }
    chassis.moveToPoint(-44.8F, 12.9F, 2100);
    if (!wait_for_drive() || !move_lift_and_claw(kReleaseOffsetDeg, claw::open_autonomous)) {
        return;
    }
    chassis.moveToPoint(-36.5F, 9.5F, 1050, {.forwards = false});
    if (!wait_for_drive()) {
        return;
    }

    chassis.moveToPose(-59.6F, 18.6F, 302.77F, 1900);
    wait_for_drive();
}

void run_red_down() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 已知缺口：仅占位 reset pose，没有实际自动路线。
}

void run_blue_right() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 已知缺口：仅占位 reset pose，没有实际自动路线。
}

void run_blue_up() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 已知缺口：仅占位 reset pose，没有实际自动路线。
}

} // namespace

void run() {
    switch (brain_ui::selected_auton()) {
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
    lifting::stop();
    claw::stop();
}

} // namespace autons
