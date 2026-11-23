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

// 技能赛（Section 3 - Robot Skills，英文第 51 页）：单局 1 分钟，本路线按 30 秒预算运行，
// 留 30 秒停车余量；禁用、切模式或预算用尽时立即退出。
constexpr std::uint32_t kRouteBudgetMs = 14900;
constexpr std::uint32_t kPollMs = 10;
constexpr float kPlacementToleranceIn = 3.0F; // 沿用底盘位置环的大误差范围。

// 带载升降的转速上限与停车超时；抬升高度沿用路线 1 的第二次放 Pin 高度，待实车复核。
constexpr std::int32_t kLiftMaxVelocityRpm = 100;
constexpr std::uint32_t kLiftTimeoutMs = 1800;
constexpr double kRaisedLiftDeg = -600;

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

// 单条自动路线的运行上下文：统一 30 秒预算、退出判断与带场地镜像的底盘动作。
// 路线 1 的四个预设与路线 2 共用这些动作原语；夹爪、升降、Toggle 等机构动作仍写在各自路线里。
class RouteRun {
public:
    RouteRun(bool allow_driver, RoutePoint (*transform)(RoutePoint))
        : m_allow_driver(allow_driver), m_transform(transform) {}

    bool can_run() {
        // Override 2.1 <R9>：禁用或切换阶段后立即停止动作。
        // allow_driver 供手动阶段的按 A 入口运行同一条路线。
        return !pros::competition::is_disabled() &&
               (m_allow_driver || pros::competition::is_autonomous()) && !m_timer.isDone();
    }

    std::uint32_t time_left_ms() {
        return m_timer.getTimeLeft();
    }

    // 以下 x/y/theta 一律写 Red Left 侧的原始值，由 transform 转换到当前预设。
    void set_start(float x, float y, float theta) {
        const auto point = at(x, y, theta);
        chassis::controller().setPose(point.x, point.y, point.theta);
    }

    void move_to_point(float x, float y, int timeout_in, lemlib::MoveToPointParams params = {}) {
        const auto point = at(x, y, 0.0F);
        chassis::controller().moveToPoint(point.x, point.y, timeout_in, params);
    }

    void move_to_pose(float x, float y, float theta, int timeout_in,
                      lemlib::MoveToPoseParams params = {}) {
        const auto point = at(x, y, theta);
        chassis::controller().moveToPose(point.x, point.y, point.theta, timeout_in, params);
    }

    void turn_to_heading(float theta, int timeout_in) {
        chassis::controller().turnToHeading(at(0.0F, 0.0F, theta).theta, timeout_in);
    }

    void swing_to_heading(float theta, lemlib::DriveSide locked_side, int timeout_in) {
        chassis::controller().swingToHeading(at(0.0F, 0.0F, theta).theta, locked_side, timeout_in);
    }

    bool wait_for_drive() {
        while (can_run() && chassis::controller().isInMotion()) {
            pros::delay(kPollMs);
        }
        return can_run();
    }

    // 等运动结束后检查是否停在目标点的放置容差内。
    bool wait_at_point(float x, float y) {
        if (!wait_for_drive()) {
            return false;
        }
        const auto point = at(x, y, 0.0F);
        const auto pose = chassis::controller().getPose();
        return std::hypot(pose.x - point.x, pose.y - point.y) <= kPlacementToleranceIn;
    }

    // 原地等待指定毫秒数（分片轮询），禁用、切模式或时间预算用尽时提前返回 false。
    bool wait_ms(std::uint32_t duration_ms) {
        const auto start_ms = pros::millis();
        while (can_run() && pros::millis() - start_ms < duration_ms) {
            pros::delay(kPollMs);
        }
        return can_run();
    }

    // 离目标还剩约 lead_ms 的行驶时间时先合爪，不再等停稳；一直没触发则在运动结束后合。
    // 接近速度用约 50 ms 的位姿变化估算，速度太低时只用“运动结束”兜底。
    bool close_before_stop(float x, float y, std::uint32_t lead_ms) {
        const auto point = at(x, y, 0.0F);
        constexpr std::uint32_t kSpeedWindowMs = 50;
        const auto start_pose = chassis::controller().getPose();
        auto window_distance = std::hypot(start_pose.x - point.x, start_pose.y - point.y);
        auto window_ms = pros::millis();
        while (can_run() && chassis::controller().isInMotion()) {
            pros::delay(kPollMs);
            const auto pose = chassis::controller().getPose();
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
    }

private:
    RoutePoint at(float x, float y, float theta) const {
        return m_transform({x, y, theta});
    }

    bool m_allow_driver;
    RoutePoint (*m_transform)(RoutePoint);
    lemlib::Timer m_timer{kRouteBudgetMs};
};

// 四个预设共用同一条 Red Left 路线，只按场地镜像变换每个点位与朝向；
// 夹爪、升降、Toggle 动作与路径形状保持不变。
void run_route(bool allow_driver, RoutePoint (*transform)(RoutePoint)) {
    // 路线开始时夹紧，固定预装 Pin。
    claw::close();
    RouteRun run(allow_driver, transform);
    if (!run.can_run()) {
        return;
    }

    run.set_start(-62.5F, -7.5F, 90.0F);
    // 开局用后臂撞己方 Toggle：前进顶到 Toggle 就下放，倒车退出后抬起，连做两趟再走后续路线。
    // 顶住 Toggle 时不一定能到点，每段只等运动结束，不检查是否到位。
    // 撞击点比原来的 -50 再前 4 in（-46），顶得更久；行程按实车效果调整。
    constexpr int kTogglePasses = 2;
    for (int pass = 0; pass < kTogglePasses; ++pass) {
        run.move_to_point(-46.0F, -7.5F, 1100);
        if (!run.wait_for_drive()) {
            return;
        }
        toggle::down();
        run.move_to_point(-67.0F, -7.5F, 1350, {.forwards = false});
        if (!run.wait_for_drive()) {
            return;
        }
        toggle::up();
    }
    // 第一次放 Pin：驶向红 Goal，顶进去后开爪放下预装 Pin。
    // 实车在碰到 Goal 前就停住，目标点沿车头方向再顶 3 in，timeout 加 300 ms。
    run.move_to_pose(-49.0F, -22.3F, 150.0F, 1800, {.lead = 0.5F});
    if (!run.wait_at_point(-49.0F, -22.3F)) {
        return;
    }
    claw::open();
    // 停留 0.4 s 让预装 Pin 落稳，再倒车退出红 Goal。
    if (!run.wait_ms(400)) {
        return;
    }
    // 倒车退出红 Goal。
    run.move_to_pose(-60.0F, -8.0F, 95.0F, 1850, {.forwards = false, .lead = 0.2F});
    if (!run.wait_for_drive()) {
        return;
    }

    // 不再抢黄 Pin：倒车退出红 Goal 后直接回场地中央停泊，全程不升升降。
    // 停稳后确认升降在启动零位，结束自动路线。
    run.move_to_pose(-9.8F, -4.9F, 240.0F, 3000, {.forwards = false, .lead = 0.6F});
    if (!run.wait_for_drive()) {
        return;
    }
    lifting::move_to({0.0, 0.0}, kLiftMaxVelocityRpm, std::min(kLiftTimeoutMs, run.time_left_ms()),
                     allow_driver);
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

// Plan 2 的 Red Left：与路线 1 相同的两次 Toggle 撞击后，先向红 Goal 放预装 Pin，
// 倒车后抢内侧 Pin（行驶途中升起升降），再转回红 Goal 放 Pin，最后倒车回场地中央。
// 路径坐标取自 Path Planner 导出；红 Goal 沿用路线 1 的实车修正点，夹爪与升降沿用路线 1 的节奏。
void run_red_left_2(bool allow_driver) {
    claw::close();
    RouteRun run(allow_driver, as_is);
    if (!run.can_run()) {
        return;
    }

    run.set_start(-62.5F, -7.5F, 90.0F);
    // 第一动作沿用路线 1：连续两次顶 Toggle，顶到就下放，倒车退出后抬起；
    // 撞击点同样前移到 -46（多顶 4 in），行程按实车效果调整。
    constexpr int kTogglePasses = 2;
    for (int pass = 0; pass < kTogglePasses; ++pass) {
        run.move_to_point(-46.0F, -7.5F, 1100);
        if (!run.wait_for_drive()) {
            return;
        }
        toggle::down();
        run.move_to_point(-67.0F, -7.5F, 1050, {.forwards = false});
        if (!run.wait_for_drive()) {
            return;
        }
        toggle::up();
    }

    // 第一次放 Pin：顶进红 Goal 后开爪，目标点与顶入方式同路线 1。
    run.move_to_pose(-49.0F, -22.3F, 150.0F, 1850, {.lead = 0.5F});
    if (!run.wait_at_point(-49.0F, -22.3F)) {
        return;
    }
    claw::open();

    // 倒车退出红 Goal。
    run.move_to_pose(-60.0F, -8.0F, 95.0F, 1600, {.forwards = false, .lead = 0.2F});
    if (!run.wait_for_drive()) {
        return;
    }

    // 抢内侧 Pin：底盘行驶与升起升降同时开始，谁先完成就等另一方，两边都到位后再合爪。
    run.move_to_pose(-51.3F, -45.2F, 130.0F, 3100, {.lead = 0.4F});
    lifting::move_to({kRaisedLiftDeg, kRaisedLiftDeg}, kLiftMaxVelocityRpm,
                     std::min(kLiftTimeoutMs, run.time_left_ms()), allow_driver);
    if (!run.wait_at_point(-51.3F, -45.2F)) {
        return;
    }
    claw::close();

    // 转向红 Goal 后放 Pin：升降已在抢 Pin 时到位，停稳即开爪。
    run.swing_to_heading(-20.8F, lemlib::DriveSide::LEFT, 3000);
    if (!run.wait_for_drive()) {
        return;
    }
    run.move_to_point(-45.9F, -29.7F, 1100);
    if (!run.wait_at_point(-45.9F, -29.7F)) {
        return;
    }
    claw::open();

    // 倒车回场地中央，停稳后把升降降到底（启动零位），结束路线 2。
    run.move_to_point(-14.2F, -5.8F, 2600, {.forwards = false});
    if (!run.wait_for_drive()) {
        return;
    }
    lifting::move_to({0.0, 0.0}, kLiftMaxVelocityRpm, std::min(kLiftTimeoutMs, run.time_left_ms()),
                     allow_driver);
}

void run_red_down_2(bool) {}

void run_blue_right_2(bool) {}

void run_blue_up_2(bool) {}

// 路线 3：恢复旧版路线 1 的抢黄 Pin、升升降并放第二个 Pin 的动作，保留两次 Toggle 撞击；
// 与路线 1 相比，红 Goal 目标点 x/y 各外移 0.4、约远离场地中心 2.9 in，
// 开爪后停留 0.3 s（路线 1 为 0.4 s）。
void run_red_left_3(bool allow_driver) {
    claw::close();
    RouteRun run(allow_driver, as_is);
    if (!run.can_run()) {
        return;
    }

    run.set_start(-62.5F, -7.5F, 90.0F);
    constexpr int kTogglePasses = 2;
    for (int pass = 0; pass < kTogglePasses; ++pass) {
        run.move_to_point(-46.0F, -7.5F, 1100);
        if (!run.wait_for_drive()) {
            return;
        }
        toggle::down();
        run.move_to_point(-67.0F, -7.5F, 1350, {.forwards = false});
        if (!run.wait_for_drive()) {
            return;
        }
        toggle::up();
    }

    // 第一次放 Pin：目标点比路线 1 的落点 x/y 各减 0.4，开爪后停留 0.3 s 再倒车。
    run.move_to_pose(-51.58F, -23.70F, 150.0F, 1800, {.lead = 0.5F});
    if (!run.wait_at_point(-51.58F, -23.70F)) {
        return;
    }
    claw::open();
    if (!run.wait_ms(300)) {
        return;
    }

    // 倒车退出红 Goal。
    run.move_to_pose(-60.0F, -8.0F, 95.0F, 1850, {.forwards = false, .lead = 0.2F});
    if (!run.wait_for_drive()) {
        return;
    }

    // 夹黄 Pin：抓取点沿指向场地中心的直线内移 3 in；离目标还剩约 150 ms 行程时合爪。
    constexpr std::uint32_t kGrabLeadMs = 150;
    run.move_to_pose(-25.42F, -19.48F, 128.0F, 2000, {.lead = 0.4F});
    if (!run.close_before_stop(-25.42F, -19.48F, kGrabLeadMs)) {
        return;
    }

    // 转回红 Goal，把黄 Pin 放到同一个 Goal 上；升降比第一次高一些。
    // 实车调头后方向偏左，回程朝向往右收 10°（-100 → -90）。
    run.turn_to_heading(-90.0F, 1300);
    if (!run.wait_for_drive()) {
        return;
    }
    run.move_to_pose(-42.0F, -19.6F, -83.0F, 1200);
    lifting::move_to({kRaisedLiftDeg, kRaisedLiftDeg}, kLiftMaxVelocityRpm,
                     std::min(kLiftTimeoutMs, run.time_left_ms()), allow_driver);
    if (!run.wait_at_point(-42.0F, -19.6F)) {
        return;
    }
    claw::open();

    // 倒车去场地中央，停稳后把升降降到底（启动零位），结束路线 3。
    run.move_to_pose(-9.8F, -4.9F, 240.0F, 2650, {.forwards = false, .lead = 0.6F});
    if (!run.wait_for_drive()) {
        return;
    }
    lifting::move_to({0.0, 0.0}, kLiftMaxVelocityRpm, std::min(kLiftTimeoutMs, run.time_left_ms()),
                     allow_driver);
}

void run_red_down_3(bool) {}

void run_blue_right_3(bool) {}

void run_blue_up_3(bool) {}

using RouteFn = void (*)(bool);

// 预设 × 方案路线表；未实现的组合是空函数，运行后立即停车。
constexpr RouteFn kRoutes[static_cast<std::size_t>(brain_ui::AutonPreset::Count)]
                         [static_cast<std::size_t>(brain_ui::AutonPlan::Count)] = {
                             {run_red_left, run_red_left_2, run_red_left_3},
                             {run_red_down, run_red_down_2, run_red_down_3},
                             {run_blue_right, run_blue_right_2, run_blue_right_3},
                             {run_blue_up, run_blue_up_2, run_blue_up_3},
};

void run_impl(bool allow_driver) {
    claw::unlock_startup();
    const auto preset = static_cast<std::size_t>(brain_ui::selected_auton());
    const auto plan = static_cast<std::size_t>(brain_ui::selected_plan());
    kRoutes[preset][plan](allow_driver);
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
