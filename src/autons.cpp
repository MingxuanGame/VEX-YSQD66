#include "autons.hpp"

#include "brain_ui.hpp"
#include "chassis.h"
#include "claw.hpp"
#include "lemlib/api.hpp"
#include "lifting.hpp"
#include "pros/rtos.hpp"

// 在 static/ 放入 JerryIO / LemLib Path Planner 导出的路径文本后取消注释：
// ASSET(example_txt);

namespace autons {
namespace {

void run_red_left() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 预留：红方左路线（在此组装 LemLib Path Planner 路径与 C++ Custom 升降/夹爪代码）
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
