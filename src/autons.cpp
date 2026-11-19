#include "autons.hpp"

#include "chassis.h"
#include "claw.hpp"
#include "lemlib/api.hpp"
#include "lifting.hpp"
#include "pros/rtos.hpp"

// 在 static/ 放入 JerryIO / LemLib Path Planner 导出的路径文本后取消注释：
// ASSET(example_txt);

namespace autons {

void run() {
    auto &chassis = chassis::controller();
    chassis.setPose(0.0F, 0.0F, 0.0F);

    // 在此处组装 LemLib Path Planner 路线与 C++ Custom 升降/夹爪动作（当前默认停车）。

    chassis::stop();
    lifting::stop();
    claw::stop();
}

} // namespace autons
