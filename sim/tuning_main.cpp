#include "collision/collision_tuning.hpp"
#include "tuning/tuning.hpp"
#include "sim/core/sim_clock.hpp"

#include <cstdio>
#include <cstdlib>

/**
 * @file tuning_main.cpp
 * @brief 只测试调参链路的仿真入口：
 *        ParamsManager -> CollisionTuning 注册 -> 修改 -> ParamApply 推送 -> 存盘/读盘。
 *        不涉及底盘、电机或碰撞检测。
 */

namespace {

struct FakeControllerConfig {
    int collision_current = 0;
    double collision_gyro = 0.0;
    int escape_distance = 0;
};

FakeControllerConfig g_fake;

void applyCurrent(const tuning::TunableParam &p) {
    g_fake.collision_current = p.getIntValue();
}

void applyGyro(const tuning::TunableParam &p) {
    g_fake.collision_gyro = p.getDoubleValue();
}

void applyEscapeDistance(const tuning::TunableParam &p) {
    g_fake.escape_distance = p.getIntValue();
}

void printConfig(const char *tag) {
    std::printf("%s: current=%d gyro=%.2f escape=%d\n", tag, g_fake.collision_current,
                g_fake.collision_gyro, g_fake.escape_distance);
}

} // namespace

int main() {
    sim::init_clock();

    tuning::ParamsManager manager;
    const int registered = collision::CollisionTuning::registerAll(manager);

    std::printf("registered=%d size=%d\n", registered, manager.getSize());
    if (registered != 6 || manager.getSize() != 6) {
        std::printf("FAIL: expected 6 collision parameters\n");
        return 1;
    }

    if (!tuning::ParamApply::bind("coll_current", applyCurrent) ||
        !tuning::ParamApply::bind("coll_gyro", applyGyro) ||
        !tuning::ParamApply::bind("esc_dist", applyEscapeDistance)) {
        std::printf("FAIL: ParamApply::bind failed\n");
        return 1;
    }

    auto *current = manager.getParam("coll_current");
    auto *gyro = manager.getParam("coll_gyro");
    auto *escape = manager.getParam("esc_dist");
    if (!current || !gyro || !escape) {
        std::printf("FAIL: parameter lookup failed\n");
        return 1;
    }

    std::printf("defaults: current=%d gyro=%.2f escape=%d\n", current->getIntValue(),
                gyro->getDoubleValue(), escape->getIntValue());

    // coll_current: 2500 -> 3000（切到百位，按 5 次）
    current->moveDigitPos(tuning::MoveDirection::Left);
    current->moveDigitPos(tuning::MoveDirection::Left);
    for (int i = 0; i < 5; ++i)
        current->incParam();

    // coll_gyro: 15.0 -> 17.0
    gyro->incParam();
    gyro->incParam();

    // esc_dist: 3 -> 4
    escape->incParam();

    tuning::ParamApply::applyAllDirty(manager);
    printConfig("after apply");

    if (g_fake.collision_current != 3000 || g_fake.collision_gyro != 17.0 ||
        g_fake.escape_distance != 4) {
        std::printf("FAIL: values were not applied to fake controller\n");
        return 1;
    }

    const char *path = "tuning_sim_test.txt";
    if (!manager.saveToFile(path)) {
        std::printf("FAIL: saveToFile failed\n");
        return 1;
    }

    tuning::ParamsManager loaded;
    collision::CollisionTuning::registerAll(loaded);
    const int loaded_count = loaded.loadFromFile(path);
    std::remove(path);

    std::printf("loaded=%d\n", loaded_count);
    if (loaded_count != 6) {
        std::printf("FAIL: expected 6 loaded parameters\n");
        return 1;
    }

    // 模拟“重启后读盘”：值应恢复，并且是 dirty，等待 apply 推送。
    tuning::ParamApply::applyAllDirty(loaded);
    printConfig("after load+apply");

    if (g_fake.collision_current != 3000 || g_fake.collision_gyro != 17.0 ||
        g_fake.escape_distance != 4) {
        std::printf("FAIL: loaded values were not applied\n");
        return 1;
    }

    // 恢复默认值，验证 resetAll 也走 apply 链路。
    loaded.resetAll();
    tuning::ParamApply::applyAllDirty(loaded);
    printConfig("after reset");

    if (g_fake.collision_current != 2500 || g_fake.collision_gyro != 15.0 ||
        g_fake.escape_distance != 3) {
        std::printf("FAIL: reset values were not applied\n");
        return 1;
    }

    std::printf("tuning simulation passed\n");
    return 0;
}
