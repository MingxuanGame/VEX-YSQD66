#include "test_common.hpp"
#include "collision/collision_detector.hpp"
#include "collision/collision_recovery.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"
#include "sim/core/sim_clock.hpp"

namespace {

struct FakeChassis : robot::Chassis {
    double current = 0.0;
    double velocity = 0.0;
    int last_left = 0;
    int last_right = 0;
    bool stopped = false;

    void setDrive(int left, int right) override {
        last_left = left;
        last_right = right;
    }

    void stop(bool) override {
        stopped = true;
        last_left = 0;
        last_right = 0;
    }

    double getLeftCurrent() const override {
        return current;
    }

    double getRightCurrent() const override {
        return current;
    }

    double getLeftVelocity() const override {
        return velocity;
    }

    double getRightVelocity() const override {
        return velocity;
    }

    double getLeftPosition() const override {
        return 0.0;
    }

    double getRightPosition() const override {
        return 0.0;
    }

    void tarePosition() override {}
};

struct FakeImu : robot::Imu {
    double gyro = 0.0;

    double getHeading() const override {
        return 0.0;
    }

    double getRotation() const override {
        return 0.0;
    }

    double getGyroRateZ() const override {
        return gyro;
    }
};

// 让电流在 1000/4000 之间跳变，制造“对方在动”的高方差，用于 Car 分类。
struct AlternatingChassis : robot::Chassis {
    mutable int calls = 0;

    void setDrive(int, int) override {}

    void stop(bool) override {}

    double getLeftCurrent() const override {
        const int sample = calls++ / 2;
        return (sample % 2 == 0) ? 1000.0 : 4000.0;
    }

    double getRightCurrent() const override {
        const int sample = calls++ / 2;
        return (sample % 2 == 0) ? 1000.0 : 4000.0;
    }

    double getLeftVelocity() const override {
        return 0.0;
    }

    double getRightVelocity() const override {
        return 0.0;
    }

    double getLeftPosition() const override {
        return 0.0;
    }

    double getRightPosition() const override {
        return 0.0;
    }

    void tarePosition() override {}
};

struct CarImu : robot::Imu {
    double getHeading() const override {
        return 0.0;
    }

    double getRotation() const override {
        return 0.0;
    }

    double getGyroRateZ() const override {
        return 20.0;
    }
};

// 电流恒定为锥的范围，速度在观察窗口后恢复，用于 Cone 分类。
struct ConeChassis : robot::Chassis {
    mutable int velocity_calls = 0;

    void setDrive(int, int) override {}

    void stop(bool) override {}

    double getLeftCurrent() const override {
        return 1800.0;
    }

    double getRightCurrent() const override {
        return 1800.0;
    }

    double getLeftVelocity() const override {
        return (velocity_calls++ < 2) ? 0.0 : 30.0;
    }

    double getRightVelocity() const override {
        return (velocity_calls++ < 2) ? 0.0 : 30.0;
    }

    double getLeftPosition() const override {
        return 0.0;
    }

    double getRightPosition() const override {
        return 0.0;
    }

    void tarePosition() override {}
};

struct ZeroImu : robot::Imu {
    double getHeading() const override {
        return 0.0;
    }

    double getRotation() const override {
        return 0.0;
    }

    double getGyroRateZ() const override {
        return 0.0;
    }
};

int g_retries = 0;

bool retryOnce() {
    ++g_retries;
    return true;
}

} // namespace

void test_collision() {
    using namespace collision;

    sim::init_clock();

    FakeChassis chassis;
    FakeImu imu;
    CollisionDetector detector(chassis, imu);

    // 高电流 + 低转速 -> 触发；无晃动 -> 归为 Deadzone。
    chassis.current = 3000.0;
    chassis.velocity = 0.0;
    CHECK(detector.isTriggered());

    CollisionFeatures features;
    CHECK(detector.detect(&features) == CollisionType::Deadzone);
    CHECK(features.avg_current > 2500.0);

    // 非阻塞观察：start/update 不应该阻塞主循环。
    CollisionDetector async_detector(chassis, imu);
    async_detector.startObservation(sim::millis());
    for (int i = 0; i < 30 && !async_detector.isObservationDone(); ++i) {
        sim::delay(10);
        async_detector.updateObservation(sim::millis());
    }
    CHECK(async_detector.isObservationDone());
    CHECK(async_detector.getObservationResult() == CollisionType::Deadzone);

    // 高电流方差 + 持续晃动 -> Car。
    AlternatingChassis car_chassis;
    CarImu car_imu;
    CollisionDetector car_detector(car_chassis, car_imu);
    CollisionFeatures car_features;
    const CollisionType car_type = car_detector.detect(&car_features);
    CHECK(car_type == CollisionType::Car);
    CHECK(car_features.gyro_above_count >= 10);
    CHECK(car_features.current_variance > 200000.0);

    // 中等电流 + 无晃动 + 速度恢复 -> Cone。
    ConeChassis cone_chassis;
    ZeroImu cone_imu;
    CollisionDetector cone_detector(cone_chassis, cone_imu);
    CollisionFeatures cone_features;
    const CollisionType cone_type = cone_detector.detect(&cone_features);
    CHECK(cone_type == CollisionType::Cone);
    CHECK(cone_features.velocity_at_end > 20.0);

    // 注入模式应覆盖真实传感器，并支持负数退出。
    detector.injectFakeData(3000.0, 0.0, 0.0);
    CHECK(detector.isTriggered());
    detector.injectFakeData(-1.0, -1.0, -1.0);
    CHECK(detector.isTriggered()); // 退出注入后仍读取 fake chassis

    // 恢复：路通时直接后退 + 重试。
    chassis.current = 0.0;
    CollisionRecovery recovery(chassis);
    CHECK(recovery.escapeCollision());

    // 非阻塞脱困：主循环逐帧推进，不调用 delay。
    CollisionRecovery async_recovery(chassis);
    CHECK(async_recovery.startEscapeAsync(sim::millis()));
    for (int i = 0; i < 80 && async_recovery.isEscaping(); ++i) {
        sim::delay(10);
        async_recovery.updateEscapeAsync(sim::millis());
    }
    CHECK(!async_recovery.isEscaping());
    CHECK(async_recovery.escapeSucceeded());

    CHECK(recovery.recover(CollisionType::Car, retryOnce, nullptr, 5000));
    CHECK(g_retries == 1);

    // 防推挤：持续高电流 + 静止 500ms 后应反向卸力。
    chassis.current = 3000.0;
    chassis.velocity = 0.0;
    CHECK(!recovery.updateAntiPush(1000));
    CHECK(recovery.updateAntiPush(2000));
    CHECK(chassis.last_left == -60);
    CHECK(chassis.last_right == -60);

    // 没有底盘时不能崩溃，而是安全返回 false。
    CollisionRecovery no_chassis;
    CHECK(!no_chassis.escapeCollision());
    CHECK(!no_chassis.isPathClear());
    CHECK(!no_chassis.recover(CollisionType::Cone, nullptr, nullptr));
}
