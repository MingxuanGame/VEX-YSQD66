#include "test_common.hpp"
#include "control/control_loop.hpp"
#include "control/drive_controller.hpp"
#include "io/controller.hpp"
#include "robot/battery.hpp"
#include "robot/chassis.hpp"
#include "robot/display.hpp"
#include "robot/imu.hpp"
#include "robot/mechanism.hpp"
#include "tuning/params_manager.hpp"

#include <cstdlib>
#include <cstring>

namespace {

struct ControlFakeController : io::Controller {
    bool connected = true;
    int left_y = 0;
    int right_x = 0;
    bool y = false;
    bool b = false;
    bool a = false;
    bool l1 = false;
    bool l2 = false;
    bool r1 = false;
    bool r2 = false;
    bool x = false;
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;

    bool isConnected() const override {
        return connected;
    }

    int analog(io::Analog channel) const override {
        if (channel == io::Analog::LeftY)
            return left_y;
        if (channel == io::Analog::RightX)
            return right_x;
        return 0;
    }

    bool digital(io::Digital button) const override {
        if (button == io::Digital::Y)
            return y;
        if (button == io::Digital::B)
            return b;
        if (button == io::Digital::A)
            return a;
        if (button == io::Digital::L1)
            return l1;
        if (button == io::Digital::L2)
            return l2;
        if (button == io::Digital::R1)
            return r1;
        if (button == io::Digital::R2)
            return r2;
        if (button == io::Digital::X)
            return x;
        if (button == io::Digital::Up)
            return up;
        if (button == io::Digital::Down)
            return down;
        if (button == io::Digital::Left)
            return left;
        if (button == io::Digital::Right)
            return right;
        return false;
    }
};

struct ControlFakeChassis : robot::Chassis {
    int left = 0;
    int right = 0;
    double left_current = 0.0;
    double right_current = 0.0;
    double velocity = 0.0;
    bool healthy = true;

    bool isHealthy() const override {
        return healthy;
    }

    void setDrive(int l, int r) override {
        left = l;
        right = r;
    }

    void stop(bool) override {
        left = 0;
        right = 0;
    }

    double getLeftCurrent() const override {
        return left_current;
    }

    double getRightCurrent() const override {
        return right_current;
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

struct ControlFakeImu : robot::Imu {
    double heading = 0.0;
    bool healthy = true;

    bool isHealthy() const override {
        return healthy;
    }

    double getHeading() const override {
        return heading;
    }

    double getRotation() const override {
        return heading;
    }

    double getGyroRateZ() const override {
        return 0.0;
    }
};

struct ControlFakeBattery : robot::Battery {
    int voltage_mv = 12000;

    int getVoltageMv() const override {
        return voltage_mv;
    }
};

struct ControlFakeMechanism : robot::Mechanism {
    int clamp_open = 0;
    int clamp_close = 0;
    int push_extend = 0;
    int push_retract = 0;
    int lift_up = 0;
    int lift_down = 0;
    int lift_hold = 0;
    double current_ma = 0.0;
    bool healthy = true;

    bool isHealthy() const override {
        return healthy;
    }

    void liftTo(double, int) override {}

    void liftManual(int speed) override {
        if (speed > 0)
            ++lift_up;
        if (speed < 0)
            ++lift_down;
    }

    void liftHold() override {
        ++lift_hold;
    }

    void clampOpen() override {
        ++clamp_open;
    }

    void clampClose() override {
        ++clamp_close;
    }

    void clampSetAngle(int) override {}

    void pushExtend() override {
        ++push_extend;
    }

    void pushRetract() override {
        ++push_retract;
    }

    double getCurrentMa() const override {
        return current_ma;
    }
};

struct ControlFakeDisplay : robot::Display {
    int print_calls = 0;
    char last[40] = {};

    void clearLine(int) override {}

    void printLine(int, const char *text) override {
        ++print_calls;
        if (text) {
            std::strncpy(last, text, sizeof(last) - 1);
            last[sizeof(last) - 1] = '\0';
        }
    }
};

} // namespace

void test_control() {
    ControlFakeController controller;
    ControlFakeChassis chassis;
    ControlFakeImu imu;
    ControlFakeBattery battery;
    ControlFakeMechanism mechanism;
    ControlFakeDisplay display;
    tuning::ParamsManager params;
    CHECK(params.addParam("ui_kp", 1.0, 0.0, 10.0, tuning::ParamType::Double,
                          tuning::ParamGroup::PIDGroup, -2, 2) >= 0);
    CHECK(params.addParam("ui_ki", 0.2, 0.0, 10.0, tuning::ParamType::Double,
                          tuning::ParamGroup::PIDGroup, -2, 2) >= 0);

    control::ControlLoop loop(controller, chassis, imu, &battery);
    loop.setMechanism(&mechanism);
    loop.setDisplay(&display);
    loop.setParamsManager(&params);

    uint32_t now = 0;
    controller.left_y = 50;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::Drive);
    CHECK(chassis.left > 0);
    CHECK(chassis.left <= 100);
    CHECK(chassis.right > 0);

    // 底盘/IMU/机构健康信号失败时必须进入 SafeWait；恢复健康后由 reset 回到 Drive。
    controller.left_y = 0;
    const auto expect_fault = [&](bool &healthy) {
        healthy = false;
        now += 20;
        loop.update(now);
        CHECK(loop.getMode() == control::Mode::SafeWait);
        CHECK(chassis.left == 0 && chassis.right == 0);
        healthy = true;
        loop.reset();
        CHECK(loop.getMode() == control::Mode::Drive);
    };
    expect_fault(chassis.healthy);
    expect_fault(imu.healthy);
    expect_fault(mechanism.healthy);

    controller.left_y = 50;
    now += 20;
    loop.update(now);

    // Drive 模式并发机构控制：R1 夹爪、R2 推板、L1 抬升。
    controller.r1 = true;
    now += 20;
    loop.update(now);
    controller.r1 = false;
    now += 20;
    loop.update(now);
    CHECK(mechanism.clamp_close == 1);

    controller.r1 = true;
    now += 20;
    loop.update(now);
    controller.r1 = false;
    CHECK(mechanism.clamp_open == 1);

    controller.r2 = true;
    now += 20;
    loop.update(now);
    controller.r2 = false;
    CHECK(mechanism.push_extend == 1);

    controller.l1 = true;
    now += 20;
    loop.update(now);
    controller.l1 = false;
    CHECK(mechanism.lift_up >= 1);

    // 长按 Y 进入 TUNE，底盘锁死。
    controller.left_y = 0;
    controller.y = true;
    for (int i = 0; i < 35; ++i) {
        now += 20;
        loop.update(now);
    }
    CHECK(loop.getMode() == control::Mode::Tune);
    CHECK(chassis.left == 0);
    CHECK(chassis.right == 0);
    CHECK(display.print_calls > 0);

    // Tune 点动：按住 L1 才允许低速驱动，松手立即刹车。
    controller.l1 = true;
    controller.left_y = 50;
    now += 20;
    loop.update(now);
    CHECK(chassis.left > 0);
    CHECK(chassis.left <= 35);
    controller.l1 = false;
    controller.left_y = 0;
    now += 20;
    loop.update(now);
    CHECK(chassis.left == 0);
    CHECK(chassis.right == 0);

    // Tune 内导航和微调：Down 选下一个参数，X 减一档。
    controller.down = true;
    now += 20;
    loop.update(now);
    controller.down = false;
    CHECK(params.getSelectedIndex() == 1);
    controller.x = true;
    now += 20;
    loop.update(now);
    controller.x = false;
    CHECK(params.getSelectedParam() != nullptr);
    CHECK(params.getSelectedParam()->isDirty());

    // B 退出 TUNE，回到 DRIVE。
    controller.y = false;
    controller.b = true;
    now += 20;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::Drive);
    controller.b = false;

    // 断联进入 SafeWait，输出必须为零。
    controller.connected = false;
    controller.left_y = 100;
    now += 20;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::SafeWait);
    CHECK(chassis.left == 0);
    CHECK(chassis.right == 0);

    // 恢复连接后按 A 回到 DRIVE。
    controller.connected = true;
    controller.left_y = 0;
    controller.a = true;
    now += 20;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::Drive);
    controller.a = false;

    // 自动模式屏蔽摇杆。
    loop.startAuto();
    controller.left_y = 100;
    now += 20;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::Auto);
    loop.endAuto();
    CHECK(chassis.left == 0);
    CHECK(chassis.right == 0);
    now += 20;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::Drive);

    // 紧急脱困：输出受限，松手后恢复普通驾驶。
    controller.left_y = 0;
    const int lift_before_escape = mechanism.lift_up;
    controller.l1 = true;
    controller.l2 = true;
    now += 20;
    loop.update(now);
    CHECK(loop.getMode() == control::Mode::Drive);
    CHECK(mechanism.lift_up == lift_before_escape);
    CHECK(chassis.left >= -60 && chassis.left <= 60);
    CHECK(chassis.right >= -60 && chassis.right <= 60);
    controller.l1 = false;
    controller.l2 = false;

    // 防推挤：高电流、零速度、操作手没有硬推 -> 卸力。
    chassis.left_current = 3000.0;
    chassis.right_current = 3000.0;
    chassis.velocity = 0.0;
    controller.left_y = 0;
    for (int i = 0; i < 35; ++i) {
        now += 20;
        loop.update(now);
    }
    CHECK(chassis.left < 0);
    CHECK(chassis.right < 0);

    // 操作手猛推杆 -> 防推挤立即让步。
    controller.left_y = 100;
    for (int i = 0; i < 3; ++i) {
        now += 20;
        loop.update(now);
    }
    CHECK(chassis.left > 0);

    // 电流预算：四轮 24A + 机构 5A，超过 20A 后按比例缩小。
    ControlFakeChassis budget_chassis;
    budget_chassis.left_current = 12000.0;
    budget_chassis.right_current = 12000.0;
    budget_chassis.velocity = 0.0;
    control::DriveConfig budget_config;
    budget_config.max_total_current_ma = 20000.0;
    control::DriveController budget_driver(budget_chassis, budget_config);
    budget_driver.setAuxCurrentMa(5000.0);
    io::InputState budget_input;
    budget_input.isConnected = true;
    budget_input.leftY = 100;
    budget_driver.update(budget_input, now);
    CHECK(budget_driver.getLastCurrentLimitScale() < 1.0);
    CHECK(budget_driver.getLastLeftOutput() < 100);

    // 航向 Assist：按住 X 后锁住当前航向，IMU 偏转时自动修 turn。
    ControlFakeChassis assist_chassis;
    ControlFakeImu assist_imu;
    control::DriveController assist_driver(assist_chassis);
    assist_driver.setImu(&assist_imu);
    io::InputState assist_input;
    assist_input.isConnected = true;
    assist_input.x.update(true, now);
    assist_driver.update(assist_input, now);
    CHECK(assist_driver.isAssistActive());
    assist_imu.heading = 10.0;
    assist_input.x.update(true, now + 20);
    assist_driver.update(assist_input, now + 20);
    CHECK(assist_driver.getLastLeftOutput() != assist_driver.getLastRightOutput());

    // 外部视觉目标航向注入：目标 30°，当前 0°，Assist 应向右修。
    assist_driver.clearAssistTarget();
    assist_imu.heading = 0.0;
    assist_driver.setAssistTargetHeading(30.0);
    assist_input.x.update(true, now + 40);
    assist_driver.update(assist_input, now + 40);
    CHECK(assist_driver.isAssistActive());
    CHECK(assist_driver.getLastLeftOutput() < assist_driver.getLastRightOutput());

    // IMU 未就绪时 Assist 必须退出，不能拿 0 度继续修航向。
    assist_imu.healthy = false;
    assist_input.x.update(true, now + 50);
    assist_driver.update(assist_input, now + 50);
    CHECK(!assist_driver.isAssistActive());

    // 底盘健康信号失败时直接停车。
    ControlFakeChassis unhealthy_chassis;
    unhealthy_chassis.healthy = false;
    control::DriveController unhealthy_driver(unhealthy_chassis);
    io::InputState unhealthy_input;
    unhealthy_input.isConnected = true;
    unhealthy_input.leftY = 100;
    unhealthy_driver.update(unhealthy_input, now);
    CHECK(unhealthy_chassis.left == 0 && unhealthy_chassis.right == 0);

    // 输出斜率限制：单周期最多变化 max_delta_per_update。
    ControlFakeChassis slew_chassis;
    control::DriveConfig slew_config;
    slew_config.deadzone = 0;
    slew_config.expo = 1.0;
    slew_config.max_delta_per_update = 20;
    control::DriveController slew_driver(slew_chassis, slew_config);
    io::InputState slew_input;
    slew_input.isConnected = true;
    slew_input.leftY = 100;
    slew_driver.update(slew_input, now);
    CHECK(std::abs(slew_chassis.left) <= 20);
    slew_driver.update(slew_input, now + 20);
    CHECK(std::abs(slew_chassis.left) <= 40);

    // 低电压手动补偿：10V 时 50 输出应放大到约 60。
    ControlFakeChassis low_battery_chassis;
    ControlFakeBattery low_battery;
    low_battery.voltage_mv = 10000;
    control::DriveConfig low_battery_config;
    low_battery_config.deadzone = 0;
    low_battery_config.expo = 1.0;
    low_battery_config.max_delta_per_update = 127;
    control::DriveController low_battery_driver(low_battery_chassis, low_battery_config);
    low_battery_driver.setBattery(&low_battery);
    io::InputState low_battery_input;
    low_battery_input.isConnected = true;
    low_battery_input.leftY = 50;
    low_battery_driver.update(low_battery_input, now);
    CHECK(low_battery_driver.getLastLeftOutput() > 50);

    // 软件看门狗：主循环断开 180ms 后必须刹车并进入 SafeWait。
    ControlFakeController watchdog_controller;
    ControlFakeChassis watchdog_chassis;
    ControlFakeImu watchdog_imu;
    ControlFakeBattery watchdog_battery;
    watchdog_controller.left_y = 50;
    control::ControlLoop watchdog_loop(watchdog_controller, watchdog_chassis, watchdog_imu,
                                       &watchdog_battery);
    watchdog_loop.update(0);
    watchdog_loop.update(20);
    watchdog_loop.update(200);
    CHECK(watchdog_loop.getMode() == control::Mode::SafeWait);
    CHECK(watchdog_chassis.left == 0);
    CHECK(watchdog_chassis.right == 0);

    // 自动赛脱困松手后必须退出 Auto，进入 SafeWait。
    ControlFakeController auto_controller;
    ControlFakeChassis auto_chassis;
    ControlFakeImu auto_imu;
    ControlFakeBattery auto_battery;
    control::ControlLoop auto_loop(auto_controller, auto_chassis, auto_imu, &auto_battery);
    auto_loop.startAuto();
    auto_loop.update(0);
    auto_controller.l1 = true;
    auto_controller.l2 = true;
    auto_loop.update(20);
    CHECK(auto_loop.getMode() == control::Mode::Auto);
    auto_controller.l1 = false;
    auto_controller.l2 = false;
    auto_loop.update(40);
    CHECK(auto_loop.getMode() == control::Mode::SafeWait);
    CHECK(auto_chassis.left == 0);
    CHECK(auto_chassis.right == 0);

    // MotionEngine 超时 Fault 必须收敛到 SafeWait。
    ControlFakeController fault_controller;
    ControlFakeChassis fault_chassis;
    ControlFakeImu fault_imu;
    ControlFakeBattery fault_battery;
    control::ControlLoop fault_loop(fault_controller, fault_chassis, fault_imu, &fault_battery);
    fault_loop.startAuto();
    fault_loop.getMotionEngine().startDrive(1.0, 50);
    fault_loop.update(20000);
    CHECK(fault_loop.getMode() == control::Mode::SafeWait);
}
