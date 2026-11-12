#include "auton/kit.hpp"
#include "test_common.hpp"

#include <cmath>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <limits>
#include <vector>

namespace {
class AutoHardware final : public auton::Hardware {
public:
    bool online = true;
    bool sensor_valid = true;
    bool pressed = false;
    bool accept = true;
    double angle_deg = 0;
    int left_mv = 0;
    int right_mv = 0;
    int lift_mv = 0;
    std::vector<auton::Action> actions;

    bool ready() const override {
        return online;
    }

    bool heading(double &angle) const override {
        angle = angle_deg;
        return sensor_valid;
    }

    bool contact(bool &value) const override {
        value = pressed;
        return sensor_valid;
    }

    bool drive_voltage(int left, int right) override {
        left_mv = left;
        right_mv = right;
        return accept;
    }

    bool lift_voltage(int value) override {
        lift_mv = value;
        return accept;
    }

    bool action(auton::Action action) override {
        actions.push_back(action);
        return accept;
    }

    void stop() override {
        left_mv = 0;
        right_mv = 0;
        lift_mv = 0;
    }
};

auton::Route configured_route() {
    auto route = auton::defaults().routes[0];
    for (auto &step : route.steps) {
        step.values = {30, 40, 400, 160, 60, 40, 2000, 2000};
    }
    return route;
}

void drive_to_end(auton::Runner &runner, AutoHardware &hardware, std::uint32_t &now_ms,
                  bool make_contact = true) {
    int contact_ticks = 0;
    for (int tick = 0; tick < 1200 && runner.state() == auton::State::Running; ++tick) {
        hardware.angle_deg += (hardware.left_mv - hardware.right_mv) * 0.0001;
        if (runner.phase() == auton::Phase::Contact) {
            ++contact_ticks;
        } else {
            contact_ticks = 0;
        }
        hardware.pressed = make_contact && contact_ticks >= 3;
        now_ms += 20;
        runner.update(now_ms, true);
    }
}

void test_editor() {
    auton::Config config = auton::defaults();
    auton::Editor editor;
    std::uint32_t now_ms = 0;
    const auto press = [&](auton::Key key) {
        auton::Buttons buttons;
        now_ms += 20;
        editor.update(buttons, now_ms, config);
        buttons.down[static_cast<int>(key)] = true;
        now_ms += 20;
        return editor.update(buttons, now_ms, config);
    };
    // 不完整、错误或长按都不能进入设置。
    press(auton::Key::Up);
    press(auton::Key::B);
    CHECK(!editor.active());
    auton::Buttons held;
    held.down[static_cast<int>(auton::Key::Up)] = true;
    for (int i = 0; i < 10; ++i) {
        editor.update(held, now_ms += 20, config);
    }
    CHECK(!editor.active());
    constexpr std::array<auton::Key, 10> kCode{
        auton::Key::Up,    auton::Key::Up,   auton::Key::Down,  auton::Key::Down, auton::Key::Left,
        auton::Key::Right, auton::Key::Left, auton::Key::Right, auton::Key::B,    auton::Key::A};
    for (auto key : kCode) {
        press(key);
    }
    CHECK(editor.active());
    CHECK(config.routes[0].steps[0].values[0] == 0); // 序列末尾 A 不能减一次参数。
    press(auton::Key::X);
    CHECK(config.routes[0].steps[0].values[0] == 1);
    press(auton::Key::Y);
    press(auton::Key::X);
    CHECK(config.routes[0].steps[0].values[0] == 11);
    CHECK(editor.digit() == 1);
    press(auton::Key::A);
    CHECK(editor.digit() == 0);
    press(auton::Key::B);
    CHECK(config.routes[0].steps[0].values[0] == 10);
    press(auton::Key::X);
    press(auton::Key::Y);
    press(auton::Key::L1);
    CHECK(config.selected_route == 1);
    CHECK(config.routes[1].steps[0].values[0] == 0);
    press(auton::Key::X);
    CHECK(config.routes[1].steps[0].values[0] == 10);
    CHECK(config.routes[0].steps[0].values[0] == 11);
    press(auton::Key::Right);
    press(auton::Key::Down);
    CHECK(editor.step() == 1 && editor.param() == auton::Param::Back);
    CHECK(press(auton::Key::R1).save);
    CHECK(editor.dirty());
    editor.mark_saved();
    CHECK(!editor.dirty());
    CHECK(press(auton::Key::R2).stop);
    CHECK(!editor.active());
    editor.reset(held);
    editor.update(held, now_ms += 20, config);
    CHECK(!editor.active());
}

void test_storage() {
    // 每次测试使用单独临时目录，不覆盖用户 SD 或项目内配置。
    const auto folder =
        std::filesystem::temp_directory_path() /
        ("vex-auton-test-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(folder);
    const auto base = (folder / "params").string();
    auton::Store store(base.c_str());
    auto first = auton::defaults();
    auto loaded = first;
    CHECK(!store.load(loaded));
    first.routes[0].steps[0].values[0] = 42;
    CHECK(store.save(first));
    auto second = first;
    second.selected_route = 1;
    second.routes[1].steps[4].values[0] = 75;
    CHECK(store.save(second));
    CHECK(store.load(loaded));
    CHECK(loaded.selected_route == 1 && loaded.routes[1].steps[4].values[0] == 75);
    // 模拟最新槽断电写入不完整，应恢复前一个有效槽。
    auto *file = std::fopen((base + ".1").c_str(), "wb");
    CHECK(file != nullptr);
    if (file) {
        std::fputs("truncated", file);
        std::fclose(file);
    }
    CHECK(store.load(loaded));
    CHECK(loaded.selected_route == 0 && loaded.routes[0].steps[0].values[0] == 42);
    second.routes[0].steps[0].values[0] = 361;
    CHECK(!store.save(second));
    CHECK(store.load(loaded));
    CHECK(loaded.routes[0].steps[0].values[0] == 42);
    file = std::fopen((base + ".0").c_str(), "r+b");
    CHECK(file != nullptr);
    if (file) {
        std::fputc(0, file); // 完整长度但校验不正确。
        std::fclose(file);
    }
    CHECK(!store.load(loaded));
    CHECK(loaded.routes[0].steps[0].values[0] == 42);
    std::filesystem::remove(base + ".0");
    std::filesystem::remove(base + ".1");
    std::filesystem::remove(folder);
}

void test_runner() {
    AutoHardware hardware;
    auton::Runner runner(hardware);
    auto route = configured_route();
    CHECK(!runner.start(auton::defaults().routes[0], 0));
    CHECK(hardware.left_mv == 0 && hardware.actions.empty());
    hardware.online = false;
    CHECK(!runner.start(route, 0));
    hardware.online = true;
    std::uint32_t now_ms = 0;
    CHECK(runner.start(route, now_ms));
    drive_to_end(runner, hardware, now_ms);
    CHECK(runner.state() == auton::State::Done);
    CHECK(hardware.left_mv == 0 && hardware.lift_mv == 0);

    // 第六步前进先结束、下降后结束；不能提前停止下降或继续前进。
    route = configured_route();
    route.steps[5].values = {0, 0, 40, 0, 120, 0, 0, 2000};
    CHECK(runner.start(route, now_ms, 5, true));
    while (runner.phase() == auton::Phase::Turn && runner.state() == auton::State::Running) {
        runner.update(now_ms += 20, true);
    }
    CHECK(runner.phase() == auton::Phase::Forward);
    runner.update(now_ms += 20, true);
    runner.update(now_ms += 20, true);
    CHECK(hardware.left_mv == 0 && hardware.lift_mv == -12000);
    CHECK(runner.state() == auton::State::Running);
    drive_to_end(runner, hardware, now_ms);
    CHECK(runner.state() == auton::State::Done && hardware.lift_mv == 0);

    // 模拟下一周期到达机构限位，适配器拒绝输出后必须立即全停。
    CHECK(runner.start(route, now_ms, 2, true));
    hardware.accept = false;
    runner.update(now_ms += 20, true);
    CHECK(runner.state() == auton::State::Fault);
    CHECK(hardware.left_mv == 0 && hardware.lift_mv == 0);
    hardware.accept = true;

    auton::Editor editor;
    auto config = auton::defaults();
    std::array<auton::MotorReading, 4> readings{};
    readings[0] = {3, true, 7345, 51, 123, 234};
    const auto view = auton::render(config, editor, runner, readings, "SD LOADED");
    CHECK(view[2][std::strlen(auton::param_name(editor.param())) + 7] == '^');
    CHECK(std::strstr(view[12].data(), "+7345mV") != nullptr);
    CHECK(std::strstr(view[13].data(), "ERR") != nullptr);
    const std::vector<auton::Action> expected{auton::Action::Toggle, auton::Action::Open,
                                              auton::Action::Close,  auton::Action::Open,
                                              auton::Action::Close,  auton::Action::Open};
    CHECK(hardware.actions == expected);

    // 提前到桩时立即停止正在进行的抬升并下降，不能原地补完 lift_ms。
    hardware.pressed = false;
    CHECK(runner.start(route, now_ms, 4, true));
    while (runner.phase() == auton::Phase::Turn && runner.state() == auton::State::Running) {
        hardware.angle_deg += (hardware.left_mv - hardware.right_mv) * 0.0001;
        runner.update(now_ms += 20, true);
    }
    CHECK(hardware.lift_mv == 12000);
    hardware.pressed = true;
    runner.update(now_ms += 20, true);
    CHECK(runner.phase() == auton::Phase::Lower);
    CHECK(hardware.left_mv == 0 && hardware.lift_mv == -12000);
    runner.cancel();

    // 第三步抬升与转弯并行，禁用同一周期清零所有输出。
    hardware.pressed = false;
    CHECK(runner.start(route, now_ms, 2, true));
    runner.update(now_ms += 20, true);
    CHECK(hardware.left_mv != hardware.right_mv && hardware.lift_mv == 12000);
    runner.update(now_ms += 20, false);
    CHECK(runner.state() == auton::State::Cancelled);
    CHECK(hardware.left_mv == 0 && hardware.right_mv == 0 && hardware.lift_mv == 0);

    // 单步结束，不执行下一步；零后退时间不能产生一个周期的满电压反冲。
    route.steps[1].values[static_cast<int>(auton::Param::Back)] = 0;
    const auto old_actions = hardware.actions.size();
    CHECK(runner.start(route, now_ms, 1, true));
    CHECK(hardware.left_mv == 0);
    drive_to_end(runner, hardware, now_ms);
    CHECK(runner.state() == auton::State::Done && runner.step() == 1);
    CHECK(hardware.actions.size() == old_actions + 1);

    // 缺失撞桩信号不能作为成功继续放物。
    hardware.pressed = false;
    hardware.actions.clear();
    CHECK(runner.start(route, now_ms, 4, true));
    drive_to_end(runner, hardware, now_ms, false);
    CHECK(runner.state() == auton::State::Fault);
    CHECK(std::strcmp(runner.status(), "CONTACT TIMEOUT") == 0);
    CHECK(hardware.actions.empty());
    CHECK(hardware.left_mv == 0 && hardware.lift_mv == 0);

    CHECK(runner.start(route, now_ms, 0, true));
    hardware.angle_deg = std::numeric_limits<double>::quiet_NaN();
    runner.update(now_ms += 20, true);
    CHECK(runner.state() == auton::State::Fault && hardware.left_mv == 0);
    hardware.angle_deg = 0;
    CHECK(runner.start(route, now_ms, 0, true));
    runner.update(now_ms += 120, true);
    CHECK(std::strcmp(runner.status(), "LOOP WATCHDOG") == 0);

    // 全程限时独立于各段 timeout：前两段长后退使总时间先到 14.8s。
    route.steps[1].values[static_cast<int>(auton::Param::Back)] = 10000;
    route.steps[3].values[static_cast<int>(auton::Param::Back)] = 10000;
    CHECK(runner.start(route, now_ms));
    drive_to_end(runner, hardware, now_ms);
    CHECK(std::strcmp(runner.status(), "AUTO TIME LIMIT") == 0);
    CHECK(hardware.left_mv == 0 && hardware.lift_mv == 0);
}
} // namespace

void test_auton() {
    test_editor();
    test_storage();
    test_runner();
}
