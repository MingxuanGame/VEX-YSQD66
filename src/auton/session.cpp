#include "auton/session.hpp"
#include "main.h"
#include "auton/kit.hpp"
#include "auton/pros_hardware.hpp"
#include "chassis.h"
#include "io/pros_controller.hpp"

#include <cmath>

namespace auton {
namespace {
auton::Config _g_config = auton::defaults();
auton::Editor _g_editor;
auton::Store _g_store("/usd/auton");
const char *_g_storage_status = "NOT LOADED";
std::uint32_t _g_render_ms = 0;
bool _g_screen_owned = false;

auton::Runner &runner() {
    static auton::Runner instance(auton::pros_hardware());
    return instance;
}

auton::Buttons buttons() {
    auto &controller = io::prosController();
    auton::Buttons result{};
    result.connected = controller.isConnected();
    constexpr std::array<io::Digital, auton::kKeyCount> kMapping{
        io::Digital::Up, io::Digital::Down, io::Digital::Left, io::Digital::Right,
        io::Digital::A,  io::Digital::B,    io::Digital::X,    io::Digital::Y,
        io::Digital::L1, io::Digital::L2,   io::Digital::R1,   io::Digital::R2};
    for (int i = 0; i < auton::kKeyCount; ++i) {
        result.down[i] = controller.digital(kMapping[i]);
    }
    return result;
}

void save() {
    if (_g_store.save(_g_config)) {
        _g_editor.mark_saved();
        _g_storage_status = "SD SAVED";
    } else {
        _g_storage_status = "SD SAVE FAILED";
    }
}

void take_screen() {
    if (!_g_screen_owned) {
        pros::lcd::shutdown();
        pros::screen::set_eraser(0x000000);
        pros::screen::set_pen(0xFFFFFF);
        pros::screen::erase();
        _g_screen_owned = true;
    }
}

void restore_screen() {
    if (_g_screen_owned) {
        pros::screen::erase();
        pros::lcd::initialize();
        _g_screen_owned = false;
    }
}

void render(std::uint32_t now_ms, bool force = false) {
    if (!force && now_ms - _g_render_ms < 100) {
        return;
    }
    _g_render_ms = now_ms;
    const auto screen =
        auton::render(_g_config, _g_editor, runner(), chassis::readings(), _g_storage_status);
    for (int i = 0; i < static_cast<int>(screen.size()); ++i) {
        // TEXT_SMALL 的等宽字形和填充覆盖旧文字；不与 LLEMU/LVGL 同时拥有屏幕。
        pros::screen::print(pros::E_TEXT_SMALL, 0, static_cast<std::int16_t>(i * 14), "%-59.59s",
                            screen[i].data());
    }
}

bool tune_tick(std::uint32_t now_ms) {
    const bool was_active = _g_editor.active();
    const auto input = buttons();
    const auto event =
        _g_editor.update(input, now_ms, _g_config, runner().state() == auton::State::Running);
    if (event.stop) {
        runner().cancel();
    }
    if (event.save) {
        save();
    }
    if (event.test_step) {
        const auto &controller = io::prosController();
        const bool centered = std::abs(controller.analog(io::Analog::LeftY)) <= 5 &&
                              std::abs(controller.analog(io::Analog::RightX)) <= 5;
        if (input.connected && centered && !pros::competition::is_connected() &&
            !pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
            runner().start(_g_config.routes[_g_config.selected_route], now_ms, _g_editor.step(),
                           true);
        } else {
            _g_storage_status = "TEST BLOCKED: FIELD/STICK";
        }
    }
    const bool allowed = input.connected && !pros::competition::is_connected() &&
                         !pros::competition::is_disabled() && !pros::competition::is_autonomous();
    runner().update(now_ms, allowed);
    if (_g_editor.active()) {
        take_screen();
        render(now_ms);
        if (runner().state() != auton::State::Running) {
            auton::pros_hardware().stop();
        }
    } else if (was_active) {
        restore_screen();
    }
    return was_active || _g_editor.active();
}
} // namespace
} // namespace auton

namespace auton {
void initialize() {
    chassis::initialize();
    _g_storage_status = _g_store.load(_g_config) ? "SD LOADED" : "SD DEFAULT/ERROR";
    _g_editor.reset(buttons());
}

void stop() {
    runner().cancel();
    _g_editor.reset({});
}

void begin_opcontrol() {
    stop();
    restore_screen();
    _g_editor.reset(buttons());
}

bool settings_tick() {
    // 正常驾驶仅侦听 Konami 输入；完整序列完成前不占用屏幕或底盘。
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return true;
    }
    return tune_tick(pros::millis());
}

void run_selected() {
    const auto started_ms = pros::millis();
    stop();
    // 优先读取 SD 已保存参数；读取失败保留当前内存值，未配置的默认值会拒绝起跑。
    Config saved{};
    if (_g_store.load(saved)) {
        _g_config = saved;
        _g_editor.mark_saved();
        _g_storage_status = "SD LOADED";
    } else {
        _g_storage_status = "SD READ FAILED: USING RAM";
    }
    // Override 2.1 <GG12>/<GG13>（46-47 页）：自动期间不读取调参或试跑按键。
    if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) {
        return;
    }
    take_screen();
    runner().start(_g_config.routes[_g_config.selected_route], started_ms);
    while (runner().state() == State::Running) {
        const auto now_ms = pros::millis();
        runner().update(now_ms,
                        !pros::competition::is_disabled() && pros::competition::is_autonomous());
        render(now_ms);
        pros::delay(20);
    }
    pros_hardware().stop();
    render(pros::millis(), true);
}
} // namespace auton
