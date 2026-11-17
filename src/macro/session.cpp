#include "macro/session.hpp"
#include "chassis.h"
#include "io/pros_controller.hpp"
#include "macro/engine.hpp"
#include "macro/screen.hpp"
#include "macro/storage.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <cerrno>
#include <cstdio>
#include <vector>

namespace macrorec {
namespace {

class Components final : public Hardware {
public:
    void apply(const Command &command) override;
    void stop() override;
    void reset() override;
    bool read_feedback(Feedback &feedback) override;
};

void Components::apply(const Command &command) {
    chassis::set_power(command.chassis_left_power, command.chassis_right_power);
}

void Components::stop() {
    chassis::stop();
}

void Components::reset() {
    chassis::reset_state();
}

bool Components::read_feedback(Feedback &feedback) {
    return chassis::read_feedback(feedback.motor_position_deg, feedback.imu_rotation_deg);
}

Components _g_components;
Engine _g_engine(_g_components);
Clip _g_loaded{};
View _g_view = View::Home;
std::vector<int> _g_ids;
std::vector<int> _g_visible_ids;
std::vector<int> _g_checkpoints;
std::size_t _g_page = 0;
int _g_selected_id = 0;
bool _g_pending_save = false;
bool _g_retry_save = false;
bool _g_redraw = true;
std::uint32_t _g_service_ms = 0;
std::array<char, 96> _g_message{"20ms | 14.5s record + 0.5s reset"};
std::array<char, 16384> _g_listing{};

void message(const char *text) {
    std::snprintf(_g_message.data(), _g_message.size(), "%s", text);
    _g_redraw = true;
}

void result(const char *text) {
    _g_view = View::Result;
    message(text);
}

bool practice_allowed() {
    // Override 2.1 <R9>、<GG12>/<GG13>：接入场控或非手动启用阶段禁止录放。
    return !pros::competition::is_connected() && !pros::competition::is_disabled() &&
           !pros::competition::is_autonomous();
}

Input read_input() {
    auto &controller = io::prosController();
    Input input{};
    input.connected = controller.isConnected();
    if (!input.connected) {
        return input;
    }
    input.left_x = controller.analog(io::Analog::LeftX);
    input.left_y = controller.analog(io::Analog::LeftY);
    input.right_x = controller.analog(io::Analog::RightX);
    input.right_y = controller.analog(io::Analog::RightY);
    constexpr std::array<io::Digital, 12> kButtons{
        io::Digital::Up, io::Digital::Down, io::Digital::Left, io::Digital::Right,
        io::Digital::A,  io::Digital::B,    io::Digital::X,    io::Digital::Y,
        io::Digital::L1, io::Digital::L2,   io::Digital::R1,   io::Digital::R2};
    for (std::size_t i = 0; i < kButtons.size(); ++i) {
        if (controller.digital(kButtons[i])) {
            input.buttons |= static_cast<std::uint16_t>(1U << i);
        }
    }
    return input;
}

bool refresh_catalog() {
    _g_ids.clear();
    if (pros::usd::is_installed() != 1) {
        result("NO SD CARD");
        return false;
    }
    _g_listing.fill('\0');
    errno = 0;
    if (pros::usd::list_files("/", _g_listing.data(), _g_listing.size() - 1) != 1) {
        std::array<char, 96> text{};
        std::snprintf(text.data(), text.size(), "SD LIST FAILED: errno=%d", errno);
        result(text.data());
        return false;
    }
    _g_ids = catalog(_g_listing.data());
    return true;
}

const std::vector<int> &list_items() {
    return _g_view == View::Checkpoints ? _g_checkpoints : _g_visible_ids;
}

void render() {
    draw_screen(_g_view, list_items(), _g_page, _g_engine, pros::millis(), _g_selected_id,
                _g_message.data(), _g_retry_save, _g_redraw);
    _g_redraw = false;
}

void save_pending() {
    _g_components.stop();
    _g_retry_save = true;
    if (!refresh_catalog()) {
        return;
    }
    const int next = _g_ids.empty() ? 1 : _g_ids.back() + 1;
    if (next > kMaxMacroId) {
        result("MACRO NUMBER LIMIT REACHED");
        return;
    }
    result("SAVING TO SD - KEEP POWER ON");
    render();
    SaveError error{};
    if (!save_new_clip("/usd", next, _g_engine.clip(), &error, _g_listing.data())) {
        std::array<char, 96> text{};
        std::snprintf(text.data(), text.size(), "SD SAVE FAILED: %s errno=%d", error.stage,
                      error.error_number);
        result(text.data());
        return;
    }
    _g_selected_id = next;
    _g_engine.cancel();
    _g_pending_save = false;
    _g_retry_save = false;
    result("SD SAVED AND VERIFIED");
}

void handle_touch(bool driver) {
    const auto touch = poll_touch(_g_view, list_items(), _g_page, _g_retry_save);
    if (touch.action == TouchAction::None) {
        return;
    }
    // 切换菜单或访问 SD 前立即停止电机输出。
    _g_components.stop();
    switch (touch.action) {
    case TouchAction::NewRecording:
        if (!driver || !practice_allowed()) {
            result("RECORD: UNPLUG FIELD, ENABLE DRIVER");
        } else if (refresh_catalog()) {
            if (!_g_ids.empty() && _g_ids.back() == kMaxMacroId) {
                result("MACRO NUMBER LIMIT REACHED");
                return;
            }
            _g_selected_id = 0;
            _g_engine.start_recording(pros::millis());
            _g_view = View::Running;
            message("3s DELAY | L1: ADD CHECKPOINT | 14.5s CAPTURE");
        }
        return;
    case TouchAction::OpenList:
        _g_visible_ids.clear();
        if (refresh_catalog()) {
            std::array<char, 192> path{};
            for (const int id : _g_ids) {
                if (macro_path("/usd", id, path.data(), path.size()) &&
                    compatible_clip(path.data())) {
                    _g_visible_ids.push_back(id);
                }
            }
            _g_page = 0;
            _g_view = View::List;
            message(_g_visible_ids.empty() ? "NO CURRENT MACROS ON SD"
                                           : "SELECT MACRO, THEN CHECKPOINT OR END");
        }
        return;
    case TouchAction::Select: {
        if (!driver || !practice_allowed()) {
            result("REPLAY: UNPLUG FIELD, ENABLE DRIVER");
            return;
        }
        _g_engine.cancel();
        std::array<char, 192> path{};
        if (pros::usd::is_installed() != 1 ||
            !macro_path("/usd", touch.id, path.data(), path.size()) ||
            !load_clip(path.data(), _g_loaded)) {
            result("SD READ FAILED / INVALID MACRO");
            return;
        }
        _g_selected_id = touch.id;
        _g_checkpoints.clear();
        for (std::size_t i = 0; i + 1 < kFrameCount; ++i) {
            if (_g_loaded[i].checkpoint) {
                _g_checkpoints.push_back(static_cast<int>(i + 1));
            }
        }
        _g_checkpoints.push_back(static_cast<int>(kFrameCount));
        _g_page = 0;
        _g_view = View::Checkpoints;
        message("START AT ORIGINAL POSE | SELECT CHECKPOINT");
        return;
    }
    case TouchAction::Resume:
        if (driver && practice_allowed() &&
            _g_engine.start_from_checkpoint(_g_loaded, static_cast<std::size_t>(touch.id),
                                            pros::millis())) {
            _g_view = View::Running;
            message(_g_engine.recording() ? "REPLAY TO CHECKPOINT | WAIT 3s | RECORD NEW TAIL"
                                          : "FULL REPLAY: CONTROLLER INPUT IGNORED");
        } else {
            result("REPLAY START BLOCKED");
        }
        return;
    case TouchAction::Previous:
        if (_g_page > 0) {
            --_g_page;
        }
        _g_redraw = true;
        return;
    case TouchAction::Next:
        if ((_g_page + 1) * 4 < list_items().size()) {
            ++_g_page;
        }
        _g_redraw = true;
        return;
    case TouchAction::Home:
        _g_engine.cancel();
        _g_pending_save = false;
        _g_retry_save = false;
        _g_view = View::Home;
        message("20ms | 14.5s record + 0.5s reset");
        return;
    case TouchAction::Stop:
        _g_engine.cancel();
        result("CANCELLED - INCOMPLETE RECORD DISCARDED");
        return;
    case TouchAction::Retry:
        if (_g_pending_save) {
            save_pending();
        }
        return;
    case TouchAction::None:
        return;
    }
}

void tick(bool driver) {
    if (_g_pending_save && !_g_retry_save) {
        save_pending();
    }
    const auto now_ms = pros::millis();
    const bool service_ui = now_ms - _g_service_ms >= kPeriodMs;
    if (service_ui) {
        _g_service_ms = now_ms;
        handle_touch(driver);
    }
    if (_g_engine.busy()) {
        const Input input = _g_engine.recording() ? read_input() : Input{};
        _g_engine.update(pros::millis(), driver && practice_allowed(), input);
        if (_g_engine.state() == State::Complete) {
            if (_g_engine.recording()) {
                _g_pending_save = true;
                save_pending();
            } else {
                result("REPLAY COMPLETE - OUTPUT ZERO");
            }
        } else if (_g_engine.state() == State::Fault) {
            result(_g_engine.status());
        }
    } else if (driver && _g_view == View::Home && !pros::competition::is_disabled() &&
               !pros::competition::is_autonomous()) {
        const auto input = read_input();
        if (input.connected) {
            _g_components.apply(map_input(input));
        } else {
            _g_components.stop();
        }
    } else {
        _g_components.stop();
    }
    if (service_ui) {
        render();
    }
}

} // namespace

void initialize() {
    chassis::initialize();
    _g_engine.cancel();
    initialize_screen();
    Feedback feedback{};
    if (!_g_components.read_feedback(feedback)) {
        message("IMU / ENCODER NOT READY: CHECK DEVICES, RESTART");
    }
    render();
}

void stop() {
    const bool was_busy = _g_engine.busy();
    const auto state = _g_engine.state();
    // 已采齐 725 帧的录制在退出时保留 RAM，由后续空闲周期存盘。
    if (_g_engine.recording() && _g_engine.frame_count() == kFrameCount &&
        (state == State::Resetting || state == State::Complete)) {
        _g_pending_save = true;
    }
    _g_engine.cancel();
    if (was_busy) {
        result("MODE CHANGED - OUTPUT ZERO");
    }
    reset_touch();
}

void idle_tick() {
    tick(false);
}

void begin_driver() {
    stop();
}

void driver_tick() {
    tick(true);
}

std::uint32_t driver_period_ms() {
    return _g_engine.busy() ? 1 : kPeriodMs;
}

} // namespace macrorec
