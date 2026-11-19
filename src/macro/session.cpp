#include "macro/session.hpp"
#include "calibration/session.hpp"
#include "chassis.h"
#include "io/pros_controller.hpp"
#include "lifting.hpp"
#include "macro/engine.hpp"
#include "macro/screen.hpp"
#include "macro/storage.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <vector>

namespace macrorec {
namespace {

constexpr std::uint16_t kLiftButtons = kLiftUpButton | kLiftDownButton;
constexpr std::uint32_t kLiftJogTimeoutMs = 500;

const char *errno_name(int err) {
    switch (err) {
    case 0:
        return "OK";
    case ENXIO:
        return "ENXIO";
    case ENODEV:
        return "ENODEV";
    case EAGAIN:
        return "EAGAIN";
    case EACCES:
        return "EACCES";
    case EDOM:
        return "EDOM";
    case EOVERFLOW:
        return "EOVERFLOW";
    default:
        return "ERR";
    }
}

void format_sensor_value(double value, int err, char *buf, std::size_t size, bool compact) {
    if (std::isfinite(value)) {
        std::snprintf(buf, size, compact ? "%.0f" : "%.1fdeg", value);
        return;
    }
    const char *kind = std::isnan(value) ? "NAN" : "ERR";
    if (err != 0) {
        if (compact) {
            std::snprintf(buf, size, "E%d", err);
        } else {
            std::snprintf(buf, size, "%s(%d:%s)", kind, err, errno_name(err));
        }
    } else {
        std::snprintf(buf, size, "%s", kind);
    }
}

class Components final : public Hardware {
public:
    Command apply(Command command) override;
    void stop() override;
    void reset() override;
    bool read_feedback(Feedback &feedback) override;
    bool read_debug_info(DebugInfo &info) const override;
};

Command Components::apply(Command command) {
    chassis::set_power(command.chassis_left_power, command.chassis_right_power);
    command.lift_power = lifting::set_power(command.lift_power);
    return command;
}

void Components::stop() {
    chassis::stop();
    lifting::stop();
}

void Components::reset() {
    lifting::stop();
    chassis::reset_state();
}

bool Components::read_feedback(Feedback &feedback) {
    return chassis::read_feedback(feedback.motor_position_deg, feedback.imu_rotation_deg);
}

bool Components::read_debug_info(DebugInfo &info) const {
    const auto &diag = chassis::sensor_diagnostics();
    if (!diag.has_error) {
        return false;
    }
    info = {};
    info.active = true;
    const int cal = diag.imu_calibrated ? 1 : 0;
    if (diag.calib_ms > 0 && diag.imu_errno != 0) {
        std::snprintf(info.summary.data(), info.summary.size(), "%s e=%d(%s) %ums", diag.stage,
                      diag.imu_errno, errno_name(diag.imu_errno),
                      static_cast<unsigned>(diag.calib_ms));
    } else if (diag.calib_ms > 0) {
        std::snprintf(info.summary.data(), info.summary.size(), "%s %ums", diag.stage,
                      static_cast<unsigned>(diag.calib_ms));
    } else if (diag.imu_errno != 0) {
        std::snprintf(info.summary.data(), info.summary.size(), "%s e=%d(%s)", diag.stage,
                      diag.imu_errno, errno_name(diag.imu_errno));
    } else {
        std::snprintf(info.summary.data(), info.summary.size(), "%s tL=%d tR=%d", diag.stage,
                      diag.left_tare_errno, diag.right_tare_errno);
    }

    std::array<char, 24> rot_buf{};
    format_sensor_value(diag.imu_rotation_deg, diag.imu_errno, rot_buf.data(), rot_buf.size(),
                        false);
    const char *reset_label = diag.imu_reset_ret == 0   ? "SKIP"
                              : diag.imu_reset_ret == 1 ? "OK"
                                                        : "ERR";
    std::snprintf(info.imu.data(), info.imu.size(), "P5 %s(%d) rst=%s cal=%d rot=%.20s",
                  diag.imu_status_label, diag.imu_status, reset_label, cal, rot_buf.data());

    std::array<std::array<char, 24>, 4> enc_buf{};
    std::array<std::array<char, 10>, 4> enc_short{};
    for (std::size_t i = 0; i < 4; ++i) {
        format_sensor_value(diag.motor_position_deg[i], diag.motor_errno[i], enc_buf[i].data(),
                            enc_buf[i].size(), false);
        format_sensor_value(diag.motor_position_deg[i], diag.motor_errno[i], enc_short[i].data(),
                            enc_short[i].size(), true);
    }
    std::snprintf(info.encoders_l.data(), info.encoders_l.size(), "ENC L: P2=%.20s P1=%.20s",
                  enc_buf[0].data(), enc_buf[1].data());
    std::snprintf(info.encoders_r.data(), info.encoders_r.size(), "ENC R: P3=%.20s P4=%.20s",
                  enc_buf[2].data(), enc_buf[3].data());

    std::array<char, 10> rot_short{};
    format_sensor_value(diag.imu_rotation_deg, diag.imu_errno, rot_short.data(), rot_short.size(),
                        true);
    std::snprintf(info.encoders_all.data(), info.encoders_all.size(),
                  "P5:%.8s P2:%.8s P1:%.8s P3:%.8s P4:%.8s", rot_short.data(), enc_short[0].data(),
                  enc_short[1].data(), enc_short[2].data(), enc_short[3].data());
    return true;
}

Components _g_components;
Engine _g_engine(_g_components);
calibration::Session _g_calibration(_g_components);
Clip _g_loaded{};
DebugInfo _g_debug_info{};
View _g_view = View::Home;
std::vector<int> _g_ids;
std::vector<int> _g_visible_ids;
std::vector<int> _g_checkpoints;
std::size_t _g_page = 0;
int _g_selected_id = 0;
bool _g_pending_save = false;
bool _g_retry_save = false;
bool _g_redraw = true;
bool _g_manual_lift_armed = false;
bool _g_lift_jog_armed = false;
std::optional<std::uint32_t> _g_lift_jog_started_ms;
std::optional<lifting::Position> _g_lift_lower_deg;
const char *_g_lift_status = nullptr;
std::uint32_t _g_service_ms = 0;
std::array<char, 96> _g_message{"20ms | 14.5s record + 0.5s reset"};
std::array<char, 16384> _g_listing{};

void message(const char *text) {
    _g_debug_info = {};
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
                _g_message.data(), _g_debug_info, _g_retry_save, _g_redraw);
    _g_redraw = false;
}

void tick_lift_limits(std::uint32_t now_ms, bool driver, TouchAction action) {
    const bool allowed = driver && practice_allowed();
    const Input input = allowed ? read_input() : Input{};
    const auto position = lifting::read_position();
    const bool ready = allowed && input.connected && position.has_value();
    const auto buttons = input.buttons & kLiftButtons;
    const bool capture = action == TouchAction::MarkLiftLower ||
                         action == TouchAction::MarkLiftUpper ||
                         action == TouchAction::UseSavedLiftLimits;

    if (!position) {
        _g_lift_lower_deg.reset();
    }
    if (!ready || buttons == kLiftButtons || capture) {
        _g_lift_jog_armed = false;
    } else if (buttons == 0) {
        _g_lift_jog_armed = true;
        _g_lift_jog_started_ms.reset();
    }
    if (_g_lift_jog_armed && buttons != 0) {
        if (!_g_lift_jog_started_ms) {
            _g_lift_jog_started_ms = now_ms;
        }
        if (now_ms - *_g_lift_jog_started_ms >= kLiftJogTimeoutMs) {
            _g_lift_jog_armed = false;
            _g_lift_status = "JOG TIMEOUT - RELEASE X/B";
        } else {
            _g_lift_status = nullptr;
            lifting::jog(buttons == kLiftUpButton ? 1 : -1);
        }
    }
    if (!_g_lift_jog_armed || buttons == 0) {
        lifting::stop();
    }

    if (capture && ready) {
        if (buttons != 0 || _g_lift_jog_started_ms) {
            _g_lift_status = "RELEASE X/B AND LET LIFT STOP";
        } else if (action == TouchAction::UseSavedLiftLimits) {
            if (lifting::use_saved_limits(*position)) {
                _g_lift_lower_deg.reset();
                _g_lift_status = "SAVED LIMITS ACTIVE";
            } else {
                _g_lift_status = "CANNOT RESTORE LIMITS. CHECK SD AND MOTORS.";
            }
        } else if (action == TouchAction::MarkLiftLower) {
            _g_lift_lower_deg = position;
            _g_lift_status = "LOWER CAPTURED. JOG UP, THEN SET UPPER.";
        } else if (!_g_lift_lower_deg) {
            _g_lift_status = "SET LOWER FIRST";
        } else if (lifting::set_limits({*_g_lift_lower_deg, *position})) {
            _g_lift_lower_deg.reset();
            _g_lift_status = "LIMITS SAVED TO SD AND ACTIVE";
        } else {
            _g_lift_status = "CHECK UPWARD ENCODER DIRECTION AND SD CARD";
        }
    }
    const char *status = _g_lift_status;
    if (!position) {
        status = "CHECK MOTORS ON P9 AND P10";
    } else if (!allowed) {
        status = "UNPLUG FIELD AND ENABLE DRIVER TO SET LIMITS";
    } else if (!input.connected) {
        status = "CONNECT CONTROLLER";
    } else if (!status) {
        status = _g_lift_lower_deg             ? "JOG UP, THEN SET UPPER"
                 : lifting::limits()           ? "LIMITS ACTIVE"
                 : lifting::has_saved_limits() ? "AT ORIGINAL LOWER END, TAP USE SAVED"
                                               : "JOG DOWN, THEN SET LOWER";
    }
    draw_lift_limits(now_ms, position, _g_lift_lower_deg, status, _g_redraw || capture);
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

void handle_touch(bool driver, Touch touch) {
    if (touch.action == TouchAction::None) {
        return;
    }
    // 切换菜单或访问 SD 前立即停止电机输出。
    _g_components.stop();
    _g_manual_lift_armed = false;
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
    case TouchAction::OpenCalibration:
        _g_engine.cancel();
        _g_calibration.open();
        _g_view = View::Calibration;
        return;
    case TouchAction::OpenLiftLimits:
        _g_view = View::LiftLimits;
        _g_lift_lower_deg.reset();
        _g_lift_status = nullptr;
        _g_lift_jog_armed = false;
        _g_lift_jog_started_ms.reset();
        _g_redraw = true;
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
        message("MATCH ORIGINAL POSE + LIFT HEIGHT | SELECT CHECKPOINT");
        return;
    }
    case TouchAction::Resume: {
        const auto frame_count = static_cast<std::size_t>(touch.id);
        if (driver && practice_allowed() && !lifting::limits() &&
            std::any_of(_g_loaded.begin(), _g_loaded.begin() + frame_count,
                        [](const Frame &frame) { return frame.command.lift_power != 0; })) {
            result("SET LIFT LIMITS BEFORE REPLAY");
            return;
        }
        if (driver && practice_allowed() &&
            _g_engine.start_from_checkpoint(_g_loaded, frame_count, pros::millis())) {
            _g_view = View::Running;
            message(_g_engine.recording() ? "REPLAY TO CHECKPOINT | WAIT 3s | RECORD NEW TAIL"
                                          : "FULL REPLAY: CONTROLLER INPUT IGNORED");
        } else {
            result("REPLAY START BLOCKED");
        }
        return;
    }
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
    case TouchAction::MarkLiftLower:
    case TouchAction::MarkLiftUpper:
    case TouchAction::UseSavedLiftLimits:
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
        if (_g_view != View::Calibration) {
            const Touch touch = poll_touch(_g_view, list_items(), _g_page, _g_retry_save);
            if (_g_view == View::LiftLimits && touch.action != TouchAction::Home) {
                tick_lift_limits(now_ms, driver, touch.action);
                return;
            }
            handle_touch(driver, touch);
        }
    }
    if (_g_view == View::LiftLimits) {
        return;
    }
    if (_g_view == View::Calibration) {
        if (service_ui) {
            const bool allowed = driver && practice_allowed();
            auto &controller = io::prosController();
            const bool connected = allowed && controller.isConnected();
            _g_calibration.tick(now_ms, allowed, connected,
                                connected && controller.digital(io::Digital::A));
            if (!_g_calibration.active()) {
                _g_view = View::Home;
                _g_redraw = true;
                reset_touch();
                render();
            }
        }
        return;
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
            if (_g_engine.debug_info().active) {
                _g_debug_info = _g_engine.debug_info();
            }
        }
    } else if (driver && _g_view == View::Home && !pros::competition::is_disabled() &&
               !pros::competition::is_autonomous()) {
        const auto input = read_input();
        if (input.connected) {
            _g_manual_lift_armed |= (input.buttons & kLiftButtons) == 0;
            auto command = map_input(input);
            if (!_g_manual_lift_armed) {
                command.lift_power = 0;
            }
            _g_components.apply(command);
        } else {
            _g_components.stop();
            _g_manual_lift_armed = false;
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
    lifting::initialize();
    _g_engine.cancel();
    initialize_screen();
    Feedback feedback{};
    if (!_g_components.read_feedback(feedback)) {
        message("IMU / ENCODER NOT READY: CHECK DEVICES");
        _g_components.read_debug_info(_g_debug_info);
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
    _g_calibration.interrupt();
    _g_manual_lift_armed = false;
    _g_lift_jog_armed = false;
    _g_lift_jog_started_ms.reset();
    _g_lift_lower_deg.reset();
    _g_lift_status = nullptr;
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
