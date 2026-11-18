#include "macro/session.hpp"
#include "chassis.h"
#if 0 // 机构手动控制：暂时关闭
#include "claw.hpp"
#endif
#include "io/pros_controller.hpp"
#if 0 // 机构手动控制：暂时关闭
#include "lift.hpp"
#endif
#include "macro/engine.hpp"
#include "macro/screen.hpp"
#include "macro/storage.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <vector>

namespace macrorec {
namespace {

#if 0 // 机构手动控制：暂时关闭，确认端口/按键后启用
// ===== 机构手动控制：新增代码，按键暂为占位 =====
// TODO: 按键布局确认后只改这里。
constexpr std::uint16_t kLiftToggleButton = 1U << 10; // 占位：R1
constexpr std::uint16_t kClawOpenButton = 1U << 9;    // 占位：L2
constexpr std::uint16_t kClawCloseButton = 1U << 11;  // 占位：R2

void stop_mechanisms() {
    lift::stop();
    claw::stop();
}
// ===== 机构手动控制结束 =====
#endif

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

const char *imu_status_name(int status) {
    if (status == 0) {
        return "RDY";
    }
    if (status == 19) {
        return "CAL";
    }
    if (status == 0xFF) {
        return "ERR";
    }
    return "UNK";
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
    void apply(const Command &command) override;
    void stop() override;
    void reset() override;
    bool read_feedback(Feedback &feedback) override;
    bool read_debug_info(DebugInfo &info) const override;
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

bool Components::read_debug_info(DebugInfo &info) const {
    const auto &diag = chassis::sensor_diagnostics();
    if (!diag.has_error) {
        return false;
    }
    info = {};
    info.active = true;
    const int cal = diag.imu_calibrated ? 1 : 0;
    const char *st_name = imu_status_name(diag.imu_status);
    if (diag.calib_ms > 0) {
        std::snprintf(info.summary.data(), info.summary.size(), "%s cal=%d st=%s(%d) t=%ums",
                      diag.stage, cal, st_name, diag.imu_status,
                      static_cast<unsigned>(diag.calib_ms));
    } else if (diag.imu_errno != 0) {
        std::snprintf(info.summary.data(), info.summary.size(), "%s cal=%d st=%s(%d) e=%d(%s)",
                      diag.stage, cal, st_name, diag.imu_status, diag.imu_errno,
                      errno_name(diag.imu_errno));
    } else {
        std::snprintf(info.summary.data(), info.summary.size(), "%s cal=%d st=%s(%d) tL=%d tR=%d",
                      diag.stage, cal, st_name, diag.imu_status, diag.left_tare_errno,
                      diag.right_tare_errno);
    }

    std::array<char, 24> rot_buf{};
    format_sensor_value(diag.imu_rotation_deg, diag.imu_errno, rot_buf.data(), rot_buf.size(),
                        false);
    std::snprintf(info.imu.data(), info.imu.size(), "IMU P5: rot=%.20s st=%s(%d) rst=%s",
                  rot_buf.data(), st_name, diag.imu_status, diag.imu_reset_ret == 1 ? "OK" : "ERR");

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
#if 0 // 机构手动控制：暂时关闭
    // ===== 机构手动控制：菜单切换时停止机构 =====
    stop_mechanisms();
    // ===== 机构手动控制结束 =====
#endif
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
#if 0   // 机构手动控制：暂时关闭
        // ===== 机构手动控制：宏运行期间机构保持停止 =====
        stop_mechanisms();
        // ===== 机构手动控制结束 =====
#endif
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
            _g_components.apply(map_input(input));
#if 0       // 机构手动控制：暂时关闭
            // ===== 机构手动控制：新增代码，按键为占位 =====
            lift::update((input.buttons & kLiftToggleButton) != 0);
            if ((input.buttons & kClawOpenButton) != 0) {
                claw::open();
            } else if ((input.buttons & kClawCloseButton) != 0) {
                claw::close();
            } else {
                claw::stop();
            }
            // ===== 机构手动控制结束 =====
#endif
        } else {
            _g_components.stop();
#if 0       // 机构手动控制：暂时关闭
            // ===== 机构手动控制：手柄断开时停止机构 =====
            stop_mechanisms();
            // ===== 机构手动控制结束 =====
#endif
        }
    } else {
        _g_components.stop();
#if 0   // 机构手动控制：暂时关闭
        // ===== 机构手动控制：离开 Home/宏时停止机构 =====
        stop_mechanisms();
        // ===== 机构手动控制结束 =====
#endif
    }
    if (service_ui) {
        render();
    }
}

} // namespace

void initialize() {
    chassis::initialize();
#if 0 // 机构手动控制：暂时关闭
    // ===== 机构手动控制：初始化机构 =====
    lift::initialize();
    claw::initialize();
    // ===== 机构手动控制结束 =====
#endif
    _g_engine.cancel();
    initialize_screen();
    Feedback feedback{};
    if (!_g_components.read_feedback(feedback)) {
        message("IMU / ENCODER NOT READY: CHECK DEVICES, RESTART");
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
#if 0 // 机构手动控制：暂时关闭
    // ===== 机构手动控制：模式切换时停止机构 =====
    stop_mechanisms();
    // ===== 机构手动控制结束 =====
#endif
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
