#include "macro/session.hpp"
#include "macro/engine.hpp"
#include "macro/screen.hpp"
#include "macro/storage.hpp"
#include "chassis.h"
#include "io/pros_controller.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <vector>

namespace macrorec {
namespace {
class Components final : public Hardware {
public:
    bool apply(const Command &command) override;
    void stop() override;
    bool reset() override;
    std::uint32_t time_ms(std::uint32_t fallback_ms) const override;

private:
    bool m_ready = false;
};

bool Components::apply(const Command &command) {
    // TODO：机构端口、方向与限位标定后扩展组件输出；目前只控制底盘。
    return m_ready && chassis::set_power(command.chassis_left_power, command.chassis_right_power);
}

void Components::stop() {
    chassis::stop();
    // TODO：接入机构时同时停止其输出、取消缓存目标；不在这里执行机械归零。
}

bool Components::reset() {
    m_ready = false;
    m_ready = chassis::reset_state();
    // TODO：机构接入时在这里取消全部目标、清除按键边沿/切换状态并检查重置结果。
    return m_ready;
}

std::uint32_t Components::time_ms(std::uint32_t) const {
    return pros::millis();
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
bool _g_input_cached = false;
Input _g_cached_input{};
std::uint32_t _g_service_ms = 0;
std::array<char, 96> _g_message{"20ms | 14.5s record + 0.5s reset"};
// SD 枚举缓冲区不放在 PROS 竞赛回调的小栈上；满缓冲区时拒绝保存，避免覆盖旧编号。
std::array<char, 32768> _g_listing{};

void message(const char *text) {
    std::snprintf(_g_message.data(), _g_message.size(), "%s", text);
    _g_redraw = true;
}

void result(const char *text) {
    _g_view = View::Result;
    message(text);
}

bool practice_allowed() {
    // Override 2.1 <R9>（英文第 61 页）、<GG12>/<GG13>（46-47 页）：屏幕不能绕过场控。
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
    _g_listing.fill('\x7f');
    // 空目录允许 SDK 不写入文件名；保护字节放在传给 SDK 的范围之外。
    // SDK 可以清零整个有效缓冲区，不能用有效范围末字节是否改变判断溢出。
    _g_listing.front() = '\0';
    constexpr auto kListingCapacity = _g_listing.size() - 1;
    errno = 0;
    if (pros::usd::list_files("/", _g_listing.data(), kListingCapacity) != 1) {
        const int error = errno;
        std::array<char, 96> text{};
        std::snprintf(text.data(), text.size(), "SD LIST FAILED: errno=%d", error);
        result(text.data());
        return false;
    }
    const auto *end =
        static_cast<const char *>(std::memchr(_g_listing.data(), '\0', kListingCapacity));
    if (_g_listing.back() != '\x7f' || !end || end == _g_listing.data() + kListingCapacity - 1) {
        result("SD LIST BUFFER FULL / INVALID");
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
    // refresh_catalog 已成功校验完整枚举；本任务是唯一 SD 写入者。
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
    // 所有菜单动作先取消手动输出，SD 访问期间不会保留最后一帧电机功率。
    _g_input_cached = false;
    _g_components.stop();
    switch (touch.action) {
    case TouchAction::NewRecording:
        if (!driver || !practice_allowed()) {
            result("RECORD: UNPLUG FIELD, ENABLE DRIVER");
        } else if (refresh_catalog()) {
            if (!_g_ids.empty() && _g_ids.back() == kMaxMacroId) {
                result("MACRO NUMBER LIMIT REACHED");
                break;
            }
            _g_selected_id = 0;
            if (!_g_engine.start_recording(pros::millis())) {
                result(_g_engine.status());
                break;
            }
            _g_view = View::Running;
            message("3s DELAY | L1: ADD CHECKPOINT | 14.5s CAPTURE");
        }
        break;
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
        break;
    case TouchAction::Select: {
        if (!driver || !practice_allowed()) {
            result("REPLAY: UNPLUG FIELD, ENABLE DRIVER");
            break;
        }
        _g_engine.cancel();
        if (_g_engine.state() == State::Fault) {
            result(_g_engine.status());
            break;
        }
        std::array<char, 192> path{};
        if (pros::usd::is_installed() != 1 ||
            !macro_path("/usd", touch.id, path.data(), path.size()) ||
            !load_clip(path.data(), _g_loaded)) {
            result("SD READ FAILED / INVALID MACRO");
            break;
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
        break;
    }
    case TouchAction::Resume:
        // 文件在选择宏时已完整校验；启动前再次检查场控和手动模式。
        if (driver && practice_allowed() &&
            _g_engine.start_from_checkpoint(_g_loaded, static_cast<std::size_t>(touch.id),
                                            pros::millis())) {
            _g_view = View::Running;
            message(_g_engine.recording() ? "REPLAY TO CHECKPOINT | WAIT 3s | RECORD NEW TAIL"
                                          : "FULL REPLAY: CONTROLLER INPUT IGNORED");
        } else {
            result("REPLAY START BLOCKED");
        }
        break;
    case TouchAction::Previous:
        if (_g_page > 0) {
            --_g_page;
        }
        _g_redraw = true;
        break;
    case TouchAction::Next:
        if ((_g_page + 1) * 4 < list_items().size()) {
            ++_g_page;
        }
        _g_redraw = true;
        break;
    case TouchAction::Home:
        _g_engine.cancel();
        _g_pending_save = false;
        _g_retry_save = false;
        _g_view = View::Home;
        message("20ms | 14.5s record + 0.5s reset");
        break;
    case TouchAction::Stop:
        _g_engine.cancel();
        result("CANCELLED - INCOMPLETE RECORD DISCARDED");
        break;
    case TouchAction::Retry:
        if (_g_pending_save) {
            save_pending();
        }
        break;
    case TouchAction::None:
        break;
    }
}

void tick(bool driver) {
    if (_g_pending_save && !_g_retry_save) {
        save_pending();
    }
    const auto service_now_ms = pros::millis();
    const auto state = _g_engine.state();
    const bool moving = state == State::Recording || state == State::Replaying;
    // 每 20ms 服务一次触摸/绘图，给临近帧边界的采样和电机下发留出时间。
    const bool service_ui = service_now_ms - _g_service_ms >= kPeriodMs &&
                            (!moving || _g_engine.frame_wait_ms(service_now_ms) >= 5);
    if (service_ui) {
        _g_service_ms = service_now_ms;
        handle_touch(driver);
    }
    if (_g_engine.busy()) {
        const auto previous_count = _g_engine.frame_count();
        // 后续帧提前最多 2ms 采样一次，避免 16 个手柄读取挤占输出边界。
        // 倒计时/检查点暂停仍须结束后才读取首帧，之后以实际下发完成时刻建立时基。
        const auto now_ms = pros::millis();
        if (_g_engine.needs_input(now_ms) && !_g_input_cached) {
            _g_cached_input = read_input();
            _g_input_cached = true;
        }
        _g_engine.update(pros::millis(), driver && practice_allowed(),
                         _g_input_cached ? _g_cached_input : Input{});
        if (_g_engine.frame_count() != previous_count || !_g_engine.busy()) {
            _g_input_cached = false;
        }
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
        if (!input.connected || !valid_frame({input, {}}) ||
            !_g_components.apply(map_input(input))) {
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
    if (_g_engine.state() == State::Fault) {
        result(_g_engine.status());
    }
    render();
}

void stop() {
    _g_input_cached = false;
    const bool was_busy = _g_engine.busy();
    // 已采齐的录制在模式退出时只需停止；保留 RAM 供下一空闲回调保存。
    const auto state = _g_engine.state();
    if (_g_engine.recording() && _g_engine.frame_count() == kFrameCount &&
        (state == State::Resetting || state == State::Complete)) {
        _g_pending_save = true;
    }
    _g_engine.cancel();
    if (_g_engine.state() == State::Fault) {
        result(_g_engine.status());
    } else if (was_busy) {
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
