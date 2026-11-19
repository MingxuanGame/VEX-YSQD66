#include "brain_ui.hpp"

#include "io/pros_controller.hpp"
#include "lemlib/timer.hpp"
#include "lifting.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstdint>
#include <optional>

namespace brain_ui {
namespace {

constexpr std::uint32_t kRenderPeriodMs = 100;
constexpr std::uint32_t kLiftJogTimeoutMs = 500;

std::atomic<bool> _g_btn_left{false};
std::atomic<bool> _g_btn_center{false};
std::atomic<bool> _g_btn_right{false};

bool _g_lift_jog_armed = false;
std::optional<std::uint32_t> _g_lift_jog_started_ms;
std::optional<lifting::Position> _g_lift_lower_deg;
const char *_g_status = nullptr;
lemlib::Timer _g_render_timer(kRenderPeriodMs);

void on_btn_left() {
    _g_btn_left.store(true);
}

void on_btn_center() {
    _g_btn_center.store(true);
}

void on_btn_right() {
    _g_btn_right.store(true);
}

bool practice_allowed(bool driver_enabled) {
    return driver_enabled && !pros::competition::is_connected() &&
           !pros::competition::is_disabled() && !pros::competition::is_autonomous();
}

void handle_lift_limits(bool driver_enabled, bool set_low, bool set_up, bool use_saved) {
    const std::uint32_t now_ms = pros::millis();
    const bool allowed = practice_allowed(driver_enabled);
    auto &controller = io::prosController();
    const bool connected = allowed && controller.isConnected();
    const bool up = connected && controller.digital(io::Digital::X);
    const bool down = connected && controller.digital(io::Digital::B);
    const bool both = up && down;
    const bool moving_cmd = up != down;
    const auto position = lifting::read_position();
    const bool ready = allowed && connected && position.has_value();
    const bool capture = set_low || set_up || use_saved;

    if (!position) {
        _g_lift_lower_deg.reset();
    }

    if (lift_jog_mode()) {
        if (!ready || both || capture) {
            _g_lift_jog_armed = false;
        } else if (!moving_cmd) {
            _g_lift_jog_armed = true;
            _g_lift_jog_started_ms.reset();
        }

        if (_g_lift_jog_armed && moving_cmd) {
            if (!_g_lift_jog_started_ms) {
                _g_lift_jog_started_ms = now_ms;
            }
            if (now_ms - *_g_lift_jog_started_ms >= kLiftJogTimeoutMs) {
                _g_lift_jog_armed = false;
                _g_status = "JOG TIMEOUT - RELEASE X/B";
                lifting::stop();
            } else {
                _g_status = nullptr;
                lifting::jog(up ? 1 : -1);
            }
        } else {
            lifting::stop();
        }
    } else {
        _g_lift_jog_armed = false;
        _g_lift_jog_started_ms.reset();
    }

    if (capture) {
        if (!allowed) {
            _g_status = "UNPLUG FIELD & ENABLE DRIVER";
        } else if (!connected) {
            _g_status = "CONNECT CONTROLLER";
        } else if (!position) {
            _g_status = "CHECK MOTORS ON P9 AND P10";
        } else if (moving_cmd || _g_lift_jog_started_ms) {
            _g_status = "RELEASE X/B AND LET LIFT STOP";
        } else if (use_saved) {
            if (lifting::use_saved_limits(*position)) {
                _g_lift_lower_deg.reset();
                _g_status = "SAVED LIMITS ACTIVE";
            } else {
                _g_status = "NO SAVED LIMITS ON SD";
            }
        } else if (set_low) {
            _g_lift_lower_deg = position;
            _g_status = "LOWER SET | JOG UP -> SET UP";
        } else if (!_g_lift_lower_deg) {
            _g_status = "TAP SET LOW FIRST";
        } else if (lifting::set_limits({*_g_lift_lower_deg, *position})) {
            _g_lift_lower_deg.reset();
            _g_status = "LIMITS SAVED TO SD & ACTIVE";
        } else {
            _g_status = "CHECK ENCODER DIR / SD CARD";
        }
    }
}

void render_lift_limits(bool driver_enabled) {
    const auto now_pos = lifting::read_position();
    const auto &limits = lifting::limits();
    const auto lower = _g_lift_lower_deg ? _g_lift_lower_deg
                       : limits          ? std::optional{limits->lower_deg}
                                         : std::nullopt;
    const auto upper =
        (!_g_lift_lower_deg && limits) ? std::optional{limits->upper_deg} : std::nullopt;

    const char *status = _g_status;
    if (!now_pos) {
        status = "CHECK MOTORS ON P9 AND P10";
    } else if (!status) {
        if (_g_lift_lower_deg) {
            status = "JOG UP, THEN TAP SET UP";
        } else if (limits) {
            status = "LIMITS ACTIVE";
        } else if (!practice_allowed(driver_enabled)) {
            status = "UNPLUG FIELD & ENABLE DRIVER";
        } else if (lifting::has_saved_limits()) {
            status = "AT LOWER END, TAP USE SAVED";
        } else {
            status = "JOG DOWN, THEN TAP SET LOW";
        }
    }

    pros::lcd::print(0, "LIFT LIMITS (X up / B down)");
    if (now_pos) {
        pros::lcd::print(1, "NOW   P9:%7.1f  P10:%7.1f", (*now_pos)[0], (*now_pos)[1]);
    } else {
        pros::lcd::print(1, "NOW   P9:     --  P10:     --");
    }
    if (lower) {
        pros::lcd::print(2, "LOWER P9:%7.1f  P10:%7.1f", (*lower)[0], (*lower)[1]);
    } else {
        pros::lcd::print(2, "LOWER P9:     --  P10:     --");
    }
    if (upper) {
        pros::lcd::print(3, "UPPER P9:%7.1f  P10:%7.1f", (*upper)[0], (*upper)[1]);
    } else {
        pros::lcd::print(3, "UPPER P9:     --  P10:     --");
    }
    pros::lcd::print(4, "%s", status);
    pros::lcd::clear_line(5);
    pros::lcd::print(6, "[SET LOW]  [SET UP]  [USE SAVED]");
    pros::lcd::clear_line(7);
}

} // namespace

void initialize() {
    pros::lcd::initialize();
    pros::lcd::register_btn0_cb(on_btn_left);
    pros::lcd::register_btn1_cb(on_btn_center);
    pros::lcd::register_btn2_cb(on_btn_right);
    _g_render_timer.reset();
}

void reset_lift_calibration() {
    _g_lift_jog_armed = false;
    _g_lift_jog_started_ms.reset();
    _g_lift_lower_deg.reset();
    _g_status = nullptr;
}

bool lift_jog_mode() {
    return !lifting::limits() || _g_lift_lower_deg.has_value();
}

void tick(bool driver_enabled) {
    const bool set_low = _g_btn_left.exchange(false);
    const bool set_up = _g_btn_center.exchange(false);
    const bool use_saved = _g_btn_right.exchange(false);
    const bool had_button = set_low || set_up || use_saved;

    handle_lift_limits(driver_enabled, set_low, set_up, use_saved);

    if (had_button || _g_render_timer.isDone()) {
        _g_render_timer.reset();
        render_lift_limits(driver_enabled);
    }
}

} // namespace brain_ui
