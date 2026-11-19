#include "brain_ui.hpp"

#include "io/pros_controller.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/lvgl.h"
#include "lifting.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

namespace brain_ui {
namespace {

constexpr std::uint32_t kRenderPeriodMs = 100;
constexpr std::uint32_t kLiftJogTimeoutMs = 500;
constexpr std::size_t kPresetCount = static_cast<std::size_t>(AutonPreset::Count);

struct PresetMeta {
    const char *short_label;
    const char *full_name;
    bool is_red;
};

constexpr std::array<PresetMeta, kPresetCount> kPresets{{
    {"RED\nLEFT", "RED LEFT", true},
    {"RED\nDOWN", "RED DOWN", true},
    {"BLUE\nRIGHT", "BLUE RIGHT", false},
    {"BLUE\nUP", "BLUE UP", false},
}};

std::atomic<int> _g_selected_auton{static_cast<int>(AutonPreset::RedLeft)};
std::atomic<int> _g_pending_auton{-1};
std::atomic<bool> _g_btn_set_low{false};
std::atomic<bool> _g_btn_set_up{false};
std::atomic<bool> _g_btn_use_saved{false};

bool _g_ui_ready = false;
bool _g_lift_jog_armed = false;
std::optional<std::uint32_t> _g_lift_jog_started_ms;
std::optional<lifting::Position> _g_lift_lower_deg;
const char *_g_status = nullptr;
lemlib::Timer _g_render_timer(kRenderPeriodMs);

lv_obj_t *_g_auton_header = nullptr;
std::array<lv_obj_t *, kPresetCount> _g_auton_buttons{};
lv_obj_t *_g_now_label = nullptr;
lv_obj_t *_g_low_label = nullptr;
lv_obj_t *_g_up_label = nullptr;
lv_obj_t *_g_status_label = nullptr;

void on_auton_clicked(lv_event_t *event) {
    const auto index = reinterpret_cast<std::uintptr_t>(lv_event_get_user_data(event));
    if (index < kPresetCount) {
        _g_pending_auton.store(static_cast<int>(index));
    }
}

void on_set_low_clicked(lv_event_t *event) {
    (void)event;
    _g_btn_set_low.store(true);
}

void on_set_up_clicked(lv_event_t *event) {
    (void)event;
    _g_btn_set_up.store(true);
}

void on_use_saved_clicked(lv_event_t *event) {
    (void)event;
    _g_btn_use_saved.store(true);
}

lv_obj_t *make_button(lv_obj_t *parent, int32_t x, int32_t y, int32_t w, int32_t h,
                      std::uint32_t bg_hex, const char *text, lv_event_cb_t cb, void *user_data) {
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(bg_hex), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x475B6E), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_center(label);
    return btn;
}

lv_obj_t *make_label(lv_obj_t *parent, int32_t x, int32_t y, std::uint32_t color_hex,
                     const lv_font_t *font, const char *text) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_color(label, lv_color_hex(color_hex), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_label_set_text(label, text);
    return label;
}

void apply_auton_selection_styles() {
    const auto active = static_cast<std::size_t>(selected_auton());
    std::array<char, 40> header{};
    std::snprintf(header.data(), header.size(), "AUTO: %s", kPresets[active].full_name);
    lv_label_set_text(_g_auton_header, header.data());

    for (std::size_t i = 0; i < kPresetCount; ++i) {
        const bool selected = (i == active);
        const std::uint32_t bg = kPresets[i].is_red ? (selected ? 0xD63031 : 0x6E1B1B)
                                                    : (selected ? 0x2E86DE : 0x194575);
        lv_obj_set_style_bg_color(_g_auton_buttons[i], lv_color_hex(bg), LV_PART_MAIN);
        lv_obj_set_style_border_width(_g_auton_buttons[i], selected ? 3 : 1, LV_PART_MAIN);
        lv_obj_set_style_border_color(_g_auton_buttons[i],
                                      lv_color_hex(selected ? 0xFFD700 : 0x475B6E), LV_PART_MAIN);
    }
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

void format_pos_line(char *buf, std::size_t size, const char *tag,
                     const std::optional<lifting::Position> &pos) {
    if (pos) {
        std::snprintf(buf, size, "%-5s P9: %7.1f   P10: %7.1f", tag, (*pos)[0], (*pos)[1]);
    } else {
        std::snprintf(buf, size, "%-5s P9:      --   P10:      --", tag);
    }
}

void render_ui(bool driver_enabled, bool refresh_auton) {
    if (!_g_ui_ready) {
        return;
    }

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
            status = "AT LOWER END -> TAP USE SAVED";
        } else {
            status = "JOG DOWN -> TAP SET LOW";
        }
    }

    std::array<char, 48> now_buf{};
    std::array<char, 48> low_buf{};
    std::array<char, 48> up_buf{};
    format_pos_line(now_buf.data(), now_buf.size(), "NOW", now_pos);
    format_pos_line(low_buf.data(), low_buf.size(), "LOW", lower);
    format_pos_line(up_buf.data(), up_buf.size(), "UP", upper);

    lv_lock();
    if (refresh_auton) {
        apply_auton_selection_styles();
    }
    lv_label_set_text(_g_now_label, now_buf.data());
    lv_label_set_text(_g_low_label, low_buf.data());
    lv_label_set_text(_g_up_label, up_buf.data());
    lv_label_set_text(_g_status_label, status);
    lv_unlock();
}

} // namespace

void initialize() {
    for (int attempt = 0; attempt < 20 && !lv_display_get_default(); ++attempt) {
        pros::delay(10);
    }
    if (!lv_display_get_default()) {
        return;
    }

    lv_lock();
    lv_obj_t *screen = lv_screen_active();
    lv_obj_clean(screen);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    _g_auton_header = make_label(screen, 10, 4, 0xFFFFFF, &lv_font_montserrat_16, "AUTO: RED LEFT");
    make_label(screen, 336, 6, 0x8FA3B8, &lv_font_montserrat_14, "AUTON PRESET");

    constexpr int32_t kAutonW = 109;
    constexpr int32_t kAutonGap = 8;
    for (std::size_t i = 0; i < kPresetCount; ++i) {
        const int32_t x = 10 + static_cast<int32_t>(i) * (kAutonW + kAutonGap);
        _g_auton_buttons[i] =
            make_button(screen, x, 26, kAutonW, 48, 0x23313D, kPresets[i].short_label,
                        on_auton_clicked, reinterpret_cast<void *>(static_cast<std::uintptr_t>(i)));
    }

    make_label(screen, 10, 82, 0x8FA3B8, &lv_font_montserrat_14, "LIFT LIMITS (X up / B down)");
    _g_now_label = make_label(screen, 10, 102, 0xFFFFFF, &lv_font_montserrat_14, "NOW   P9: --");
    _g_low_label = make_label(screen, 10, 120, 0xD0DCE5, &lv_font_montserrat_14, "LOW   P9: --");
    _g_up_label = make_label(screen, 10, 138, 0xD0DCE5, &lv_font_montserrat_14, "UP    P9: --");
    _g_status_label =
        make_label(screen, 10, 158, 0x55EFC4, &lv_font_montserrat_14, "INITIALIZING...");

    make_button(screen, 10, 184, 148, 48, 0x236A93, "SET LOW", on_set_low_clicked, nullptr);
    make_button(screen, 166, 184, 148, 48, 0x236A93, "SET UP", on_set_up_clicked, nullptr);
    make_button(screen, 322, 184, 148, 48, 0x127766, "USE SAVED", on_use_saved_clicked, nullptr);

    apply_auton_selection_styles();
    lv_unlock();

    _g_ui_ready = true;
    _g_render_timer.reset();
}

AutonPreset selected_auton() {
    const int raw = _g_selected_auton.load();
    if (raw < 0 || raw >= static_cast<int>(AutonPreset::Count)) {
        return AutonPreset::RedLeft;
    }
    return static_cast<AutonPreset>(raw);
}

const char *auton_name(AutonPreset preset) {
    const auto index = static_cast<std::size_t>(preset);
    return index < kPresetCount ? kPresets[index].full_name : kPresets[0].full_name;
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
    const int pending_auton = _g_pending_auton.exchange(-1);
    bool auton_changed = false;
    if (pending_auton >= 0 && pending_auton < static_cast<int>(AutonPreset::Count)) {
        _g_selected_auton.store(pending_auton);
        auton_changed = true;
    }

    const bool set_low = _g_btn_set_low.exchange(false);
    const bool set_up = _g_btn_set_up.exchange(false);
    const bool use_saved = _g_btn_use_saved.exchange(false);
    const bool had_button = auton_changed || set_low || set_up || use_saved;

    handle_lift_limits(driver_enabled, set_low, set_up, use_saved);

    if (had_button || _g_render_timer.isDone()) {
        _g_render_timer.reset();
        render_ui(driver_enabled, auton_changed);
    }
}

} // namespace brain_ui
