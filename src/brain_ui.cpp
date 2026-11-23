#include "brain_ui.hpp"

#include "chassis.h"
#include "liblvgl/lvgl.h"
#include "lifting.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>

extern "C" {
LV_FONT_DECLARE(brain_menu_font);
}

namespace brain_ui {
namespace {

constexpr std::size_t kPresetCount = static_cast<std::size_t>(AutonPreset::Count);
constexpr std::size_t kPlanCount = static_cast<std::size_t>(AutonPlan::Count);
constexpr std::array<const char *, kPlanCount> kPlanLabels{"Plan 1", "Plan 2", "Plan 3"};
constexpr std::uint32_t kReadoutRefreshMs = 100;
constexpr char kPoseUnavailable[] = "Pose  X -- in   Y -- in   Theta -- deg";

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
std::atomic<AutonPlan> _g_selected_plan{AutonPlan::Plan1};
std::atomic<AutonPlan> _g_pending_plan{AutonPlan::Count};
bool _g_ui_ready = false;
lv_obj_t *_g_auton_header = nullptr;
std::array<lv_obj_t *, kPresetCount> _g_auton_buttons{};
std::array<lv_obj_t *, kPlanCount> _g_plan_buttons{};

void on_auton_clicked(lv_event_t *event) {
    const auto index = reinterpret_cast<std::uintptr_t>(lv_event_get_user_data(event));
    if (index < kPresetCount) {
        _g_pending_auton.store(static_cast<int>(index));
    }
}

void on_plan_clicked(lv_event_t *event) {
    const auto index = reinterpret_cast<std::uintptr_t>(lv_event_get_user_data(event));
    _g_pending_plan.store(static_cast<AutonPlan>(index));
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

std::array<char, 24> format_angle(std::optional<double> angle_deg) {
    std::array<char, 24> text{"--"};
    if (angle_deg) {
        // 镜像端口的零位可能返回 -0.0，两列统一显示 +0.0。
        const double display_deg = *angle_deg == 0.0 ? 0.0 : *angle_deg;
        std::snprintf(text.data(), text.size(), "%+.1f", display_deg);
    }
    return text;
}

void set_text_if_changed(lv_obj_t *label, const char *text) {
    if (std::strcmp(lv_label_get_text(label), text) != 0) {
        lv_label_set_text(label, text);
    }
}

void refresh_lift_angles(lv_timer_t *timer) {
    auto *label = static_cast<lv_obj_t *>(lv_timer_get_user_data(timer));
    const auto angles = lifting::motor_angles();
    const auto port_9 = format_angle(angles.port_9_deg);
    const auto port_10 = format_angle(angles.port_10_deg);
    std::array<char, 64> text{};
    std::snprintf(text.data(), text.size(), "P9 / P10  {%s, %s} deg", port_9.data(),
                  port_10.data());
    set_text_if_changed(label, text.data());
}

void refresh_chassis_readout(lv_timer_t *timer) {
    auto *label = static_cast<lv_obj_t *>(lv_timer_get_user_data(timer));
    const auto rotation = format_angle(chassis::imu_rotation_deg());
    const auto pose = chassis::pose();
    std::array<char, 96> pose_text{};
    if (std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta)) {
        std::snprintf(pose_text.data(), pose_text.size(),
                      "Pose  X %+.2f in   Y %+.2f in   Theta %+.2f deg", pose.x, pose.y,
                      pose.theta);
    }

    std::array<char, 160> text{};
    std::snprintf(text.data(), text.size(), "IMU rotation  %s deg\n%s", rotation.data(),
                  pose_text.front() ? pose_text.data() : kPoseUnavailable);
    set_text_if_changed(label, text.data());
}

void style_selection(lv_obj_t *button, std::uint32_t background, bool selected) {
    lv_obj_set_style_bg_color(button, lv_color_hex(background), LV_PART_MAIN);
    lv_obj_set_style_border_width(button, selected ? 3 : 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(button, lv_color_hex(selected ? 0xFFD700 : 0x475B6E),
                                  LV_PART_MAIN);
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
        style_selection(_g_auton_buttons[i], bg, selected);
    }

    const auto active_plan = static_cast<std::size_t>(selected_plan());
    for (std::size_t i = 0; i < kPlanCount; ++i) {
        const bool selected = (i == active_plan);
        style_selection(_g_plan_buttons[i], selected ? 0x3B5269 : 0x23313D, selected);
    }
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
    constexpr int32_t kAutonY = 26;
    constexpr int32_t kAutonH = 48;
    for (std::size_t i = 0; i < kPresetCount; ++i) {
        const int32_t x = 10 + static_cast<int32_t>(i) * (kAutonW + kAutonGap);
        _g_auton_buttons[i] =
            make_button(screen, x, kAutonY, kAutonW, kAutonH, 0x23313D, kPresets[i].short_label,
                        on_auton_clicked, reinterpret_cast<void *>(static_cast<std::uintptr_t>(i)));
    }

    constexpr int32_t kPlanY = kAutonY + kAutonH + kAutonGap;
    // 三个方案按钮平分整行：3 × 148 + 2 × 8 间距，左右各留 10 px。
    constexpr int32_t kPlanW = 148;
    constexpr int32_t kPlanH = 40;
    for (std::size_t i = 0; i < kPlanCount; ++i) {
        const int32_t x = 10 + static_cast<int32_t>(i) * (kPlanW + kAutonGap);
        _g_plan_buttons[i] =
            make_button(screen, x, kPlanY, kPlanW, kPlanH, 0x23313D, kPlanLabels[i],
                        on_plan_clicked, reinterpret_cast<void *>(static_cast<std::uintptr_t>(i)));
    }

    constexpr int32_t kLiftY = kPlanY + kPlanH + 4;
    make_label(screen, 10, kLiftY, 0xFFFFFF, &brain_menu_font, "升降");
    auto *lift_angles =
        make_label(screen, 52, kLiftY, 0xFFFFFF, &lv_font_montserrat_16, "P9 / P10  {--, --} deg");
    lv_timer_create(refresh_lift_angles, kReadoutRefreshMs, lift_angles);

    auto *chassis_readout =
        make_label(screen, 10, kLiftY + 24, 0xFFFFFF, &lv_font_montserrat_16,
                   "IMU rotation  -- deg\nPose  X -- in   Y -- in   Theta -- deg");
    lv_timer_create(refresh_chassis_readout, kReadoutRefreshMs, chassis_readout);

    apply_auton_selection_styles();
    lv_unlock();

    _g_ui_ready = true;
}

AutonPreset selected_auton() {
    const int raw = _g_selected_auton.load();
    if (raw < 0 || raw >= static_cast<int>(AutonPreset::Count)) {
        return AutonPreset::RedLeft;
    }
    return static_cast<AutonPreset>(raw);
}

AutonPlan selected_plan() {
    return _g_selected_plan.load();
}

const char *auton_name(AutonPreset preset) {
    const auto index = static_cast<std::size_t>(preset);
    return index < kPresetCount ? kPresets[index].full_name : kPresets[0].full_name;
}

void tick() {
    const int pending = _g_pending_auton.exchange(-1);
    const auto pending_plan = _g_pending_plan.exchange(AutonPlan::Count);
    if (!_g_ui_ready || (pending < 0 && pending_plan == AutonPlan::Count)) {
        return;
    }
    if (pending >= 0) {
        _g_selected_auton.store(pending);
    }
    if (pending_plan != AutonPlan::Count) {
        _g_selected_plan.store(pending_plan);
    }
    lv_lock();
    apply_auton_selection_styles();
    lv_unlock();
}

} // namespace brain_ui
