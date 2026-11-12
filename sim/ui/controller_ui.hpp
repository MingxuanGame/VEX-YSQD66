#pragma once
#include "io/controller.hpp"
#include <cstdint>

#ifdef _WIN32
#include <windows.h>
#endif

namespace sim {

struct ConeVisual {
    double x_m = 0.0;
    double y_m = 0.0;
    bool active = true;
    bool out_of_bounds = false;
    int color = 2;
};

struct ToggleVisual {
    bool seated = true;
    int color = 2;
};

struct FieldState {
    static constexpr int kConeCount = 20;
    static constexpr int kToggleCount = 4;
    bool valid = false;
    double x_m = 0.0;
    double y_m = 0.0;
    double heading_deg = 0.0;
    double velocity_mps = 0.0;
    double current_ma = 0.0;
    double lift_height_m = 0.0;
    double clamp_angle_deg = 0.0;
    double push_position = 0.0;
    ConeVisual cones[kConeCount];
    int cone_count = 0;
    ToggleVisual toggles[kToggleCount];
    int toggle_count = 0;
    int mode = 0;
};

struct PlacementPose {
    double x_m = 0.0;
    double y_m = 0.0;
    double heading_deg = 90.0;
};

/**
 * @file controller_ui.hpp
 * @brief 仿真器手柄 + 赛场 UI。
 *
 * 窗口左侧：12ft × 12ft 场地俯视图和机器人轨迹，右侧：VEX 手柄。
 * 键盘：
 *   方向键   -> DRIVE：左摇杆 8 方向；TUNE：参数导航
 *   W/S/Q/E  -> TUNE：按住 L1 时低速点动
 *   I/J/K/L  -> 手柄方向键
 *   A/B/X/Y  -> 手柄 A/B/X/Y
 *   1/2      -> L1/L2
 *   3/4      -> R1/R2
 *   ESC      -> 退出
 *
 * TUNE 下把“参数导航”和“底盘点动”分到两套按键，避免上下切参数和
 * 前进/后退冲突。图形摇杆只做左摇杆 8 方向。
 */
class ControllerUi : public io::Controller {
public:
    ControllerUi();
    ~ControllerUi() override;

    bool open();
    void pump();
    void render();
    bool isOpen() const;

    bool isDown(io::Digital button) const;

    // 在窗口底部显示调参状态；传 nullptr 清空。
    void setStatusText(const char *line1, const char *line2 = nullptr);
    void setFieldState(const FieldState &state);
    void resetField();

    // 启动前放置：鼠标拖拽选择底盘位置，确认后锁定。
    void beginPlacement();
    bool isPlacementActive() const;
    PlacementPose getPlacementPose() const;
    bool consumeStartRequest();
    bool consumeRestartRequest();

    // TUNE 下方向键改为参数导航，W/S/Q/E 才驱动；切换时清空残留方向。
    void setTuneMode(bool tune_mode);

    // io::Controller：方向键提供 8 方向左摇杆；右摇杆暂未实现。
    bool isConnected() const override;
    int analog(io::Analog channel) const override;
    bool digital(io::Digital button) const override;

private:
#ifdef _WIN32
    static LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam);
    void paint(HWND hwnd);
    void paintField(HDC dc, int x, int y, int size);
    void paintController(HDC dc);
    void setKey(WPARAM key, bool down);
    void handleMouseDown(int client_x, int client_y);
    void handleMouseMove(int client_x, int client_y);
    void handleMouseUp();
    void setPlacementFromClient(int client_x, int client_y);
    bool clientToDesign(int client_x, int client_y, int *design_x, int *design_y) const;
    bool pointInField(int design_x, int design_y) const;
    void updateArrowAnalog();
    void updateJogAnalog();
#endif

#ifdef _WIN32
    HWND m_hwnd;
#else
    void *m_hwnd;
#endif
    bool m_open;
    bool m_keys[12];
    int m_analog_left_y;
    int m_analog_right_x;
    bool m_arrow_up;
    bool m_arrow_down;
    bool m_arrow_left;
    bool m_arrow_right;
    bool m_tune_mode;
    int m_jog_left_y;
    int m_jog_right_x;
    bool m_jog_up;
    bool m_jog_down;
    bool m_jog_left;
    bool m_jog_right;
    bool m_placement_active;
    bool m_placement_dragging;
    bool m_start_requested;
    bool m_restart_requested;
    double m_placement_x_m;
    double m_placement_y_m;
    double m_placement_heading_deg;
    char m_status0[64];
    char m_status1[64];

    FieldState m_field;

    struct TrailPoint {
        float x;
        float y;
    };

    static constexpr int kMaxTrail = 256;
    TrailPoint m_trail[kMaxTrail];
    int m_trail_count;
};

} // namespace sim
