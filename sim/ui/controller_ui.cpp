#include "sim/ui/controller_ui.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace sim {

#ifdef _WIN32

namespace {

constexpr int kWindowWidth = 1200;
constexpr int kWindowHeight = 650;
constexpr int kControllerWidth = 680;
constexpr int kControllerHeight = 530;
constexpr int kFieldX = 20;
constexpr int kFieldY = 40;
constexpr int kFieldSize = 590;
constexpr int kControllerX = 630;
constexpr int kControllerY = 50;
constexpr int kControllerDrawW = 540;
constexpr int kControllerDrawH = 420;
constexpr const char *kWindowClass = "VexSimControllerUi";

constexpr double kFieldSizeM = 3.6576; // 12 ft

int digitalIndex(io::Digital button) {
    return static_cast<int>(button);
}

int keyToDigital(WPARAM key) {
    switch (key) {
    case 'A':
        return digitalIndex(io::Digital::A);
    case 'B':
        return digitalIndex(io::Digital::B);
    case 'X':
        return digitalIndex(io::Digital::X);
    case 'Y':
        return digitalIndex(io::Digital::Y);
    case '1':
        return digitalIndex(io::Digital::L1);
    case '2':
        return digitalIndex(io::Digital::L2);
    case '3':
        return digitalIndex(io::Digital::R1);
    case '4':
        return digitalIndex(io::Digital::R2);
    case 'I':
        return digitalIndex(io::Digital::Up);
    case 'K':
        return digitalIndex(io::Digital::Down);
    case 'J':
        return digitalIndex(io::Digital::Left);
    case 'L':
        return digitalIndex(io::Digital::Right);
    default:
        return -1;
    }
}

const char *modeName(int mode) {
    switch (mode) {
    case 0:
        return "DRIVE";
    case 1:
        return "TUNE";
    case 2:
        return "AUTO";
    case 3:
        return "SAFE_WAIT";
    default:
        return "UNKNOWN";
    }
}

void drawText(HDC dc, int x, int y, int w, int h, const char *text, COLORREF color,
              UINT format = DT_CENTER | DT_VCENTER | DT_SINGLELINE) {
    SetTextColor(dc, color);
    RECT rect{x, y, x + w, y + h};
    DrawTextA(dc, text, -1, &rect, format);
}

void drawButton(HDC dc, int x, int y, int w, int h, const char *label, COLORREF base,
                bool pressed) {
    const COLORREF fill =
        pressed ? RGB(std::min(255, GetRValue(base) + 70), std::min(255, GetGValue(base) + 70),
                      std::min(255, GetBValue(base) + 70))
                : base;
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 2, RGB(20, 22, 28));
    HGDIOBJ old_brush = SelectObject(dc, brush);
    HGDIOBJ old_pen = SelectObject(dc, pen);

    RoundRect(dc, x, y, x + w, y + h, 14, 14);
    drawText(dc, x, y, w, h, label, RGB(255, 255, 255));

    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(brush);
    DeleteObject(pen);
}

void drawCircle(HDC dc, int cx, int cy, int radius, const char *label, COLORREF base,
                bool pressed) {
    const COLORREF fill =
        pressed ? RGB(std::min(255, GetRValue(base) + 70), std::min(255, GetGValue(base) + 70),
                      std::min(255, GetBValue(base) + 70))
                : base;
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 2, RGB(20, 22, 28));
    HGDIOBJ old_brush = SelectObject(dc, brush);
    HGDIOBJ old_pen = SelectObject(dc, pen);

    Ellipse(dc, cx - radius, cy - radius, cx + radius, cy + radius);
    drawText(dc, cx - radius, cy - radius, radius * 2, radius * 2, label, RGB(255, 255, 255));

    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(brush);
    DeleteObject(pen);
}

void worldToScreen(double wx, double wy, int x, int y, int size, int *sx, int *sy) {
    const double scale = static_cast<double>(size) / kFieldSizeM;
    *sx = x + static_cast<int>(wx * scale);
    *sy = y + size - static_cast<int>(wy * scale);
}

} // namespace

ControllerUi::ControllerUi()
    : m_hwnd(nullptr), m_open(false), m_analog_left_y(0), m_analog_right_x(0), m_arrow_up(false),
      m_arrow_down(false), m_arrow_left(false), m_arrow_right(false), m_tune_mode(false),
      m_jog_left_y(0), m_jog_right_x(0), m_jog_up(false), m_jog_down(false), m_jog_left(false),
      m_jog_right(false), m_trail_count(0) {
    for (bool &key : m_keys)
        key = false;
    m_status0[0] = '\0';
    m_status1[0] = '\0';
}

ControllerUi::~ControllerUi() {
    if (m_hwnd) {
        DestroyWindow(m_hwnd);
        m_hwnd = nullptr;
    }
}

bool ControllerUi::open() {
    SetProcessDPIAware();
    HINSTANCE instance = GetModuleHandleA(nullptr);

    WNDCLASSA wc{};
    wc.lpfnWndProc = &ControllerUi::windowProc;
    wc.hInstance = instance;
    wc.lpszClassName = kWindowClass;
    wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);
    wc.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
    RegisterClassA(&wc);

    RECT rect{0, 0, kWindowWidth, kWindowHeight};
    const DWORD style = WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX;
    AdjustWindowRect(&rect, style, FALSE);

    m_hwnd = CreateWindowExA(0, kWindowClass, "VEX V5 Simulator | Field + Controller", style,
                             CW_USEDEFAULT, CW_USEDEFAULT, rect.right - rect.left,
                             rect.bottom - rect.top, nullptr, nullptr, instance, this);
    if (!m_hwnd)
        return false;

    ShowWindow(m_hwnd, SW_SHOW);
    UpdateWindow(m_hwnd);
    SetForegroundWindow(m_hwnd);
    SetFocus(m_hwnd);
    m_open = true;
    return true;
}

void ControllerUi::pump() {
    if (!m_open)
        return;

    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) {
            m_open = false;
            break;
        }
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
}

void ControllerUi::render() {
    if (!m_open || !m_hwnd)
        return;
    InvalidateRect(m_hwnd, nullptr, FALSE);
    UpdateWindow(m_hwnd);
}

bool ControllerUi::isOpen() const {
    return m_open;
}

bool ControllerUi::isDown(io::Digital button) const {
    const int index = digitalIndex(button);
    return index >= 0 && index < 12 && m_keys[index];
}

bool ControllerUi::isConnected() const {
    return true;
}

int ControllerUi::analog(io::Analog channel) const {
    if (channel == io::Analog::LeftY) {
        return m_tune_mode ? m_jog_left_y : m_analog_left_y;
    }
    if (channel == io::Analog::RightX) {
        return m_tune_mode ? m_jog_right_x : m_analog_right_x;
    }
    return 0;
}

bool ControllerUi::digital(io::Digital button) const {
    return isDown(button);
}

void ControllerUi::setStatusText(const char *line1, const char *line2) {
    if (line1) {
        std::strncpy(m_status0, line1, sizeof(m_status0) - 1);
        m_status0[sizeof(m_status0) - 1] = '\0';
    } else {
        m_status0[0] = '\0';
    }
    if (line2) {
        std::strncpy(m_status1, line2, sizeof(m_status1) - 1);
        m_status1[sizeof(m_status1) - 1] = '\0';
    } else {
        m_status1[0] = '\0';
    }
    if (m_hwnd)
        InvalidateRect(m_hwnd, nullptr, FALSE);
}

void ControllerUi::setFieldState(const FieldState &state) {
    m_field = state;
    if (!state.valid)
        return;

    bool add = (m_trail_count == 0);
    if (!add) {
        const TrailPoint &last = m_trail[m_trail_count - 1];
        const double dx = state.x_m - last.x;
        const double dy = state.y_m - last.y;
        add = (dx * dx + dy * dy) > 0.0004; // 2 cm
    }
    if (add) {
        if (m_trail_count >= kMaxTrail) {
            for (int i = 0; i < kMaxTrail - 1; ++i) {
                m_trail[i] = m_trail[i + 1];
            }
            --m_trail_count;
        }
        m_trail[m_trail_count].x = static_cast<float>(state.x_m);
        m_trail[m_trail_count].y = static_cast<float>(state.y_m);
        ++m_trail_count;
    }
    if (m_hwnd)
        InvalidateRect(m_hwnd, nullptr, FALSE);
}

void ControllerUi::resetField() {
    m_trail_count = 0;
    m_field = FieldState{};
    if (m_hwnd)
        InvalidateRect(m_hwnd, nullptr, FALSE);
}

void ControllerUi::setKey(WPARAM key, bool down) {
    if (key == VK_ESCAPE) {
        if (down) {
            m_open = false;
            HWND hwnd = m_hwnd;
            m_hwnd = nullptr;
            if (hwnd)
                DestroyWindow(hwnd);
        }
        return;
    }

    // TUNE：方向键切参数，W/S/Q/E 才驱动；DRIVE：方向键是 8 方向摇杆。
    if (m_tune_mode) {
        int arrow_index = -1;
        if (key == VK_UP)
            arrow_index = digitalIndex(io::Digital::Up);
        if (key == VK_DOWN)
            arrow_index = digitalIndex(io::Digital::Down);
        if (key == VK_LEFT)
            arrow_index = digitalIndex(io::Digital::Left);
        if (key == VK_RIGHT)
            arrow_index = digitalIndex(io::Digital::Right);
        if (arrow_index >= 0) {
            m_keys[arrow_index] = down;
            if (m_hwnd)
                InvalidateRect(m_hwnd, nullptr, FALSE);
            return;
        }
    } else {
        switch (key) {
        case VK_UP:
            m_arrow_up = down;
            updateArrowAnalog();
            break;
        case VK_DOWN:
            m_arrow_down = down;
            updateArrowAnalog();
            break;
        case VK_LEFT:
            m_arrow_left = down;
            updateArrowAnalog();
            break;
        case VK_RIGHT:
            m_arrow_right = down;
            updateArrowAnalog();
            break;
        default:
            break;
        }
    }

    switch (key) {
    case 'W':
        m_jog_up = down;
        updateJogAnalog();
        break;
    case 'S':
        m_jog_down = down;
        updateJogAnalog();
        break;
    case 'Q':
        m_jog_left = down;
        updateJogAnalog();
        break;
    case 'E':
        m_jog_right = down;
        updateJogAnalog();
        break;
    default:
        break;
    }

    const int index = keyToDigital(key);
    if (index >= 0 && index < 12) {
        m_keys[index] = down;
    }
    if (m_hwnd)
        InvalidateRect(m_hwnd, nullptr, FALSE);
}

void ControllerUi::setTuneMode(bool tune_mode) {
    if (m_tune_mode == tune_mode)
        return;
    m_tune_mode = tune_mode;
    m_arrow_up = false;
    m_arrow_down = false;
    m_arrow_left = false;
    m_arrow_right = false;
    m_analog_left_y = 0;
    m_analog_right_x = 0;
    m_keys[digitalIndex(io::Digital::Up)] = false;
    m_keys[digitalIndex(io::Digital::Down)] = false;
    m_keys[digitalIndex(io::Digital::Left)] = false;
    m_keys[digitalIndex(io::Digital::Right)] = false;
    m_jog_up = false;
    m_jog_down = false;
    m_jog_left = false;
    m_jog_right = false;
    m_jog_left_y = 0;
    m_jog_right_x = 0;
    if (m_hwnd)
        InvalidateRect(m_hwnd, nullptr, FALSE);
}

void ControllerUi::updateArrowAnalog() {
    int left_y = 0;
    int right_x = 0;
    if (m_arrow_up)
        left_y += 127;
    if (m_arrow_down)
        left_y -= 127;
    if (m_arrow_right)
        right_x += 127;
    if (m_arrow_left)
        right_x -= 127;
    m_analog_left_y = left_y;
    m_analog_right_x = right_x;
}

void ControllerUi::updateJogAnalog() {
    int left_y = 0;
    int right_x = 0;
    if (m_jog_up)
        left_y += 127;
    if (m_jog_down)
        left_y -= 127;
    if (m_jog_right)
        right_x += 127;
    if (m_jog_left)
        right_x -= 127;
    m_jog_left_y = left_y;
    m_jog_right_x = right_x;
}

LRESULT CALLBACK ControllerUi::windowProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
    ControllerUi *self = reinterpret_cast<ControllerUi *>(GetWindowLongPtrA(hwnd, GWLP_USERDATA));

    if (message == WM_NCCREATE) {
        auto *create = reinterpret_cast<CREATESTRUCTA *>(lparam);
        self = static_cast<ControllerUi *>(create->lpCreateParams);
        SetWindowLongPtrA(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        self->m_hwnd = hwnd;
    }

    switch (message) {
    case WM_KEYDOWN:
    case WM_SYSKEYDOWN:
        if (self)
            self->setKey(wparam, true);
        return 0;
    case WM_KEYUP:
    case WM_SYSKEYUP:
        if (self)
            self->setKey(wparam, false);
        return 0;
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT:
        if (self)
            self->paint(hwnd);
        return 0;
    case WM_CLOSE:
        if (self)
            self->m_open = false;
        DestroyWindow(hwnd);
        return 0;
    case WM_DESTROY:
        if (self) {
            self->m_open = false;
            self->m_hwnd = nullptr;
        }
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProcA(hwnd, message, wparam, lparam);
    }
}

void ControllerUi::paintField(HDC dc, int x, int y, int size) {
    HBRUSH field_brush = CreateSolidBrush(RGB(112, 115, 117));
    HBRUSH wall_brush = CreateSolidBrush(RGB(195, 198, 200));
    HBRUSH black_brush = CreateSolidBrush(RGB(20, 22, 24));
    HBRUSH yellow_brush = CreateSolidBrush(RGB(225, 195, 45));
    HBRUSH blue_brush = CreateSolidBrush(RGB(40, 105, 200));
    HBRUSH red_brush = CreateSolidBrush(RGB(205, 55, 55));
    HBRUSH white_brush = CreateSolidBrush(RGB(235, 238, 240));
    HBRUSH ghost_brush = CreateSolidBrush(RGB(82, 86, 90));
    HBRUSH bucket_brush = CreateSolidBrush(RGB(145, 50, 40));
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(82, 85, 87));
    HPEN wall_pen = CreatePen(PS_SOLID, 5, RGB(195, 198, 200));
    HPEN white_pen = CreatePen(PS_SOLID, 3, RGB(235, 238, 240));
    HPEN red_pen = CreatePen(PS_SOLID, 4, RGB(210, 50, 55));
    HPEN blue_pen = CreatePen(PS_SOLID, 4, RGB(45, 105, 215));
    HPEN black_pen = CreatePen(PS_SOLID, 1, RGB(20, 22, 24));

    HGDIOBJ old_brush = SelectObject(dc, wall_brush);
    HGDIOBJ old_pen = SelectObject(dc, wall_pen);
    Rectangle(dc, x - 2, y - 2, x + size + 2, y + size + 2);
    SelectObject(dc, field_brush);
    Rectangle(dc, x + 4, y + 4, x + size - 4, y + size - 4);

    // 地垫接缝：保留 6 格，但颜色压暗，避免像 UI 网格而不是场地。
    SelectObject(dc, grid_pen);
    for (int i = 1; i < 6; ++i) {
        const int offset = 4 + (size - 8) * i / 6;
        MoveToEx(dc, x + offset, y + 4, nullptr);
        LineTo(dc, x + offset, y + size - 4);
        MoveToEx(dc, x + 4, y + offset, nullptr);
        LineTo(dc, x + size - 4, y + offset);
    }

    auto px = [&](double normalized) { return x + static_cast<int>(normalized * size); };
    auto py = [&](double normalized) { return y + static_cast<int>(normalized * size); };

    // 中央自动分界线：双线 X + 中央菱形。
    SelectObject(dc, white_pen);
    for (int offset : {-2, 3}) {
        MoveToEx(dc, px(0.08) + offset, py(0.08), nullptr);
        LineTo(dc, px(0.50) + offset, py(0.50));
        MoveToEx(dc, px(0.92) + offset, py(0.08), nullptr);
        LineTo(dc, px(0.50) + offset, py(0.50));
        MoveToEx(dc, px(0.08) + offset, py(0.92), nullptr);
        LineTo(dc, px(0.50) + offset, py(0.50));
        MoveToEx(dc, px(0.92) + offset, py(0.92), nullptr);
        LineTo(dc, px(0.50) + offset, py(0.50));
    }
    MoveToEx(dc, px(0.50), py(0.32), nullptr);
    LineTo(dc, px(0.68), py(0.50));
    LineTo(dc, px(0.50), py(0.68));
    LineTo(dc, px(0.32), py(0.50));
    LineTo(dc, px(0.50), py(0.32));

    // 四角红/蓝自动区边线。
    SelectObject(dc, red_pen);
    MoveToEx(dc, px(0.04), py(0.18), nullptr);
    LineTo(dc, px(0.04), py(0.04));
    LineTo(dc, px(0.18), py(0.04));
    MoveToEx(dc, px(0.04), py(0.82), nullptr);
    LineTo(dc, px(0.04), py(0.96));
    LineTo(dc, px(0.18), py(0.96));
    SelectObject(dc, blue_pen);
    MoveToEx(dc, px(0.82), py(0.04), nullptr);
    LineTo(dc, px(0.96), py(0.04));
    LineTo(dc, px(0.96), py(0.18));
    MoveToEx(dc, px(0.82), py(0.96), nullptr);
    LineTo(dc, px(0.96), py(0.96));
    LineTo(dc, px(0.96), py(0.82));

    // 场地元素的描边统一用细黑笔，避免自动分界线的粗蓝/红笔污染圆环。
    SelectObject(dc, black_pen);

    // 9 个场上得分座：对应俯视图红圈标出的 5 中立方 + 2 蓝方 + 2 红方。
    enum class GoalKind { Neutral, Blue, Red };

    struct Goal {
        double x;
        double y;
        GoalKind kind;
    };

    const Goal goals[] = {{0.655, 0.345, GoalKind::Neutral}, {0.510, 0.495, GoalKind::Neutral},
                          {0.340, 0.655, GoalKind::Neutral}, {0.190, 0.820, GoalKind::Neutral},
                          {0.350, 0.345, GoalKind::Neutral}, {0.500, 0.350, GoalKind::Blue},
                          {0.660, 0.495, GoalKind::Blue},    {0.345, 0.495, GoalKind::Red},
                          {0.500, 0.655, GoalKind::Red}};
    auto draw_goal = [&](double nx, double ny, GoalKind kind) {
        const int cx = px(nx);
        const int cy = py(ny);
        SelectObject(dc, black_brush);
        Ellipse(dc, cx - 12, cy - 12, cx + 12, cy + 12);
        SelectObject(dc, white_brush);
        Ellipse(dc, cx - 9, cy - 9, cx + 9, cy + 9);
        if (kind == GoalKind::Neutral) {
            SelectObject(dc, yellow_brush);
            Ellipse(dc, cx - 6, cy - 6, cx + 6, cy + 6);
        } else if (kind == GoalKind::Blue) {
            SelectObject(dc, blue_brush);
            Ellipse(dc, cx - 7, cy - 7, cx + 7, cy + 7);
        } else {
            SelectObject(dc, red_brush);
            Ellipse(dc, cx - 7, cy - 7, cx + 7, cy + 7);
        }
    };
    for (const Goal &goal : goals) {
        draw_goal(goal.x, goal.y, goal.kind);
    }

    // 场地围边上的 8 个黑/黄场地节点（俯视图红圈中标出的部分）。
    const double edge_nodes[8][2] = {{0.34, 0.03}, {0.62, 0.03}, {0.34, 0.97}, {0.62, 0.97},
                                     {0.03, 0.35}, {0.03, 0.64}, {0.97, 0.35}, {0.97, 0.64}};
    for (const auto &node : edge_nodes) {
        const int cx = px(node[0]);
        const int cy = py(node[1]);
        SelectObject(dc, black_brush);
        Ellipse(dc, cx - 10, cy - 10, cx + 10, cy + 10);
        SelectObject(dc, yellow_brush);
        Ellipse(dc, cx - 5, cy - 5, cx + 5, cy + 5);
    }

    // 4 个三色筒：四边中点，当前朝向为黄色。
    const int bar = size / 10;
    const int bar_thick = 6;
    SelectObject(dc, black_brush);
    Rectangle(dc, x + size / 2 - bar / 2, y + 1, x + size / 2 + bar / 2, y + bar_thick);
    Rectangle(dc, x + size / 2 - bar / 2, y + size - bar_thick, x + size / 2 + bar / 2,
              y + size - 1);
    Rectangle(dc, x + 1, y + size / 2 - bar / 2, x + bar_thick, y + size / 2 + bar / 2);
    Rectangle(dc, x + size - bar_thick, y + size / 2 - bar / 2, x + size - 1,
              y + size / 2 + bar / 2);
    SelectObject(dc, yellow_brush);
    Rectangle(dc, x + size / 2 - bar / 2 + 2, y + 2, x + size / 2 + bar / 2 - 2, y + bar_thick);
    Rectangle(dc, x + size / 2 - bar / 2 + 2, y + size - bar_thick, x + size / 2 + bar / 2 - 2,
              y + size - 2);
    Rectangle(dc, x + 2, y + size / 2 - bar / 2 + 2, x + bar_thick, y + size / 2 + bar / 2 - 2);
    Rectangle(dc, x + size - bar_thick, y + size / 2 - bar / 2 + 2, x + size - 2,
              y + size / 2 + bar / 2 - 2);

    // 4 个导入桶：四角场外，深红色桶身 + 黑色桶盖。
    const int bucket = size / 24;
    auto draw_bucket = [&](int bx, int by) {
        SelectObject(dc, bucket_brush);
        Rectangle(dc, bx, by, bx + bucket, by + bucket);
        SelectObject(dc, black_brush);
        Rectangle(dc, bx, by, bx + bucket, by + 4);
    };
    draw_bucket(x - 8, y - 8);
    draw_bucket(x + size - bucket + 8, y - 8);
    draw_bucket(x - 8, y + size - bucket + 8);
    draw_bucket(x + size - bucket + 8, y + size - bucket + 8);

    // 4 组红/黄/蓝锥堆，对齐到 2ft 地垫网格交点。
    const double clusters[4][2] = {{1.0 / 6.0, 1.0 / 6.0},
                                   {2.0 / 6.0, 2.0 / 6.0},
                                   {4.0 / 6.0, 4.0 / 6.0},
                                   {5.0 / 6.0, 5.0 / 6.0}};
    for (const auto &cluster : clusters) {
        const int cx = px(cluster[0]);
        const int cy = py(cluster[1]);
        SelectObject(dc, black_brush);
        Ellipse(dc, cx - 12, cy - 12, cx + 12, cy + 12);
        SelectObject(dc, red_brush);
        Ellipse(dc, cx - 11, cy - 4, cx + 1, cy + 8);
        SelectObject(dc, blue_brush);
        Ellipse(dc, cx - 1, cy - 11, cx + 11, cy + 1);
        SelectObject(dc, yellow_brush);
        Ellipse(dc, cx - 5, cy - 11, cx + 5, cy - 1);
    }

    // 三台对方机器人占位，和真实场地的四台起始车对应。
    auto draw_ghost_robot = [&](double nx, double ny, double heading_deg) {
        const double rad = heading_deg * 3.141592653589793 / 180.0;
        const double c = std::cos(rad);
        const double s = std::sin(rad);
        const double half_l = size * 0.030;
        const double half_w = size * 0.022;
        const double local[4][2] = {
            {half_l, half_w}, {-half_l, half_w}, {-half_l, -half_w}, {half_l, -half_w}};
        POINT points[4];
        for (int i = 0; i < 4; ++i) {
            const double wx = px(nx) + local[i][0] * c - local[i][1] * s;
            const double wy = py(ny) + local[i][0] * s + local[i][1] * c;
            points[i].x = static_cast<int>(wx);
            points[i].y = static_cast<int>(wy);
        }
        SelectObject(dc, ghost_brush);
        SelectObject(dc, black_pen);
        Polygon(dc, points, 4);
    };
    draw_ghost_robot(0.56, 0.10, 180.0);
    draw_ghost_robot(0.10, 0.42, 90.0);
    draw_ghost_robot(0.90, 0.46, -90.0);

    // 轨迹。
    if (m_trail_count > 1) {
        HPEN trail_pen = CreatePen(PS_SOLID, 2, RGB(80, 160, 220));
        SelectObject(dc, trail_pen);
        for (int i = 0; i < m_trail_count; ++i) {
            int sx = 0;
            int sy = 0;
            worldToScreen(m_trail[i].x, m_trail[i].y, x, y, size, &sx, &sy);
            if (i == 0) {
                MoveToEx(dc, sx, sy, nullptr);
            } else {
                LineTo(dc, sx, sy);
            }
        }
        DeleteObject(trail_pen);
    }

    // 当前机器人。
    if (m_field.valid) {
        const double heading = m_field.heading_deg * 3.141592653589793 / 180.0;
        const double c = std::cos(heading);
        const double s = std::sin(heading);
        const double half_l = 0.23;
        const double half_w = 0.18;
        const double local[4][2] = {
            {half_l, half_w}, {-half_l, half_w}, {-half_l, -half_w}, {half_l, -half_w}};
        POINT points[4];
        for (int i = 0; i < 4; ++i) {
            const double wx = m_field.x_m + local[i][0] * c - local[i][1] * s;
            const double wy = m_field.y_m + local[i][0] * s + local[i][1] * c;
            int sx = 0;
            int sy = 0;
            worldToScreen(wx, wy, x, y, size, &sx, &sy);
            points[i].x = sx;
            points[i].y = sy;
        }
        SelectObject(dc, blue_brush);
        SelectObject(dc, black_pen);
        Polygon(dc, points, 4);
        int hx = 0;
        int hy = 0;
        worldToScreen(m_field.x_m + 0.32 * c, m_field.y_m + 0.32 * s, x, y, size, &hx, &hy);
        MoveToEx(dc, (points[0].x + points[3].x) / 2, (points[0].y + points[3].y) / 2, nullptr);
        LineTo(dc, hx, hy);
    }

    HFONT font = CreateFontA(16, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                             OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                             DEFAULT_PITCH | FF_SWISS, "Segoe UI");
    HGDIOBJ old_font = SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);
    drawText(dc, x, y - 32, size, 24, "V5RC OVERRIDE  12' x 12'  |  AUTO 15s  |  DRIVE 1:45",
             RGB(225, 228, 230));
    SelectObject(dc, old_font);
    DeleteObject(font);

    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(field_brush);
    DeleteObject(wall_brush);
    DeleteObject(black_brush);
    DeleteObject(yellow_brush);
    DeleteObject(blue_brush);
    DeleteObject(red_brush);
    DeleteObject(white_brush);
    DeleteObject(ghost_brush);
    DeleteObject(bucket_brush);
    DeleteObject(grid_pen);
    DeleteObject(wall_pen);
    DeleteObject(white_pen);
    DeleteObject(red_pen);
    DeleteObject(blue_pen);
    DeleteObject(black_pen);
}

void ControllerUi::paintController(HDC dc) {
    const RECT canvas{0, 0, kControllerWidth, kControllerHeight};
    HBRUSH background = CreateSolidBrush(RGB(18, 20, 26));
    FillRect(dc, &canvas, background);
    DeleteObject(background);

    SetBkMode(dc, TRANSPARENT);
    HFONT font = CreateFontA(18, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                             OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                             DEFAULT_PITCH | FF_SWISS, "Segoe UI");
    HGDIOBJ old_font = SelectObject(dc, font);

    drawText(dc, 0, 14, kControllerWidth, 30, "VEX V5 CONTROLLER", RGB(235, 238, 245));
    drawText(dc, 0, 44, kControllerWidth, 22,
             "DRIVE: arrows 8-way   TUNE: arrows select, W/S/Q/E jog", RGB(140, 150, 165));

    HBRUSH body = CreateSolidBrush(RGB(42, 46, 58));
    HPEN body_pen = CreatePen(PS_SOLID, 3, RGB(70, 76, 92));
    HGDIOBJ old_brush = SelectObject(dc, body);
    HGDIOBJ old_pen = SelectObject(dc, body_pen);
    RoundRect(dc, 70, 80, 610, 430, 70, 70);
    RoundRect(dc, 45, 150, 145, 390, 50, 50);
    RoundRect(dc, 535, 150, 635, 390, 50, 50);
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(body);
    DeleteObject(body_pen);

    drawButton(dc, 130, 95, 110, 32, "L1", RGB(54, 92, 160), isDown(io::Digital::L1));
    drawButton(dc, 250, 95, 110, 32, "L2", RGB(54, 92, 160), isDown(io::Digital::L2));
    drawButton(dc, 330, 95, 110, 32, "R1", RGB(160, 70, 70), isDown(io::Digital::R1));
    drawButton(dc, 450, 95, 110, 32, "R2", RGB(160, 70, 70), isDown(io::Digital::R2));

    HBRUSH stick = CreateSolidBrush(RGB(62, 67, 82));
    HPEN stick_pen = CreatePen(PS_SOLID, 2, RGB(105, 112, 130));
    HGDIOBJ old_brush2 = SelectObject(dc, stick);
    HGDIOBJ old_pen2 = SelectObject(dc, stick_pen);
    Ellipse(dc, 95, 190, 205, 300);
    Ellipse(dc, 475, 190, 585, 300);
    SelectObject(dc, old_brush2);
    SelectObject(dc, old_pen2);
    DeleteObject(stick);
    DeleteObject(stick_pen);

    drawText(dc, 475, 215, 110, 35, "NO", RGB(150, 155, 170));
    drawText(dc, 475, 245, 110, 25, "JOYSTICK", RGB(150, 155, 170));

    // 左摇杆 8 方向：DRIVE 用方向键，TUNE 用 W/S/Q/E 点动。
    const int stick_cx = 150;
    const int stick_cy = 245;
    const int display_left_y = m_tune_mode ? m_jog_left_y : m_analog_left_y;
    const int display_right_x = m_tune_mode ? m_jog_right_x : m_analog_right_x;
    const int stick_dx = display_right_x * 28 / 127;
    const int stick_dy = -display_left_y * 28 / 127;
    HBRUSH knob_brush = CreateSolidBrush(RGB(125, 145, 190));
    HPEN knob_pen = CreatePen(PS_SOLID, 2, RGB(30, 34, 44));
    SelectObject(dc, knob_brush);
    SelectObject(dc, knob_pen);
    Ellipse(dc, stick_cx + stick_dx - 18, stick_cy + stick_dy - 18, stick_cx + stick_dx + 18,
            stick_cy + stick_dy + 18);
    DeleteObject(knob_brush);
    DeleteObject(knob_pen);

    drawButton(dc, 125, 315, 50, 38, "^", RGB(78, 84, 100), isDown(io::Digital::Up));
    drawButton(dc, 125, 355, 50, 38, "v", RGB(78, 84, 100), isDown(io::Digital::Down));
    drawButton(dc, 85, 355, 38, 38, "<", RGB(78, 84, 100), isDown(io::Digital::Left));
    drawButton(dc, 177, 355, 38, 38, ">", RGB(78, 84, 100), isDown(io::Digital::Right));

    drawCircle(dc, 510, 255, 24, "Y", RGB(190, 170, 40), isDown(io::Digital::Y));
    drawCircle(dc, 470, 300, 24, "X", RGB(40, 100, 190), isDown(io::Digital::X));
    drawCircle(dc, 550, 300, 24, "B", RGB(190, 50, 50), isDown(io::Digital::B));
    drawCircle(dc, 510, 345, 24, "A", RGB(40, 160, 80), isDown(io::Digital::A));

    drawText(dc, 80, 440, 520, 24,
             "A/B/X/Y face   1/2 L1/L2   3/4 R1/R2   I/J/K/L D-pad   ESC quit", RGB(160, 168, 182));
    drawText(dc, 80, 468, 520, 22, m_status0, RGB(120, 220, 160));
    drawText(dc, 80, 492, 520, 22, m_status1, RGB(190, 205, 225));

    SelectObject(dc, old_font);
    DeleteObject(font);
}

void ControllerUi::paint(HWND hwnd) {
    PAINTSTRUCT ps;
    HDC dc = BeginPaint(hwnd, &ps);

    RECT client;
    GetClientRect(hwnd, &client);

    // 设计画布，最后按实际 client 缩放，兼容 DPI。
    HDC canvas_dc = CreateCompatibleDC(dc);
    HBITMAP canvas_bmp = CreateCompatibleBitmap(dc, kWindowWidth, kWindowHeight);
    HGDIOBJ old_canvas = SelectObject(canvas_dc, canvas_bmp);

    HBRUSH background = CreateSolidBrush(RGB(12, 14, 19));
    const RECT canvas_rect{0, 0, kWindowWidth, kWindowHeight};
    FillRect(canvas_dc, &canvas_rect, background);
    DeleteObject(background);

    paintField(canvas_dc, kFieldX, kFieldY, kFieldSize);

    HDC controller_dc = CreateCompatibleDC(dc);
    HBITMAP controller_bmp = CreateCompatibleBitmap(dc, kControllerWidth, kControllerHeight);
    HGDIOBJ old_controller = SelectObject(controller_dc, controller_bmp);
    paintController(controller_dc);
    SetStretchBltMode(canvas_dc, HALFTONE);
    StretchBlt(canvas_dc, kControllerX, kControllerY, kControllerDrawW, kControllerDrawH,
               controller_dc, 0, 0, kControllerWidth, kControllerHeight, SRCCOPY);
    SelectObject(controller_dc, old_controller);
    DeleteObject(controller_bmp);
    DeleteDC(controller_dc);

    HFONT small = CreateFontA(15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                              OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                              DEFAULT_PITCH | FF_SWISS, "Segoe UI");
    HGDIOBJ old_font = SelectObject(canvas_dc, small);
    SetBkMode(canvas_dc, TRANSPARENT);

    drawText(canvas_dc, kControllerX, kControllerY + kControllerDrawH + 18, 300, 22,
             modeName(m_field.mode), RGB(120, 220, 160), DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    char line[96];
    std::snprintf(line, sizeof(line), "x=%.2f m  y=%.2f m  h=%.1f deg", m_field.x_m, m_field.y_m,
                  m_field.heading_deg);
    drawText(canvas_dc, kControllerX, kControllerY + kControllerDrawH + 42, 540, 22, line,
             RGB(190, 205, 225), DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    std::snprintf(line, sizeof(line), "v=%.2f m/s  I=%.0f mA", m_field.velocity_mps,
                  m_field.current_ma);
    drawText(canvas_dc, kControllerX, kControllerY + kControllerDrawH + 66, 540, 22, line,
             RGB(190, 205, 225), DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    std::snprintf(line, sizeof(line), "lift=%.2fm  clamp=%.0f  push=%.2f", m_field.lift_height_m,
                  m_field.clamp_angle_deg, m_field.push_position);
    drawText(canvas_dc, kControllerX, kControllerY + kControllerDrawH + 90, 540, 22, line,
             RGB(190, 205, 225), DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    drawText(canvas_dc, kControllerX, kControllerY + kControllerDrawH + 114, 540, 22, m_status0,
             RGB(120, 220, 160), DT_LEFT | DT_VCENTER | DT_SINGLELINE);
    drawText(canvas_dc, kControllerX, kControllerY + kControllerDrawH + 138, 540, 22, m_status1,
             RGB(190, 205, 225), DT_LEFT | DT_VCENTER | DT_SINGLELINE);

    SelectObject(canvas_dc, old_font);
    DeleteObject(small);

    SetStretchBltMode(dc, HALFTONE);
    StretchBlt(dc, 0, 0, client.right, client.bottom, canvas_dc, 0, 0, kWindowWidth, kWindowHeight,
               SRCCOPY);

    SelectObject(canvas_dc, old_canvas);
    DeleteObject(canvas_bmp);
    DeleteDC(canvas_dc);
    EndPaint(hwnd, &ps);
}

#else // !_WIN32

ControllerUi::ControllerUi()
    : m_hwnd(nullptr), m_open(false), m_analog_left_y(0), m_analog_right_x(0), m_arrow_up(false),
      m_arrow_down(false), m_arrow_left(false), m_arrow_right(false), m_tune_mode(false),
      m_jog_left_y(0), m_jog_right_x(0), m_jog_up(false), m_jog_down(false), m_jog_left(false),
      m_jog_right(false), m_trail_count(0) {
    for (bool &key : m_keys)
        key = false;
    m_status0[0] = '\0';
    m_status1[0] = '\0';
}

ControllerUi::~ControllerUi() = default;

bool ControllerUi::open() {
    return false;
}

void ControllerUi::pump() {}

void ControllerUi::render() {}

bool ControllerUi::isOpen() const {
    return false;
}

bool ControllerUi::isDown(io::Digital) const {
    return false;
}

bool ControllerUi::isConnected() const {
    return true;
}

int ControllerUi::analog(io::Analog) const {
    return 0;
}

bool ControllerUi::digital(io::Digital) const {
    return false;
}

void ControllerUi::setStatusText(const char *, const char *) {}

void ControllerUi::setFieldState(const FieldState &) {}

void ControllerUi::resetField() {}

void ControllerUi::setTuneMode(bool) {}

#endif

} // namespace sim
