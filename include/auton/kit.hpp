#pragma once

#include <array>
#include <cstdint>

namespace auton {

constexpr int kStepCount = 6;
constexpr int kRouteCount = 2;
constexpr int kFullVoltageMv = 12000;
// Override 2.1：15 秒自动，预留 200ms 停止余量（Match 定义，第 120-121 页；<SC1>）。
constexpr std::uint32_t kAutoBudgetMs = 14800;

enum class Param { Angle, Back, Forward, Lift, Lower, Action, ContactTimeout, TurnTimeout, Count };
constexpr int kParamCount = static_cast<int>(Param::Count);

struct Step {
    std::array<int, kParamCount> values{};
};

struct Route {
    std::array<Step, kStepCount> steps{};
};

struct Config {
    int selected_route = 0;
    std::array<Route, kRouteCount> routes{};
};

// 非阻塞纯函数；返回未标定的初始配置、字段名称/上限、该步是否使用字段。
Config defaults();
const char *param_name(Param param);
int param_max(Param param);
bool uses_param(int step, Param param);
bool valid_config(const Config &config);

// 双槽持久化。base_path 不含槽后缀；真机使用 /usd/auton，生成 .0/.1。
// load/save 同步阻塞文件 IO，不得在运动更新中调用；失败返回 false，不替换内存配置。
// 保存写入另一槽并回读校验，上一有效槽保留；断电恢复取最新有效版本。
class Store {
public:
    explicit Store(const char *base_path);
    bool load(Config &config);
    bool save(const Config &config);

private:
    std::array<char, 160> m_base{};
};

enum class Key { Up, Down, Left, Right, A, B, X, Y, L1, L2, R1, R2, Count };
constexpr int kKeyCount = static_cast<int>(Key::Count);

struct Buttons {
    std::array<bool, kKeyCount> down{};
    bool connected = true;
};

struct EditorEvent {
    bool save = false;
    bool test_step = false;
    bool stop = false;
};

// 20ms 调用 update；按下沿识别 ↑↑↓↓←→←→BA。右侧 Y/A 选位，X/B 加减。
// L1 切路线、L2 请求单步、R1 请求保存、R2 退出。事件仅为请求，不直接驱动硬件。
class Editor {
public:
    void reset(const Buttons &buttons);
    EditorEvent update(const Buttons &buttons, std::uint32_t now_ms, Config &config,
                       bool running = false);
    bool active() const;
    int step() const;
    Param param() const;
    int digit() const;
    bool dirty() const;
    void mark_saved();

private:
    Buttons m_previous{};
    bool m_active = false;
    bool m_dirty = false;
    int m_sequence = 0;
    int m_step = 0;
    int m_param = 0;
    int m_digit = 0;
    std::uint32_t m_last_key_ms = 0;
};

enum class Action { Toggle, Open, Close };

// 适配器必须非阻塞。ready 只在设备映射、方向、限位和反馈标定完成后返回 true。
// heading 为向右为正的累积角度（度）；contact 返回有效性并输出已去抖的撞桩状态。
// action 启动机构动作并返回是否接受；lift_voltage 必须自行执行物理限位，失败返回 false。
// stop 停底盘/升降并取消机构目标，不能在禁用时打开夹爪或作机械归零。
class Hardware {
public:
    virtual ~Hardware() = default;
    virtual bool ready() const = 0;
    virtual bool heading(double &angle_deg) const = 0;
    virtual bool contact(bool &pressed) const = 0;
    virtual bool drive_voltage(int left_mv, int right_mv) = 0;
    virtual bool lift_voltage(int voltage_mv) = 0;
    virtual bool action(Action action) = 0;
    virtual void stop() = 0;
};

enum class State { Idle, Running, Done, Fault, Cancelled };
enum class Phase { Turn, Toggle, ReturnTurn, Back, Forward, Contact, Lower, Release, Grab };

// 所有时间以本次 start 为基准；20ms 调用 update，超时/禁用/失效反馈立即 stop。
// start 拷贝路线快照；single_step=true 只执行指定的 0..5 步。均非阻塞，返回起跑成功与否。
class Runner {
public:
    explicit Runner(Hardware &hardware);
    bool start(const Route &route, std::uint32_t now_ms, int first_step = 0,
               bool single_step = false);
    void update(std::uint32_t now_ms, bool allowed);
    void cancel();
    State state() const;
    Phase phase() const;
    int step() const;
    const char *status() const;

private:
    int value(Param param) const;
    bool enter(Phase phase, std::uint32_t now_ms);
    void begin_step(std::uint32_t now_ms);
    void next_phase(std::uint32_t now_ms);
    void finish_step(std::uint32_t now_ms);
    void fail(const char *reason);
    Hardware &m_hardware;
    Route m_route{};
    State m_state = State::Idle;
    Phase m_phase = Phase::Turn;
    int m_step = 0;
    bool m_single = false;
    bool m_lifting = false;
    bool m_arrived = false;
    std::uint32_t m_start_ms = 0;
    std::uint32_t m_phase_ms = 0;
    std::uint32_t m_lift_ms = 0;
    std::uint32_t m_last_update_ms = 0;
    std::uint32_t m_settle_ms = 0;
    bool m_settling = false;
    double m_target_deg = 0;
    const char *m_status = "IDLE";
};

struct MotorReading {
    int port = 0;
    bool valid = false;
    int voltage_mv = 0;
    double velocity_rpm = 0;
    double position_deg = 0;
    int current_ma = 0;
};

// 16 行 ASCII 文本，真机用小字体显示，避免依赖未安装的中文字体。
using Screen = std::array<std::array<char, 80>, 16>;
// 非阻塞，显示选中位、所选路线所有六步变量和四个驱动电机实际反馈；无效反馈显示 ERR。
Screen render(const Config &config, const Editor &editor, const Runner &runner,
              const std::array<MotorReading, 4> &motors, const char *storage_status);

} // namespace auton
