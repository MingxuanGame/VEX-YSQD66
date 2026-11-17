#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace macrorec {

constexpr std::uint32_t kPeriodMs = 20;
constexpr std::uint32_t kCountdownMs = 3000;
// Override 2.1 Match 定义（英文第 120-121 页）：15s，最后 500ms 仅清理软件/停止。
constexpr std::uint32_t kRecordMs = 14500;
constexpr std::uint32_t kResetMs = 500;
constexpr std::size_t kFrameCount = kRecordMs / kPeriodMs;
constexpr std::uint16_t kButtonMask = 0x0fff;
constexpr std::uint16_t kCheckpointButton = 1U << 8; // L1，按下沿标记当前帧结束处。

// 四轴原始值 [-127,127]；按键位序见 docs/macro-protocol.md。仅 connected 不写入文件。
struct Input {
    int left_x = 0;
    int left_y = 0;
    int right_x = 0;
    int right_y = 0;
    std::uint16_t buttons = 0;
    bool connected = false;
};

// 对组件的最终指令，单位为 PROS move() 功率 [-127,127]；不含电机物理端口。
struct Command {
    int chassis_left_power = 0;
    int chassis_right_power = 0;
};

struct Frame {
    Input input{};
    Command command{};
    bool checkpoint = false; // 标记本帧执行完毕（t_ms + 20ms）的边界，与输入电平独立。
};

using Clip = std::array<Frame, kFrameCount>;

// 非阻塞纯函数：沿用手动 Arcade 方向并等比限幅；断联返回零输出。
Command map_input(const Input &input);
// 非阻塞检查轴、按键位及组件输出范围；回放允许组件指令与原始输入不同，便于微调。
bool valid_frame(const Frame &frame);

// 组件适配器必须非阻塞；apply 失败返回 false，stop 只停输出/取消目标，不机械归零。
class Hardware {
public:
    virtual ~Hardware() = default;
    virtual bool apply(const Command &command) = 0;
    virtual void stop() = 0;
    // 停止所有组件、取消旧目标、清除切换/积分状态及编码器参考；不移动机构。
    // 非阻塞；任一组件失败返回 false，录放必须退出，不能继续输出。
    virtual bool reset() = 0;
    // 读取输出时钟（ms）；真机必须返回实时单调时钟，默认供纯逻辑测试使用传入时间。
    virtual std::uint32_t time_ms(std::uint32_t fallback_ms) const;
};

enum class State {
    Idle,
    Countdown,
    Recording,
    Replaying,
    CheckpointPause,
    Resetting,
    Complete,
    Cancelled,
    Fault
};

// 单个回调独占本对象及组件写权限；无后台任务。宏期间每 1ms 服务，输出帧周期为 20ms。
// 提前服务只等待；帧边界或 SDK 写入完成时间迟到则立即 Fault，不补帧、不拉长后续周期。
// 倒计时全屏蔽手柄，录制 725 帧后停止 500ms；跳帧、禁用、录制断联或输出失败立即停止。
// 回放只使用 command，忽略手柄，依赖 allowed 实现场控/模式检查；均非阻塞。
class Engine {
public:
    explicit Engine(Hardware &hardware);
    // 忙碌或组件重置失败返回 false；准备及首帧前重置，动作结束再重置一次。
    // 3s 后首个调度点开始，录制耗时约 3000+14500+500ms。
    bool start_recording(std::uint32_t now_ms);
    // 完整校验并复制快照；失败不启动，成功后同样倒计时 3000ms。
    bool start_replay(const Clip &clip, std::uint32_t now_ms);
    // 从原起点回放前 frame_count 帧，到检查点后停车 3s，再续录至 725 帧。
    // frame_count 必须为带检查点的帧结束边界；725 表示只完整回放，不续录。
    // 非阻塞，校验失败不启动；保留前缀并丢弃旧后缀，完整续录由调用方另存新宏。
    bool start_from_checkpoint(const Clip &clip, std::size_t frame_count, std::uint32_t now_ms);
    // allowed=false 立即故障停止；input 仅在录制阶段使用，无返回值，不访问 SD。
    void update(std::uint32_t now_ms, bool allowed, const Input &input = {});
    // 取消目标并重置组件及录放边沿状态，不保存不完整录制；失败进入 Fault，无阻塞等待。
    void cancel();
    // 以下为非阻塞查询：是否需要继续周期更新、当前阶段、是否会产生完整录制（含续录）。
    bool busy() const;
    State state() const;
    bool recording() const;
    // 首帧在准备/暂停结束后采样；后续录制帧在边界前最多 2ms 请求一次缓存采样。
    // 前缀回放不读取实时输入；调用方需缓存到对应输出帧下发后再丢弃。
    bool needs_input(std::uint32_t now_ms) const;
    // 当前输出距下一帧边界剩余的 ms；无活动输出或已经到期返回 0，非阻塞。
    std::uint32_t frame_wait_ms(std::uint32_t now_ms) const;
    // 已成功应用的帧数；clip 引用只在帧数为 725 时代表完整录制，下一次录放会改写它。
    std::size_t frame_count() const;
    const Clip &clip() const;
    // 本次阶段经过时间（ms），倒计时/检查点暂停/重置从各自阶段开始计时。
    std::uint32_t elapsed_ms(std::uint32_t now_ms) const;
    // 返回静态 ASCII 状态/故障说明，不分配内存，不阻塞。
    const char *status() const;

private:
    bool reset_hardware();
    bool stop_frame(std::uint32_t now_ms);
    bool start(std::uint32_t now_ms, bool recording);
    void fail(const char *reason);
    Hardware &m_hardware;
    Clip m_clip{};
    State m_state = State::Idle;
    bool m_recording = false;
    bool m_resuming = false;
    bool m_checkpoint_button_down = false;
    std::size_t m_resume_frame_count = 0;
    std::uint32_t m_pause_started_ms = 0;
    std::uint32_t m_started_ms = 0;
    std::uint32_t m_last_update_ms = 0;
    std::uint32_t m_last_frame_ms = 0;
    bool m_frame_active = false;
    std::size_t m_frame_count = 0;
    const char *m_status = "READY";
};

} // namespace macrorec
