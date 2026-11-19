#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace lifting {

// 按 P9、P10 排列的电机轴角度（度），正向为上升。
using Position = std::array<double, 2>;

struct Limits {
    Position lower_deg;
    Position upper_deg;
};

// 初始化刹车模式并清除开机前的限位参考。
void initialize();

// 重读 SD 行程。在本次上电已有下端参考时立即恢复限位。
void load_limits();

bool has_saved_limits();

// 以现场确认的下端角度恢复 SD 上的行程。
bool use_saved_limits(const Position &lower_deg);

// 读取两路角度。若任一路失效，返回空并作废当前限位。
std::optional<Position> read_position();

// 返回当前相对下限的高度（两路电机平均，度）；未就绪时返回空。
std::optional<double> offset_deg();

// 取得本次开机已标定的上下限。
const std::optional<Limits> &limits();

// 刹车后保存并回读校验实测行程，成功后应用上下限。
bool set_limits(const Limits &measured);

// 手动模式下应用限位和临近端点的降速，返回实际下发的功率 [-127, 127]。
// 任一路到达该方向的限位即刹停整组；需要有效的标定和编码器读数。
int set_power(int power);

// 自动启用期间使用相同的编码器限位与降速。
int set_autonomous_power(int power);

// 自动赛闭环移动到相对下限的目标高度（度），基于 LemLib PID/Timer/ExitCondition。
// 若已有升降任务运行会先取消旧任务；未标定限位或传感器异常时返回 false。
bool move_to(double target_offset_deg, std::uint32_t timeout_ms, int max_power = 127,
             bool async = false);

// 自动赛按设定功率运行指定时长（毫秒），受限位、禁用与模式切换保护。
bool move_for(int power, std::uint32_t duration_ms, bool async = false);

// 等待当前异步升降动作完成。
void wait_until_done();

// 当前是否有自动升降任务在运行。
bool is_moving();

// 标定期间低功率点动，direction 的符号决定方向，零表示刹车。
// 调用方须持续按键并限制点动时长。
// 仅脱离场控且启用手动时允许，遇到编码器错误则刹停。
void jog(int direction);

// 取消后台升降动作并刹车停止，保留已标定的限位参考。
void stop();

} // namespace lifting
