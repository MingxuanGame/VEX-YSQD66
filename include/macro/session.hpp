#pragma once

#include <cstdint>

namespace macrorec {

// initialize 回调调用一次；设置唯一底盘/触屏，不驱动、不访问 SD，无等待循环。
void initialize();
// disabled / competition_initialize 进入时停止并取消旧状态；不丢弃完整待保存录制。
// 非阻塞，不进行机械归零。
void stop();
// 每 20ms 服务菜单/列表及待保存录制；只同步执行停止后的 SD IO，不驱动。
void idle_tick();
// opcontrol 进入时取消旧目标、清理触摸沿；屏幕保留菜单，非阻塞。
void begin_driver();
// 手动 20ms、宏 1ms 服务的唯一输出源；组件帧仍为 20ms，不含 delay，不另建任务。
// 倒计时/回放/检查点暂停/重置屏蔽手柄，续录开始重新采样，录制断联即取消。
// SD IO 只在输出已停止时执行，续录保留前缀并另存新宏。
void driver_tick();
// 返回下次服务周期（宏 1ms、手动 20ms）；宏提高唤醒频率以服务精确帧边界并让出 CPU。
std::uint32_t driver_period_ms();

} // namespace macrorec
