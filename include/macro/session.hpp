#pragma once

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
// 20ms 调度手动/宏的唯一输出源；不含 delay，不另建任务。
// 倒计时/回放/重置屏蔽手柄，录制断联即取消；SD IO 只在输出已停止时执行。
void driver_tick();

} // namespace macrorec
