#pragma once

#include <cstdint>

namespace macrorec {

// 初始化底盘与 Brain 触屏界面；非阻塞，不访问 SD 卡，不驱动车辆。
void initialize();

// disabled / competition_initialize 进入时停止输出并取消未完成录放（保留已采齐的待保存录制）。
void stop();

// 禁用期周期调度（20ms），只服务菜单与待保存 SD 写入，不输出电机功率。
void idle_tick();

// opcontrol 进入时重置输出与触摸状态。
void begin_driver();

// opcontrol 主循环调度：管理正常驾驶、宏录制/回放、触屏界面与 SD 存盘。
void driver_tick();

// 返回建议的调度周期（ms）：宏运行时 1ms 以贴近 20ms 帧边界，平时 20ms。
std::uint32_t driver_period_ms();

} // namespace macrorec
