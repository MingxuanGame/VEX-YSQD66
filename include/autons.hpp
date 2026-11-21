#pragma once

namespace autons {

// 自动赛唯一公开入口：读取屏幕所选预设并运行对应路线，结束或退出时安全停车。
// 仅在自动阶段执行；路线以 14.9 秒为上限，禁用或切换阶段时立即退出。
void run();

// 手动阶段的自动赛入口：按 A 触发后运行同一条所选路线，动作、超时与停车规则同 run()。
// 现场不接场控、直接调用 opcontrol() 时使用；返回后主循环继续手动控制。
void run_in_driver();

} // namespace autons
