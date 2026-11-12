#pragma once

namespace auton {
// initialize 回调调用一次：初始化底盘配置并从 SD 载入参数，不占屏幕、不驱动。
// 同步文件 IO，可能短暂阻塞；失败保留未标定默认值，后续拒绝未配置路线。
void initialize();
// 禁用/模式切换时取消全部自动目标；非阻塞，不执行机械归零或松抓。
void stop();
// opcontrol 入口调用一次；停止旧动作、归还原屏幕并重置按键边沿，非阻塞。
void begin_opcontrol();
// 每 20ms 调用。完整 Konami Code 前只监听，返回 false，允许原底盘代码继续执行。
// 设置/单步期间接管底盘并返回 true；R2 保存退出，下一周期恢复正常驾驶。
// 保存 SD 时同步阻塞 IO，运动期间禁止保存；受场控禁用/模式退出约束。
bool settings_tick();
// autonomous 回调调用：优先读取 SD 参数，运行选中路线，14.8s 总预算包含文件读取。
// 阻塞但每 20ms 让出 CPU，禁用/手动切换/超时/硬件不就绪均停止；不接受手柄干预。
// 读取失败保留内存配置；缺少必要时间参数或硬件 TODO 未完成时拒绝运行。
void run_selected();
} // namespace auton
