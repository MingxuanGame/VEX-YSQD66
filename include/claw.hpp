#pragma once

namespace claw {

// 初始化 Brain 三线 E 口气阀，默认高电平夹紧，启动锁生效。
void initialize();

// 启用时运行自动路线或手动按 R1 时调用。仅首次解锁返回 true，调用不移动夹爪。
bool unlock_startup();

// 启用时设置夹爪开合状态并保持，调用立即返回。启动锁生效时忽略松开命令。
void open();
void close();

// 解锁后撤销气阀输出并放气松开。锁定期间保持夹紧，可在禁用和模式交接时调用。
void stop();

} // namespace claw
