#pragma once

namespace claw {

// 初始化 Brain 三线 E 口气阀为高电平夹紧，并启用启动锁；非阻塞。
void initialize();

// 解除启动锁（仅在未禁用且当前处于锁定状态时生效），不直接改变气阀输出；仅首次解锁返回 true。
bool unlock_startup();

// 立即设置夹爪为松开（低电平）或夹紧（高电平）并保持；非阻塞。启动锁生效时 open() 无效。
void open();
void close();

// 复位夹爪输出：已解锁时放气松开（低电平），未解锁时维持夹紧（高电平）；非阻塞，可在禁用或模式切换时调用。
void stop();

} // namespace claw
