#pragma once

namespace toggle {

// 初始化 Brain 三线 D 口气阀为低电平（抬起）；非阻塞。
void initialize();

// 立即设置拉翻色桶机构为抬起（低电平）或下放（高电平）；非阻塞，禁用状态下忽略写入。
void up();
void down();

// 立即将气阀复位为低电平（抬起）；非阻塞，可在禁用或模式切换时调用。
void stop();

} // namespace toggle
