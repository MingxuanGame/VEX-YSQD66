#pragma once

namespace claw {

// 初始化 Brain 三线 D 口气阀，默认低电平（放气松开）。
void initialize();

// 手动启用时设置夹爪开合状态并保持；下发后立即返回。
void open();
void close();

// 自动启用时设置夹爪开合状态并保持；下发后立即返回。
void open_autonomous();
void close_autonomous();

// 撤销气阀输出并放气松开，可在禁用和模式交接时调用。
void stop();

} // namespace claw
