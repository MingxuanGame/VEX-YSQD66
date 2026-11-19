#pragma once

namespace claw {

// 初始化夹爪电机；开环，只设置刹车，不动作。
void initialize();

// 手动张开/闭合；调用后持续输出，直到 stop() 或下一次 set_power()。
void open();
void close();

// 手动功率，范围 -127..127；禁用或自动阶段下保持停止。
void set_power(int power);

// 自动赛专用功率与开合接口，仅在自动启用期间输出。
void set_autonomous_power(int power);
void open_autonomous();
void close_autonomous();

// 刹车停止。
void stop();

} // namespace claw
