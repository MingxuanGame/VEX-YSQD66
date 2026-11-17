#pragma once

namespace claw {

// 初始化夹爪电机；开环，只设置刹车，不动作，简单。
void initialize();

// 张开/闭合；调用后持续输出，直到 stop() 或下一次 set_power()。
void open();
void close();

// 手动功率，范围 -127..127。
void set_power(int power);

// 刹车停止。
void stop();

} // namespace claw
