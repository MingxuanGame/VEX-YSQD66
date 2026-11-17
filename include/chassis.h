#pragma once

namespace chassis {
// 初始化唯一的四电机底盘，设置蓝齿轮/度编码器/刹车；非阻塞，不驱动车辆。
void initialize();
// 停止输出，恢复刹车模式并重置四电机编码器参考；非阻塞，不执行机械归零。
// 任一 SDK 操作失败返回 false，调用方必须保持停止并拒绝录放。
bool reset_state();
// 应用左右组件功率 [-127,127]，非阻塞；禁用/自动模式/SDK 失败返回 false 并停止。
// 手动及宏共用同一入口，调用方负责控制权与场控连接检查。
// 按实现中的 RIGHT_MOTOR_OFFSET_TO_LEFT 补偿功率幅值：正值加右侧，负值加左侧。
// 两侧预留补偿余量后限幅，正反转均适用；单侧零输入保持零，转向仍保留差速。
bool set_power(int left_power, int right_power);
// 非阻塞停止四个驱动电机，不执行机械归零。
void stop();
} // namespace chassis
