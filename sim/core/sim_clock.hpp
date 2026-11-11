#pragma once
#include <cstdint>

/**
 * @file sim_clock.hpp
 * @brief 虚拟时钟引擎。
 *        接管所有的时间相关调用（millis, delay）。
 *        通过替换真实时间，使得我们在电脑上跑 15 秒的自动赛只需要几毫秒，并且能驱动物理引擎更新。
 */
namespace sim {

// 物理引擎的更新回调：告诉系统过了多少秒，让物理世界演化
using PhysicsTickFunc = void (*)(double dt_sec);

/**
 * @brief 初始化时钟（启动时调用一次）
 * @param initial_time_ms 初始系统时间
 */
void init_clock(uint32_t initial_time_ms = 0);

/**
 * @brief 获取当前的虚拟毫秒数
 * @return 模拟的 V5 运行时间（毫秒）
 * @note 内部使用微秒级高精度累加，避免因为浮点误差导致 millis() 越跑越慢。
 */
uint32_t millis();

/**
 * @brief 注册物理引擎步进函数
 * @param func 物理步进回调
 * @note 每次虚拟时间推进时，都会自动调用该函数。
 */
void register_physics_tick(PhysicsTickFunc func);

/**
 * @brief 虚拟延时（不占用真实 CPU 时间）
 * @param ms 延时的毫秒数
 * @note 内部会将延时切分成固定步长（如 1ms），不断推进物理引擎，直到时间结束。
 */
void delay(uint32_t ms);

/**
 * @brief 手动推进虚拟时间
 * @param dt_sec 推进的秒数
 * @note 供主循环或场景驱动器使用。
 */
void advance_time(double dt_sec);

/**
 * @brief 设置时间倍率
 * @param scale 倍率（1.0 为真实时间，10.0 为十倍速。注意：倍率过高可能导致物理引擎的计算步长过大而失真）
 */
void set_time_scale(double scale);

} // namespace sim
