#include "sim_clock.hpp"

namespace sim {

// 内部状态
static uint64_t g_virtual_time_us = 0; // 虚拟时间（微秒），为什么用微秒？为了极高精度
static PhysicsTickFunc g_physics_tick = nullptr;
static double g_time_scale = 1.0; // 时间倍率

void init_clock(uint32_t initial_time_ms) {
    g_virtual_time_us = static_cast<uint64_t>(initial_time_ms) * 1000ULL;
    g_physics_tick = nullptr;
    g_time_scale = 1.0;
}

uint32_t millis() {
    // 将高精度的微秒转换为毫秒返回
    return static_cast<uint32_t>(g_virtual_time_us / 1000ULL);
}

void register_physics_tick(PhysicsTickFunc func) {
    g_physics_tick = func;
}

void advance_time(double dt_sec) {
    // 乘上时间倍率
    double scaled_dt = dt_sec * g_time_scale;

    // 累加到微秒库中，避免浮点误差累积
    uint64_t dt_us = static_cast<uint64_t>(scaled_dt * 1000000.0);
    g_virtual_time_us += dt_us;

    // 推进物理引擎，把时间传递过去
    if (g_physics_tick) {
        g_physics_tick(scaled_dt);
    }
}

void delay(uint32_t ms) {
    double target_time_sec = (ms / 1000.0) * g_time_scale;

    // 核心逻辑：将延时切分为 1ms 的小块
    // 为什么是 1ms？因为 VEX 的真实控制循环通常也是 10ms~20ms，1ms 的物理步进能保证极高精度的碰撞模拟
    const double fixed_step = 0.001;

    while (target_time_sec > 0.000001) {
        double step = (target_time_sec < fixed_step) ? target_time_sec : fixed_step;

        // 推进物理引擎
        if (g_physics_tick) {
            g_physics_tick(step);
        }

        // 推进时间（注意这里直接加，不用再乘倍率，因为外层已经算过了）
        g_virtual_time_us += static_cast<uint64_t>(step * 1000000.0);
        target_time_sec -= step;
    }
}

void set_time_scale(double scale) {
    // 限制倍率范围，防止物理引擎积分爆炸
    if (scale < 0.1)
        scale = 0.1;
    if (scale > 50.0)
        scale = 50.0;
    g_time_scale = scale;
}

} // namespace sim
