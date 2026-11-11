#pragma once
#include <cmath>
#include <cstdint>

namespace sim {

/**
 * @brief 轻量确定性随机数发生器。
 *
 * 仿真不能依赖全局 rand()：同一场景要能复现，两个实例也不能共享 Box-Muller
 * 的静态缓存。xorshift32 足够快、无堆分配、可显式播种。
 */
class XorShift32 {
public:
    explicit XorShift32(uint32_t initial_seed = 0x9E3779B9u) {
        seed(initial_seed);
    }

    void seed(uint32_t value) {
        m_state = (value != 0u) ? value : 0x9E3779B9u;
        m_has_spare = false;
        m_spare = 0.0;
    }

    uint32_t next() {
        uint32_t x = m_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        m_state = x;
        return x;
    }

    double uniform01() {
        // 取高 24 位，避免低位质量差；加 epsilon 防止 log(0)。
        return (static_cast<double>(next() >> 8) + 1.0) / 16777217.0;
    }

    double gaussian(double mean, double stddev) {
        if (stddev <= 0.0)
            return mean;
        if (m_has_spare) {
            m_has_spare = false;
            return mean + m_spare * stddev;
        }

        const double u1 = uniform01();
        const double u2 = uniform01();
        const double radius = std::sqrt(-2.0 * std::log(u1));
        const double angle = 2.0 * 3.141592653589793 * u2;
        m_spare = radius * std::sin(angle);
        m_has_spare = true;
        return mean + radius * std::cos(angle) * stddev;
    }

private:
    uint32_t m_state = 0x9E3779B9u;
    bool m_has_spare = false;
    double m_spare = 0.0;
};

} // namespace sim
