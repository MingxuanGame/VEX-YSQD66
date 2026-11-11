#pragma once

/**
 * @file filters.hpp
 * @brief 信号处理声明。非模板实现放在 src/vxmath/filters.cpp。
 */

namespace vxmath {

template <int N> class MovingAverage {
public:
    static_assert(N > 0, "MovingAverage 的窗口大小必须大于 0");

    MovingAverage() : m_idx(0), m_count(0), m_sum(0.0) {
        for (int i = 0; i < N; ++i)
            m_buf[i] = 0.0;
    }

    double update(double v) {
        if (m_count < N) {
            m_buf[m_idx] = v;
            m_sum += v;
            m_count++;
        } else {
            m_sum -= m_buf[m_idx];
            m_buf[m_idx] = v;
            m_sum += v;
        }
        m_idx = (m_idx + 1) % N;
        return m_count > 0 ? m_sum / m_count : 0.0;
    }

    void reset() {
        m_idx = 0;
        m_count = 0;
        m_sum = 0.0;
        for (int i = 0; i < N; ++i)
            m_buf[i] = 0.0;
    }

private:
    double m_buf[N];
    int m_idx;
    int m_count;
    double m_sum;
};

class LowPass {
public:
    explicit LowPass(double alpha);
    double update(double v);
    void reset();

private:
    double m_alpha;
    double m_last;
    bool m_init;
};

int applyDeadzone(int v, int deadzone);
int expoCurve(int v, double expo);

class Hysteresis {
public:
    Hysteresis(double threshold, double hysteresis);
    bool update(double v);
    void reset();

private:
    double m_threshold;
    double m_hys;
    bool m_state;
};

} // namespace vxmath
