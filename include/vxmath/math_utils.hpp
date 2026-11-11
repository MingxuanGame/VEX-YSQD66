#pragma once

/**
 * @file math_utils.hpp
 * @brief 基础数学工具声明。实现放在 src/vxmath/math_utils.cpp。
 */

namespace vxmath {

template <class T> T clampVal(T val, T min, T max) {
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

bool isValidDouble(double v);
bool isValidAngle(double angle);
bool isValidGyroRate(double rate);
double restrictAngle(double angle);
double getError(double start, double target);
double normalizeAngle180(double angle);
double safeGetError(double start, double target, double fallback = 0.0);
bool angleInTolerance(double error, double tol = 1.0);
double lerp(double a, double b, double t);

template <class T> int sign(T v) {
    return (v > 0) - (v < 0);
}

} // namespace vxmath
