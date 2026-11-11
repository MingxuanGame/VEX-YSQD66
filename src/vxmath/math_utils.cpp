#include "vxmath/math_utils.hpp"
#include <cmath>

namespace vxmath {

bool isValidDouble(double v) {
    return std::isfinite(v);
}

bool isValidAngle(double angle) {
    return std::isfinite(angle) && std::fabs(angle) < 10000.0;
}

bool isValidGyroRate(double rate) {
    // V5 IMU 最大量程 2000°/s，超过 2500 绝对异常
    return std::isfinite(rate) && std::fabs(rate) <= 2500.0;
}

double restrictAngle(double angle) {
    angle = std::fmod(angle, 360.0);
    if (angle < 0)
        angle += 360.0;
    return angle;
}

double getError(double start, double target) {
    double diff = target - start;
    diff = std::fmod(diff + 540.0, 360.0) - 180.0;
    return diff;
}

double normalizeAngle180(double angle) {
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0)
        angle += 360.0;
    return angle - 180.0;
}

double safeGetError(double start, double target, double fallback) {
    if (!isValidAngle(start) || !isValidAngle(target))
        return fallback;
    return getError(start, target);
}

bool angleInTolerance(double error, double tol) {
    return std::fabs(error) < tol;
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

} // namespace vxmath
