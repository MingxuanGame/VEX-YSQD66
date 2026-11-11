#include "io/input_mapping.hpp"
#include "vxmath/filters.hpp"
#include "vxmath/math_utils.hpp"

namespace io {

bool comboPressed(const InputState &in, bool (*btn1)(const InputState &),
                  bool (*btn2)(const InputState &)) {
    return btn1(in) && btn2(in);
}

int processJoystick(int raw, int deadzone, double expo) {
    // 1. 先限幅，防止上层传入异常值。
    int v = vxmath::clampVal(raw, -127, 127);
    int dz = vxmath::clampVal(deadzone, 0, 126);

    // 2. 死区内归零，防止摇杆中位漂移导致底盘原地抽搐。
    int magnitude = (v < 0) ? -v : v;
    if (magnitude <= dz)
        return 0;

    // 3. 把死区外的剩余行程重新映射到 1~127，避免越过死区时输出跳变。
    int sign = (v < 0) ? -1 : 1;
    int scaled = (magnitude - dz) * 127 / (127 - dz);

    // 4. 过指数曲线，让近距离操作更柔和。
    return vxmath::expoCurve(sign * scaled, expo);
}

} // namespace io
