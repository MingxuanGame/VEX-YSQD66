#pragma once

/**
 * @file motion_tuning.hpp
 * @brief 把速度环 PID 参数注册到 ParamsManager，并绑定到具体 MotionEngine。
 */

namespace tuning {
class ParamsManager;
class TunableParam;
} // namespace tuning

namespace motion {

class MotionEngine;

class MotionTuning {
public:
    explicit MotionTuning(MotionEngine &engine);

    /**
     * @brief 注册 vel_kp / vel_ki / vel_kd。
     * @return 成功注册的数量。
     */
    int registerAll(tuning::ParamsManager &pm);

    /**
     * @brief 把已注册参数绑定到当前 MotionEngine。
     * @return 成功绑定的数量。
     */
    int bindAll(tuning::ParamsManager &pm);

private:
    static void applyKp(const tuning::TunableParam &p, void *context);
    static void applyKi(const tuning::TunableParam &p, void *context);
    static void applyKd(const tuning::TunableParam &p, void *context);

    MotionEngine *m_engine;
};

} // namespace motion
