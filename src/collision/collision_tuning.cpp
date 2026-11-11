#include "collision/collision_tuning.hpp"
#include "tuning/params_manager.hpp"

/**
 * @file collision_tuning.cpp
 * @brief 碰撞模块的调参绑定实现。
 *        这里将阈值抽出来挂到遥控器调参系统，方便赛场上不用连电脑就能改参数。
 */

namespace collision {

int CollisionTuning::registerAll(tuning::ParamsManager &pm) {
    int count = 0;

    // 统一放在 CollisionGroup 分组下，方便在遥控器上按 L1/R1 切页查找。
    auto group = tuning::ParamGroup::CollisionGroup;

    // addParam 在重名或表满时返回 -1，不能无脑 count++。
    auto add = [&](const char *name, tuning::IntDoubleType value, tuning::IntDoubleType lo,
                   tuning::IntDoubleType hi, tuning::ParamType type, int min_digit, int max_digit) {
        if (pm.addParam(name, value, lo, hi, type, group, min_digit, max_digit) >= 0) {
            ++count;
        }
    };

    // 注册碰撞触发阈值
    add("coll_current", 2500, 500, 8000, tuning::ParamType::Int, 0, 2);
    add("coll_gyro", 15.0, 0.0, 200.0, tuning::ParamType::Double, -1, 1);
    add("coll_var", 200000, 0, 1000000, tuning::ParamType::Int, 0, 2);

    // 注册锥专属阈值
    add("cone_c_min", 1200, 0, 5000, tuning::ParamType::Int, 0, 2);
    add("cone_c_max", 2500, 0, 5000, tuning::ParamType::Int, 0, 2);

    // 注册恢复动作参数
    add("esc_dist", 3, 1, 10, tuning::ParamType::Int, 0, 1);

    return count;
}

} // namespace collision
