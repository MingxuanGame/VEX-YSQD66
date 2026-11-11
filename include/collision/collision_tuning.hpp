#pragma once

/**
 * @file collision_tuning.hpp
 * @brief 碰撞模块的调参绑定。独立成文件，确保 collision 核心逻辑不依赖 tuning 库。
 *        这样做的好处是，如果以后要移植代码或者做纯硬件测试，不需要连带编译调参系统。
 */

namespace tuning {
class ParamsManager;
}

namespace collision {

class CollisionTuning {
public:
    /**
     * @brief 将所有碰撞阈值自动注册到调参系统。
     * @param pm 调参管理器。
     * @return 成功注册的参数数量。
     * @note 在初始化阶段调用一次即可，比赛现场就能用遥控器实时修改阈值。
     */
    static int registerAll(tuning::ParamsManager &pm);
};

} // namespace collision
