#pragma once
#include "tuning/params_manager.hpp"

/**
 * @file param_validate.hpp
 * @brief 参数合法性校验。
 *        因为比赛现场经常有手滑把 Kp 调成负数，或者把电流阈值调到 0 的情况，
 *        独立的校验工具可以在初始化时拦截这些“致命”参数。
 */

namespace tuning {

class ParamValidate {
public:
    /**
     * @brief 扫描所有参数，检查是否越界或存在非法值。
     * @return 非法的参数数量（0 为全部合法）。
     * @note 发现非法值会恢复到该参数的默认值；建议在 initialize() 和 autonomous() 开始前调用。
     */
    static int validateAll(ParamsManager &pm);
};

} // namespace tuning
