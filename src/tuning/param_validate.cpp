#include "tuning/param_validate.hpp"
#include <cmath>
#include <variant>

/**
 * @file param_validate.cpp
 * @brief 参数合法性扫描。所有 variant 访问都先做类型检查，不会抛 bad_variant_access。
 */

namespace tuning {

int ParamValidate::validateAll(ParamsManager &pm) {
    int invalid_count = 0;

    for (int i = 0; i < pm.getSize(); ++i) {
        TunableParam *p = pm.getParam(i);
        if (!p)
            continue;

        bool invalid = false;
        const ParamType type = p->getParamType();

        if (type == ParamType::Double) {
            const double v = p->getDoubleValue();
            const IntDoubleType min_value = p->getMinValue();
            const IntDoubleType max_value = p->getMaxValue();
            const double *lo = std::get_if<double>(&min_value);
            const double *hi = std::get_if<double>(&max_value);
            if (!lo || !hi) {
                invalid = true;
            } else {
                // NaN/Inf 会让 PID 瞬间发疯；越界值同样不能进控制器。
                invalid = !std::isfinite(v) || v < *lo || v > *hi;
            }
        } else if (type == ParamType::Int) {
            const int v = p->getIntValue();
            const IntDoubleType min_value = p->getMinValue();
            const IntDoubleType max_value = p->getMaxValue();
            const int *lo = std::get_if<int>(&min_value);
            const int *hi = std::get_if<int>(&max_value);
            if (!lo || !hi) {
                invalid = true;
            } else {
                invalid = v < *lo || v > *hi;
            }
        } else {
            // 非法 ParamType（只可能来自坏文件/外部篡改）
            invalid = true;
        }

        if (invalid) {
            p->resetToDefault();
            ++invalid_count;
        }
    }

    return invalid_count;
}

} // namespace tuning
