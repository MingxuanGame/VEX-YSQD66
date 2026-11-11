#include "tuning/param_apply.hpp"
#include <cstring>

/**
 * @file param_apply.cpp
 * @brief 将调参系统的数据同步到下层控制逻辑。
 */

namespace tuning {

ParamApply::Binding ParamApply::s_bindings[ParamApply::kMaxBindings];
int ParamApply::s_binding_count = 0;

bool ParamApply::bind(const char *name, Handler handler) {
    if (!name || !handler)
        return false;

    for (int i = 0; i < s_binding_count; ++i) {
        if (std::strcmp(s_bindings[i].name, name) == 0) {
            s_bindings[i].handler = handler;
            s_bindings[i].context_handler = nullptr;
            s_bindings[i].context = nullptr;
            return true;
        }
    }

    if (s_binding_count >= kMaxBindings)
        return false;
    s_bindings[s_binding_count++] = Binding{name, handler, nullptr, nullptr};
    return true;
}

bool ParamApply::bind(const char *name, ContextHandler handler, void *context) {
    if (!name || !handler)
        return false;

    for (int i = 0; i < s_binding_count; ++i) {
        if (std::strcmp(s_bindings[i].name, name) == 0) {
            s_bindings[i].handler = nullptr;
            s_bindings[i].context_handler = handler;
            s_bindings[i].context = context;
            return true;
        }
    }

    if (s_binding_count >= kMaxBindings)
        return false;
    s_bindings[s_binding_count++] = Binding{name, nullptr, handler, context};
    return true;
}

void ParamApply::unbind(const char *name) {
    if (!name)
        return;
    for (int i = 0; i < s_binding_count; ++i) {
        if (std::strcmp(s_bindings[i].name, name) == 0) {
            for (int j = i; j < s_binding_count - 1; ++j) {
                s_bindings[j] = s_bindings[j + 1];
            }
            --s_binding_count;
            s_bindings[s_binding_count] = Binding{};
            return;
        }
    }
}

void ParamApply::clear() {
    for (int i = 0; i < s_binding_count; ++i) {
        s_bindings[i] = Binding{};
    }
    s_binding_count = 0;
}

void ParamApply::applyAllDirty(ParamsManager &pm) {
    pm.forEachDirty([](TunableParam &p) {
        for (int i = 0; i < s_binding_count; ++i) {
            if (std::strcmp(s_bindings[i].name, p.getParamName()) != 0)
                continue;

            if (s_bindings[i].context_handler) {
                s_bindings[i].context_handler(p, s_bindings[i].context);
            } else if (s_bindings[i].handler) {
                s_bindings[i].handler(p);
            } else {
                continue;
            }
            p.clearDirty();
            return;
        }
    });
}

} // namespace tuning
