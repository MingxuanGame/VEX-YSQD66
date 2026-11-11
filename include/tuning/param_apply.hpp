#pragma once
#include "tuning/params_manager.hpp"

/**
 * @file param_apply.hpp
 * @brief 参数应用分发。
 *        调参系统只管存数据，PID 和碰撞模块不该依赖调参系统。
 *        这里作为桥梁，把 dirty 的数据推送到具体的控制器里。
 */

namespace tuning {

class ParamApply {
public:
    using Handler = void (*)(const TunableParam &);
    using ContextHandler = void (*)(const TunableParam &, void *);

    /**
     * @brief 把一个参数名绑定到无上下文的写入函数。
     * @param name 参数名（必须是字符串字面量，内部只保存指针）
     * @param handler 收到新值时调用的函数
     */
    static bool bind(const char *name, Handler handler);

    /**
     * @brief 带上下文的绑定，适合绑定到某个具体控制器实例。
     * @param context 传给 handler 的指针，生命周期由调用方保证
     */
    static bool bind(const char *name, ContextHandler handler, void *context);

    static void unbind(const char *name);
    static void clear();

    /**
     * @brief 将当前所有 dirty 参数应用到对应的控制系统
     * @note 只清除“已绑定并成功应用”的 dirty 标记；未绑定的参数保持 dirty。
     */
    static void applyAllDirty(ParamsManager &pm);

private:
    static constexpr int kMaxBindings = 48;

    struct Binding {
        const char *name;
        Handler handler;
        ContextHandler context_handler;
        void *context;
    };

    static Binding s_bindings[kMaxBindings];
    static int s_binding_count;
};

} // namespace tuning
