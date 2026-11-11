#pragma once
#include "tuning/tunable_param.hpp"

/**
 * @file params_manager.hpp
 * @brief 参数数组的容器。
 *        使用固定大小数组和函数模板遍历，不依赖 std::function / std::string。
 */

namespace tuning {

class ParamsManager {
private:
    static constexpr int kMaxParams = 32;
    TunableParam m_params_arr[kMaxParams];
    int m_count;
    int m_selected_index;
    ParamGroup m_current_group;

    bool inCurrentGroup(int index) const;

public:
    ParamsManager();

    int addParam(const TunableParam &param);
    int addParam(const char *name, IntDoubleType value, IntDoubleType minValue,
                 IntDoubleType maxValue, ParamType type, ParamGroup group, int minDigit,
                 int maxDigit);

    TunableParam *getParam(int index);
    TunableParam *getParam(const char *name);
    const TunableParam *getParam(int index) const;
    int getSize() const;

    int getSelectedIndex() const;
    void setSelectedIndex(int index);
    TunableParam *getSelectedParam();
    void selectNext();
    void selectPrev();

    void setGroup(ParamGroup group);
    ParamGroup getGroup() const;
    void nextGroup();
    void prevGroup();

    // 模板遍历：不产生 std::function 的堆分配，具体 lambda 由编译器内联。
    template <typename Func> void forEachDirty(Func &&func) {
        for (int i = 0; i < m_count; ++i) {
            if (m_params_arr[i].isDirty())
                func(m_params_arr[i]);
        }
    }

    template <typename Func> void forEach(Func &&func) {
        for (int i = 0; i < m_count; ++i)
            func(m_params_arr[i]);
    }

    void clearAllDirty();
    void resetAll();

#ifdef ENABLE_SIMULATOR
    // 只有 PC 仿真允许文件系统；V5 固件不编译这些函数。
    bool saveToFile(const char *path) const;
    int loadFromFile(const char *path);
#endif
};

} // namespace tuning
