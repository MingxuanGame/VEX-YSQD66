#include "tuning/params_manager.hpp"
#include <cstdio>
#include <cstring>

/**
 * @file params_manager.cpp
 * @brief 参数集合的维护：注册、选中过滤、存读。
 */

namespace tuning {

ParamsManager::ParamsManager()
    : m_count(0), m_selected_index(0), m_current_group(ParamGroup::AllGroup) {}

bool ParamsManager::inCurrentGroup(int index) const {
    if (index < 0 || index >= m_count)
        return false;
    if (m_current_group == ParamGroup::AllGroup)
        return true;
    return m_params_arr[index].getParamGroup() == m_current_group;
}

int ParamsManager::addParam(const TunableParam &param) {
    if (m_count >= kMaxParams)
        return -1;
    for (int i = 0; i < m_count; ++i) {
        if (std::strcmp(m_params_arr[i].getParamName(), param.getParamName()) == 0) {
            return -1;
        }
    }
    m_params_arr[m_count] = param;
    return m_count++;
}

int ParamsManager::addParam(const char *name, IntDoubleType value, IntDoubleType minValue,
                            IntDoubleType maxValue, ParamType type, ParamGroup group, int minDigit,
                            int maxDigit) {
    TunableParam param(name, value, minValue, maxValue, type, group, minDigit, maxDigit);
    return addParam(param);
}

TunableParam *ParamsManager::getParam(int index) {
    if (index < 0 || index >= m_count)
        return nullptr;
    return &m_params_arr[index];
}

TunableParam *ParamsManager::getParam(const char *name) {
    if (!name)
        return nullptr;
    for (int i = 0; i < m_count; ++i) {
        if (std::strcmp(m_params_arr[i].getParamName(), name) == 0) {
            return &m_params_arr[i];
        }
    }
    return nullptr;
}

const TunableParam *ParamsManager::getParam(int index) const {
    if (index < 0 || index >= m_count)
        return nullptr;
    return &m_params_arr[index];
}

int ParamsManager::getSize() const {
    return m_count;
}

int ParamsManager::getSelectedIndex() const {
    return m_selected_index;
}

void ParamsManager::setSelectedIndex(int index) {
    if (index >= 0 && index < m_count && inCurrentGroup(index)) {
        m_selected_index = index;
    }
}

TunableParam *ParamsManager::getSelectedParam() {
    return getParam(m_selected_index);
}

void ParamsManager::selectNext() {
    if (m_count == 0)
        return;
    for (int i = 1; i <= m_count; ++i) {
        int idx = (m_selected_index + i) % m_count;
        if (inCurrentGroup(idx)) {
            m_selected_index = idx;
            return;
        }
    }
}

void ParamsManager::selectPrev() {
    if (m_count == 0)
        return;
    for (int i = 1; i <= m_count; ++i) {
        int idx = (m_selected_index - i + m_count * 2) % m_count;
        if (inCurrentGroup(idx)) {
            m_selected_index = idx;
            return;
        }
    }
}

void ParamsManager::setGroup(ParamGroup group) {
    m_current_group = group;
    if (!inCurrentGroup(m_selected_index)) {
        for (int i = 0; i < m_count; ++i) {
            if (inCurrentGroup(i)) {
                m_selected_index = i;
                return;
            }
        }
    }
}

ParamGroup ParamsManager::getGroup() const {
    return m_current_group;
}

static bool groupHasParam(const TunableParam *arr, int count, ParamGroup group) {
    if (group == ParamGroup::AllGroup)
        return count > 0;
    for (int i = 0; i < count; ++i) {
        if (arr[i].getParamGroup() == group)
            return true;
    }
    return false;
}

void ParamsManager::nextGroup() {
    int max_group = static_cast<int>(ParamGroup::NoneGroup);
    int start = static_cast<int>(m_current_group);
    for (int step = 1; step <= max_group + 1; ++step) {
        int group = (start + step) % (max_group + 1);
        if (groupHasParam(m_params_arr, m_count, static_cast<ParamGroup>(group))) {
            setGroup(static_cast<ParamGroup>(group));
            return;
        }
    }
}

void ParamsManager::prevGroup() {
    int max_group = static_cast<int>(ParamGroup::NoneGroup);
    int start = static_cast<int>(m_current_group);
    for (int step = 1; step <= max_group + 1; ++step) {
        int group = (start - step + (max_group + 1) * 2) % (max_group + 1);
        if (groupHasParam(m_params_arr, m_count, static_cast<ParamGroup>(group))) {
            setGroup(static_cast<ParamGroup>(group));
            return;
        }
    }
}

void ParamsManager::clearAllDirty() {
    for (int i = 0; i < m_count; ++i)
        m_params_arr[i].clearDirty();
}

void ParamsManager::resetAll() {
    for (int i = 0; i < m_count; ++i)
        m_params_arr[i].resetToDefault();
}

#ifdef ENABLE_SIMULATOR

bool ParamsManager::saveToFile(const char *path) const {
    if (!path)
        return false;
    std::FILE *file = std::fopen(path, "w");
    if (!file)
        return false;

    bool ok = true;
    char line[256];
    for (int i = 0; i < m_count; ++i) {
        if (!m_params_arr[i].serialize(line, sizeof(line)) ||
            std::fprintf(file, "%s\n", line) < 0) {
            ok = false;
            break;
        }
    }
    if (std::fclose(file) != 0)
        ok = false;
    return ok;
}

int ParamsManager::loadFromFile(const char *path) {
    if (!path)
        return -1;
    std::FILE *file = std::fopen(path, "r");
    if (!file)
        return -1;

    int loaded = 0;
    char line[256];
    while (std::fgets(line, sizeof(line), file)) {
        char *separator = std::strchr(line, '|');
        if (!separator)
            continue;

        *separator = '\0';
        TunableParam *param = getParam(line);
        *separator = '|';

        if (param && param->deserialize(line))
            ++loaded;
    }
    std::fclose(file);
    return loaded;
}

#endif

} // namespace tuning
