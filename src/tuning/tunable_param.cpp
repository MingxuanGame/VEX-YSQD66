#include "tuning/tunable_param.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

/**
 * @file tunable_param.cpp
 * @brief 单个参数的具体实现：步长计算、限幅、撤销/重做和固定缓冲区序列化。
 */

namespace tuning {

namespace {

// 把任意 variant 值转成参数声明的类型，避免 std::get<T> 抛 bad_variant_access。
IntDoubleType normalized(const IntDoubleType &value, ParamType type) {
    if (type == ParamType::Int) {
        return std::holds_alternative<int>(value) ? std::get<int>(value)
                                                  : static_cast<int>(std::get<double>(value));
    }
    return std::holds_alternative<double>(value) ? std::get<double>(value)
                                                 : static_cast<double>(std::get<int>(value));
}

} // namespace

TunableParam::TunableParam()
    : m_name{}, m_value(0), m_default_value(0), m_min_value(0), m_max_value(0),
      m_type(ParamType::Int), m_digit_pos(0), m_min_digit_pos(0), m_max_digit_pos(0),
      m_group(ParamGroup::NoneGroup), m_history_size(0), m_redolist_size(0), m_dirty(false) {
    m_name[0] = '\0';
    for (int i = 0; i < 8; ++i) {
        m_history[i] = 0;
        m_redolist[i] = 0;
    }
}

TunableParam::TunableParam(const char *name, IntDoubleType value, IntDoubleType minValue,
                           IntDoubleType maxValue, ParamType type, ParamGroup group, int minDigit,
                           int maxDigit)
    : m_name{}, m_value(value), m_default_value(value), m_min_value(minValue),
      m_max_value(maxValue), m_type(type), m_digit_pos(0), m_min_digit_pos(minDigit),
      m_max_digit_pos(maxDigit), m_group(group), m_history_size(0), m_redolist_size(0),
      m_dirty(false) {
    std::snprintf(m_name, sizeof(m_name), "%s", name ? name : "");
    for (int i = 0; i < 8; ++i) {
        m_history[i] = 0;
        m_redolist[i] = 0;
    }

    normalizeValueByType();
    clampDigitPos();
    clampValue();
}

double TunableParam::getActualStep() const {
    return std::pow(10.0, static_cast<double>(m_digit_pos));
}

void TunableParam::clampDigitPos() {
    if (m_type == ParamType::Int && m_digit_pos < 0)
        m_digit_pos = 0;
    if (m_digit_pos < m_min_digit_pos)
        m_digit_pos = m_min_digit_pos;
    if (m_digit_pos > m_max_digit_pos)
        m_digit_pos = m_max_digit_pos;
}

void TunableParam::clampValue() {
    // 先归一化，保证 std::get 的 variant index 与 m_type 一致。
    m_value = normalized(m_value, m_type);
    m_min_value = normalized(m_min_value, m_type);
    m_max_value = normalized(m_max_value, m_type);

    if (m_type == ParamType::Int) {
        int v = std::get<int>(m_value);
        if (v < std::get<int>(m_min_value))
            v = std::get<int>(m_min_value);
        if (v > std::get<int>(m_max_value))
            v = std::get<int>(m_max_value);
        m_value = v;
    } else {
        double v = std::get<double>(m_value);
        if (v < std::get<double>(m_min_value))
            v = std::get<double>(m_min_value);
        if (v > std::get<double>(m_max_value))
            v = std::get<double>(m_max_value);
        m_value = v;
    }
}

void TunableParam::normalizeValueByType() {
    m_value = normalized(m_value, m_type);
    m_min_value = normalized(m_min_value, m_type);
    m_max_value = normalized(m_max_value, m_type);
    m_default_value = normalized(m_default_value, m_type);
}

void TunableParam::clearRedoList() {
    m_redolist_size = 0;
}

void TunableParam::pushHistory() {
    if (m_history_size >= 8) {
        for (int i = 0; i < 7; ++i)
            m_history[i] = m_history[i + 1];
        m_history_size = 7;
    }
    m_history[m_history_size++] = m_value;
    clearRedoList();
}

void TunableParam::markDirty() {
    m_dirty = true;
}

void TunableParam::clearDirty() {
    m_dirty = false;
}

bool TunableParam::isDirty() const {
    return m_dirty;
}

void TunableParam::incParam() {
    normalizeValueByType();
    double step = getActualStep();
    if (m_type == ParamType::Int) {
        m_value = std::get<int>(m_value) + static_cast<int>(step);
    } else {
        m_value = std::get<double>(m_value) + step;
    }
    clampValue();
    markDirty();
}

void TunableParam::decParam() {
    normalizeValueByType();
    double step = getActualStep();
    if (m_type == ParamType::Int) {
        m_value = std::get<int>(m_value) - static_cast<int>(step);
    } else {
        m_value = std::get<double>(m_value) - step;
    }
    clampValue();
    markDirty();
}

void TunableParam::moveDigitPos(MoveDirection direction) {
    if (direction == MoveDirection::Left)
        m_digit_pos += 1;
    else
        m_digit_pos -= 1;
    clampDigitPos();
}

void TunableParam::saveParam() {
    if (m_dirty) {
        clampValue();
        pushHistory();
        clearDirty();
    }
}

void TunableParam::undoVariance() {
    if (m_history_size <= 0)
        return;
    if (m_redolist_size >= 8) {
        for (int i = 0; i < 7; ++i)
            m_redolist[i] = m_redolist[i + 1];
        m_redolist_size = 7;
    }
    m_redolist[m_redolist_size++] = m_value;

    m_value = m_history[--m_history_size];
    clampValue();
    markDirty();
}

void TunableParam::redoVariance() {
    if (m_redolist_size <= 0)
        return;
    if (m_history_size >= 8) {
        for (int i = 0; i < 7; ++i)
            m_history[i] = m_history[i + 1];
        m_history_size = 7;
    }
    m_history[m_history_size++] = m_value;

    m_value = m_redolist[--m_redolist_size];
    clampValue();
    markDirty();
}

void TunableParam::resetToDefault() {
    m_value = m_default_value;
    clampValue();
    markDirty();
}

void TunableParam::setValueRange(IntDoubleType minValue, IntDoubleType maxValue) {
    m_min_value = minValue;
    m_max_value = maxValue;
    normalizeValueByType();

    if (m_type == ParamType::Int) {
        int lo = std::get<int>(m_min_value);
        int hi = std::get<int>(m_max_value);
        if (lo > hi) {
            int tmp = lo;
            lo = hi;
            hi = tmp;
            m_min_value = lo;
            m_max_value = hi;
        }
    } else {
        double lo = std::get<double>(m_min_value);
        double hi = std::get<double>(m_max_value);
        if (lo > hi) {
            double tmp = lo;
            lo = hi;
            hi = tmp;
            m_min_value = lo;
            m_max_value = hi;
        }
    }

    clampValue();
    m_default_value = normalized(m_default_value, m_type);

    if (m_type == ParamType::Int) {
        int def = std::get<int>(m_default_value);
        def = std::clamp(def, std::get<int>(m_min_value), std::get<int>(m_max_value));
        m_default_value = def;
    } else {
        double def = std::get<double>(m_default_value);
        def = std::clamp(def, std::get<double>(m_min_value), std::get<double>(m_max_value));
        m_default_value = def;
    }
}

const char *TunableParam::getParamName() const {
    return m_name;
}

IntDoubleType TunableParam::getParamValue() const {
    return m_value;
}

IntDoubleType TunableParam::getDefaultValue() const {
    return m_default_value;
}

IntDoubleType TunableParam::getMinValue() const {
    return m_min_value;
}

IntDoubleType TunableParam::getMaxValue() const {
    return m_max_value;
}

ParamType TunableParam::getParamType() const {
    return m_type;
}

int TunableParam::getParamDigitPos() const {
    return m_digit_pos;
}

int TunableParam::getMinPos() const {
    return m_min_digit_pos;
}

int TunableParam::getMaxPos() const {
    return m_max_digit_pos;
}

ParamGroup TunableParam::getParamGroup() const {
    return m_group;
}

int TunableParam::getIntValue(int defaultValue) const {
    const int *value = std::get_if<int>(&m_value);
    return (m_type == ParamType::Int && value) ? *value : defaultValue;
}

double TunableParam::getDoubleValue(double defaultValue) const {
    const double *value = std::get_if<double>(&m_value);
    return (m_type == ParamType::Double && value) ? *value : defaultValue;
}

bool TunableParam::serialize(char *out, std::size_t capacity) const {
    if (!out || capacity == 0)
        return false;

    int written = 0;
    if (m_type == ParamType::Int) {
        written = std::snprintf(
            out, capacity, "%s|%d|%d|%d|%d|%d|%d|%d|%d", m_name, static_cast<int>(m_type),
            std::get<int>(m_value), m_min_digit_pos, m_max_digit_pos, static_cast<int>(m_group),
            std::get<int>(m_min_value), std::get<int>(m_max_value), m_digit_pos);
    } else {
        written = std::snprintf(
            out, capacity, "%s|%d|%.6f|%d|%d|%d|%.6f|%.6f|%d", m_name, static_cast<int>(m_type),
            std::get<double>(m_value), m_min_digit_pos, m_max_digit_pos, static_cast<int>(m_group),
            std::get<double>(m_min_value), std::get<double>(m_max_value), m_digit_pos);
    }
    return written > 0 && static_cast<std::size_t>(written) < capacity;
}

bool TunableParam::deserialize(const char *line) {
    if (!line)
        return false;

    char name[kMaxNameLen] = {};
    int type = -1;
    int min_digit = 0;
    int max_digit = 0;
    int group = 0;
    int digit = 0;
    double value = 0.0;
    double min_value = 0.0;
    double max_value = 0.0;

    int parsed = std::sscanf(line, "%23[^|]|%d|%lf|%d|%d|%d|%lf|%lf|%d", name, &type, &value,
                             &min_digit, &max_digit, &group, &min_value, &max_value, &digit);
    if (parsed < 8)
        return false;
    if (parsed < 9)
        digit = 0;
    if (type != static_cast<int>(ParamType::Int) && type != static_cast<int>(ParamType::Double)) {
        return false;
    }

    const bool has_registered_default = m_name[0] != '\0';
    const IntDoubleType registered_default = m_default_value;

    TunableParam temp;
    std::snprintf(temp.m_name, sizeof(temp.m_name), "%s", name);
    temp.m_type = static_cast<ParamType>(type);
    temp.m_min_digit_pos = min_digit;
    temp.m_max_digit_pos = max_digit;
    temp.m_group = static_cast<ParamGroup>(group);
    temp.m_digit_pos = digit;

    if (temp.m_type == ParamType::Int) {
        temp.m_value = static_cast<int>(value);
        temp.m_min_value = static_cast<int>(min_value);
        temp.m_max_value = static_cast<int>(max_value);
    } else {
        temp.m_value = value;
        temp.m_min_value = min_value;
        temp.m_max_value = max_value;
    }

    temp.m_default_value = has_registered_default ? registered_default : temp.m_value;

    temp.normalizeValueByType();
    temp.clampDigitPos();
    temp.clampValue();

    *this = temp;
    markDirty();
    return true;
}

} // namespace tuning
