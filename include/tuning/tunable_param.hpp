#pragma once
#include <cstddef>
#include <variant>

/**
 * @file tunable_param.hpp
 * @brief 单个参数的封装。
 *        不使用 std::string / std::function，名称用固定长度字符数组，
 *        满足 VEX 单片机禁止动态内存分配的约束。
 */

namespace tuning {

using IntDoubleType = std::variant<int, double>;

enum class MoveDirection { Left, Right };
enum class ParamType { Int, Double };

enum class ParamGroup {
    AllGroup,
    PIDGroup,
    CollisionGroup,
    MoveGroup,
    LiftGroup,
    ClampGroup,
    NoneGroup
};

class TunableParam {
public:
    static constexpr int kMaxNameLen = 24;

    TunableParam();

    TunableParam(const char *name, IntDoubleType value, IntDoubleType minValue,
                 IntDoubleType maxValue, ParamType type, ParamGroup group, int minDigit,
                 int maxDigit);

    void incParam();
    void decParam();
    void moveDigitPos(MoveDirection direction);

    void saveParam();
    void undoVariance();
    void redoVariance();
    void resetToDefault();
    void setValueRange(IntDoubleType minValue, IntDoubleType maxValue);

    const char *getParamName() const;
    IntDoubleType getParamValue() const;
    IntDoubleType getDefaultValue() const;
    IntDoubleType getMinValue() const;
    IntDoubleType getMaxValue() const;
    ParamType getParamType() const;
    int getParamDigitPos() const;
    int getMinPos() const;
    int getMaxPos() const;
    ParamGroup getParamGroup() const;

    bool isDirty() const;
    void clearDirty();

    int getIntValue(int defaultValue = 0) const;
    double getDoubleValue(double defaultValue = 0.0) const;

    /**
     * @brief 序列化到固定缓冲区，不产生任何堆分配。
     * @return false 表示缓冲区不够或格式化失败。
     */
    bool serialize(char *out, std::size_t capacity) const;

    /**
     * @brief 从一行文本解析。非法类型/格式返回 false，不修改当前对象。
     */
    bool deserialize(const char *line);

private:
    double getActualStep() const;
    void clampDigitPos();
    void clampValue();
    void clearRedoList();
    void pushHistory();
    void markDirty();
    void normalizeValueByType();

    char m_name[kMaxNameLen];
    IntDoubleType m_value;
    IntDoubleType m_default_value;
    IntDoubleType m_min_value;
    IntDoubleType m_max_value;
    ParamType m_type;

    int m_digit_pos;
    int m_min_digit_pos;
    int m_max_digit_pos;

    ParamGroup m_group;

    IntDoubleType m_history[8];
    IntDoubleType m_redolist[8];
    int m_history_size;
    int m_redolist_size;

    bool m_dirty;
};

} // namespace tuning
