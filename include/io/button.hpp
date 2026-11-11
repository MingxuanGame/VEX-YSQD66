#pragma once
#include <cstdint>

/**
 * @file button.hpp
 * @brief 遥控器按键类。
 *        为什么需要这个类？原生 get_digital 只能读当前状态，无法判断长按、双击，
 *        这个类封装了消抖、刚按下、长按、连发逻辑，让上层控制更加安全。
 */

namespace io {

class Button {
private:
    bool m_pressed = false;      // 当前是否按住
    bool m_justPressed = false;  // 本帧刚按下
    bool m_justReleased = false; // 本帧刚松开
    bool m_longFired = false;    // 本帧长按触发（只触发一次）
    uint32_t m_pressTime = 0;    // 按下的时间戳
    uint32_t m_lastRepeat = 0;   // 上次连发的时间戳

    //500ms 符合人手按键的肌肉记忆，太短容易误触，太长按着累
    static constexpr uint32_t kLongPressMs = 500;

public:
    /**
     * @brief 每帧更新按键状态
     * @param raw 原生 get_digital 读出的布尔值
     * @param now 当前系统时间（由主循环传入，避免频繁调用 millis）
     */
    void update(bool raw, uint32_t now);

    bool pressed() const;      // 当前按住
    bool justPressed() const;  // 刚按下（常用于切换模式）
    bool justReleased() const; // 刚松开
    bool longPressed() const;  // 长按触发

    /**
     * @brief 连发检测（按住不放时每隔一段时间触发一次）
     * @param interval 连发间隔（毫秒）
     * @param now 当前时间戳
     * @return 是否触发连发
     */
    bool repeated(uint32_t interval, uint32_t now);

    uint32_t pressDuration(uint32_t now) const; // 获取按住了多久

    void reset(); // 重置状态
};

} // namespace io
