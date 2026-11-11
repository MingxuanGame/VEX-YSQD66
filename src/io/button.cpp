#include "io/button.hpp"

namespace io {

void Button::update(bool raw, uint32_t now) {
    m_justPressed = false;
    m_justReleased = false;
    m_longFired = false;

    if (raw && !m_pressed) {
        // 按键从松开变为按下
        m_pressed = true;
        m_justPressed = true;
        m_pressTime = now;
        m_lastRepeat = now;
    } else if (!raw && m_pressed) {
        // 按键从按下变为松开
        m_pressed = false;
        m_justReleased = true;
    } else if (raw && m_pressed) {
        // 持续按住，判断长按
        if (!m_longFired && (now - m_pressTime) >= kLongPressMs) {
            m_longFired = true;
        }
    }
}

bool Button::pressed() const {
    return m_pressed;
}

bool Button::justPressed() const {
    return m_justPressed;
}

bool Button::justReleased() const {
    return m_justReleased;
}

bool Button::longPressed() const {
    return m_longFired;
}

bool Button::repeated(uint32_t interval, uint32_t now) {
    if (!m_pressed)
        return false;
    if (now - m_lastRepeat >= interval) {
        m_lastRepeat = now;
        return true;
    }
    return false;
}

uint32_t Button::pressDuration(uint32_t now) const {
    if (!m_pressed)
        return 0;
    return now - m_pressTime;
}

void Button::reset() {
    m_pressed = false;
    m_justPressed = false;
    m_justReleased = false;
    m_longFired = false;
    m_pressTime = 0;
    m_lastRepeat = 0;
}

} // namespace io
