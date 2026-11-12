#include "auton/kit.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace auton {

void Editor::reset(const Buttons &buttons) {
    m_previous = buttons;
    m_active = false;
    m_sequence = 0;
}

EditorEvent Editor::update(const Buttons &buttons, std::uint32_t now_ms, Config &config,
                           bool running) {
    EditorEvent event{};
    if (!buttons.connected || !m_previous.connected) {
        m_previous = buttons;
        m_sequence = 0;
        event.stop = true;
        return event;
    }
    std::array<bool, kKeyCount> pressed{};
    int count = 0;
    int key_index = 0;
    for (int i = 0; i < kKeyCount; ++i) {
        pressed[i] = buttons.down[i] && !m_previous.down[i];
        if (pressed[i]) {
            ++count;
            key_index = i;
        }
    }
    m_previous = buttons;
    const auto hit = [&pressed](Key key) { return pressed[static_cast<int>(key)]; };
    if (running) {
        event.stop = hit(Key::R2);
        return event;
    }
    if (!m_active) {
        constexpr std::array<Key, 10> kSequence{Key::Up,   Key::Up,    Key::Down, Key::Down,
                                                Key::Left, Key::Right, Key::Left, Key::Right,
                                                Key::B,    Key::A};
        constexpr std::array<int, 10> kFallback{0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
        if (now_ms - m_last_key_ms > 2000 || count > 1) {
            m_sequence = 0;
        }
        if (count == 1) {
            m_last_key_ms = now_ms;
            const auto key = static_cast<Key>(key_index);
            while (m_sequence > 0 && key != kSequence[m_sequence]) {
                m_sequence = kFallback[m_sequence - 1];
            }
            if (key == kSequence[m_sequence]) {
                ++m_sequence;
            }
            if (m_sequence == static_cast<int>(kSequence.size())) {
                m_active = true;
                m_sequence = 0;
                event.stop = true;
            }
        }
        return event;
    }
    if (hit(Key::R2)) {
        m_active = false;
        event.save = m_dirty;
        event.stop = true;
        return event;
    }
    // 多键只接受退出，避免保存/起跑与改值在同一帧发生。
    if (count != 1) {
        return event;
    }
    if (hit(Key::L1)) {
        config.selected_route = (config.selected_route + 1) % kRouteCount;
        m_dirty = true;
    }
    if (hit(Key::Left) || hit(Key::Right)) {
        m_step = (m_step + (hit(Key::Right) ? 1 : kStepCount - 1)) % kStepCount;
        m_param = 0;
    }
    if (hit(Key::Up) || hit(Key::Down)) {
        do {
            m_param = (m_param + (hit(Key::Down) ? 1 : kParamCount - 1)) % kParamCount;
        } while (!uses_param(m_step, static_cast<Param>(m_param)));
    }
    if (hit(Key::Y)) {
        m_digit = std::min(4, m_digit + 1);
    }
    if (hit(Key::A)) {
        m_digit = std::max(0, m_digit - 1);
    }
    if (hit(Key::X) || hit(Key::B)) {
        constexpr std::array<int, 5> kPowers{1, 10, 100, 1000, 10000};
        auto &value = config.routes[config.selected_route].steps[m_step].values[m_param];
        const int adjusted = std::clamp(value + (hit(Key::X) ? 1 : -1) * kPowers[m_digit], 0,
                                        param_max(static_cast<Param>(m_param)));
        if (adjusted != value) {
            value = adjusted;
            m_dirty = true;
        }
    }
    event.save = hit(Key::R1);
    event.test_step = hit(Key::L2);
    return event;
}

bool Editor::active() const {
    return m_active;
}

int Editor::step() const {
    return m_step;
}

Param Editor::param() const {
    return static_cast<Param>(m_param);
}

int Editor::digit() const {
    return m_digit;
}

bool Editor::dirty() const {
    return m_dirty;
}

void Editor::mark_saved() {
    m_dirty = false;
}

Screen render(const Config &config, const Editor &editor, const Runner &runner,
              const std::array<MotorReading, 4> &motors, const char *storage_status) {
    Screen screen{};
    const int route_index = config.selected_route;
    const int step_index = editor.step();
    std::snprintf(screen[0].data(), screen[0].size(), "R%d S%d %s run:%d/%d %s", route_index + 1,
                  step_index + 1, editor.active() ? "EDIT" : "LOCK", runner.step() + 1,
                  static_cast<int>(runner.phase()), runner.status());
    const auto param = editor.param();
    const char *name = param_name(param);
    const int value = config.routes[route_index].steps[step_index].values[static_cast<int>(param)];
    std::snprintf(screen[1].data(), screen[1].size(), "%s = %05d %s | digit 10^%d", name, value,
                  param == Param::Angle ? "deg" : "ms", editor.digit());
    const int arrow = static_cast<int>(std::strlen(name)) + 3 + 4 - editor.digit();
    std::memset(screen[2].data(), ' ', arrow);
    screen[2][arrow] = '^';
    std::snprintf(screen[3].data(), screen[3].size(), "%.18s%s L1:R L2:go R1:save R2:exit/stop",
                  storage_status ? storage_status : "", editor.dirty() ? " *" : "");
    std::snprintf(screen[4].data(), screen[4].size(),
                  "S Ddeg back  fwd   lift  down  act   hit   turn");
    for (int s = 0; s < kStepCount; ++s) {
        auto &line = screen[5 + s];
        int offset = std::snprintf(line.data(), line.size(), "%d", s + 1);
        for (int p = 0; p < kParamCount; ++p) {
            const int width = p == 0 ? 4 : 6;
            if (uses_param(s, static_cast<Param>(p))) {
                offset += std::snprintf(line.data() + offset, line.size() - offset, "%*d", width,
                                        config.routes[route_index].steps[s].values[p]);
            } else {
                offset +=
                    std::snprintf(line.data() + offset, line.size() - offset, "%*s", width, "-");
            }
        }
    }
    std::snprintf(screen[11].data(), screen[11].size(),
                  "Motor ACTUAL: port mV rpm deg mA (ERR=missing)");
    for (int i = 0; i < 4; ++i) {
        const auto &motor = motors[i];
        auto &line = screen[12 + i];
        if (motor.valid) {
            std::snprintf(line.data(), line.size(), "P%+03d %+.0fmV %+.0frpm %+.0fdeg %dmA",
                          motor.port, double(motor.voltage_mv), motor.velocity_rpm,
                          motor.position_deg, motor.current_ma);
        } else {
            std::snprintf(line.data(), line.size(), "P%+03d ERR", motor.port);
        }
    }
    return screen;
}
} // namespace auton
