#include "calibration/session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>

namespace calibration {
namespace {

constexpr std::uint32_t kArmHoldMs = 2000;
constexpr std::uint32_t kRenderPeriodMs = 200;
constexpr std::size_t kEntryLimit = 12;

} // namespace

Session::Session(macrorec::Hardware &hardware) : m_trial(hardware) {}

void Session::open() {
    m_trial.cancel();
    m_active = true;
    m_menu_page = 0;
    reset_screen_touch();
    change_page(Page::Menu);
}

void Session::interrupt() {
    if (m_active && (m_page == Page::Armed || m_page == Page::Running)) {
        abort("Mode changed. Trial cancelled and drive stopped.");
    }
    if (m_active) {
        reset_screen_touch();
    }
}

bool Session::active() const {
    return m_active;
}

void Session::change_page(Page page) {
    m_page = page;
    m_notice = nullptr;
    m_redraw = true;
}

void Session::begin_fields(bool after) {
    m_entering_after = after;
    m_field = 0;
    const auto &item = recipe(m_test);
    if ((after ? item.after : item.before).empty()) {
        if (after) {
            finish();
        } else {
            prepare_trial();
        }
        return;
    }
    change_page(Page::Entry);
    load_field();
}

void Session::load_field() {
    const auto &item = recipe(m_test);
    const auto &field = (m_entering_after ? item.after : item.before)[m_field];
    const auto &saved = m_remembered[static_cast<std::size_t>(field.remember)];
    const double value = saved.value_or(field.initial);
    m_entry.clear();
    m_entry_prefilled = std::isfinite(value);
    if (m_entry_prefilled) {
        std::array<char, 24> digits{};
        std::snprintf(digits.data(), digits.size(), "%.6f", value);
        m_entry = digits.data();
        m_entry.erase(m_entry.find_last_not_of('0') + 1);
        if (m_entry.back() == '.') {
            m_entry.pop_back();
        }
    }
    m_notice = nullptr;
    m_redraw = true;
}

void Session::accept_field() {
    if (m_entry.empty()) {
        return;
    }
    const auto &item = recipe(m_test);
    const auto fields = m_entering_after ? item.after : item.before;
    const auto &field = fields[m_field];
    char *end = nullptr;
    const double value = std::strtod(m_entry.c_str(), &end);
    if (end != m_entry.c_str() + m_entry.size() || !std::isfinite(value) || value < field.min ||
        value > field.max) {
        m_notice = "Enter a number in the shown range.";
        m_redraw = true;
        return;
    }
    (m_entering_after ? m_after : m_before)[m_field++] = value;
    if (m_field < fields.size()) {
        load_field();
    } else if (m_entering_after) {
        finish();
    } else {
        prepare_trial();
    }
}

void Session::prepare_trial() {
    if (recipe(m_test).trial == TrialKind::None) {
        begin_fields(true);
        return;
    }
    m_a_released = false;
    m_hold_since_ms.reset();
    change_page(Page::Armed);
}

void Session::finish() {
    const auto &item = recipe(m_test);
    const auto before = std::span{m_before}.first(item.before.size());
    const auto after = std::span{m_after}.first(item.after.size());
    m_result = calculate(m_test, before, after, m_trial.result());
    if (!m_result.error) {
        for (std::size_t i = 0; i < m_result.count; ++i) {
            const auto &value = m_result.values[i];
            if (value.remember != Remember::None) {
                m_remembered[static_cast<std::size_t>(value.remember)] = value.value;
            }
        }
    }
    change_page(Page::Result);
}

void Session::abort(const char *reason) {
    m_trial.cancel();
    m_result = {.error = reason};
    change_page(Page::Result);
}

void Session::handle(Touch touch) {
    const bool numeric = touch.action == Action::Digit || touch.action == Action::Dot;
    if (numeric && m_entry_prefilled) {
        m_entry.clear();
    }
    switch (touch.action) {
    case Action::Exit:
        m_trial.cancel();
        m_active = false;
        break;
    case Action::Menu:
        change_page(Page::Menu);
        break;
    case Action::Previous:
        --m_menu_page;
        m_redraw = true;
        break;
    case Action::Next:
        ++m_menu_page;
        m_redraw = true;
        break;
    case Action::Select:
        m_trial.cancel();
        m_test = static_cast<Test>(touch.value);
        change_page(Page::Guide);
        break;
    case Action::Proceed:
        begin_fields(false);
        break;
    case Action::Cancel:
        abort("Trial cancelled. Drive stopped.");
        break;
    case Action::Digit:
        if (m_entry.size() < kEntryLimit) {
            m_entry.push_back(static_cast<char>('0' + touch.value));
        }
        break;
    case Action::Dot:
        if (m_entry.size() < kEntryLimit && m_entry.find('.') == std::string::npos) {
            m_entry += m_entry.empty() ? "0." : ".";
        }
        break;
    case Action::Backspace:
        if (!m_entry.empty()) {
            m_entry.pop_back();
        }
        break;
    case Action::Accept:
        accept_field();
        break;
    case Action::None:
        break;
    }
    if (numeric || touch.action == Action::Backspace) {
        m_entry_prefilled = false;
        m_notice = nullptr;
        m_redraw = true;
    }
}

void Session::update_trial(std::uint32_t now_ms, bool allowed, bool connected, bool a_held) {
    if (m_page == Page::Armed) {
        if (!allowed || !connected) {
            m_a_released = false;
            m_hold_since_ms.reset();
        } else if (!a_held) {
            m_a_released = true;
            m_hold_since_ms.reset();
        } else if (m_a_released) {
            if (!m_hold_since_ms) {
                m_hold_since_ms = now_ms;
            }
            if (now_ms - *m_hold_since_ms >= kArmHoldMs) {
                const auto fields = recipe(m_test).before;
                m_trial.start(trial_config(m_test, std::span{m_before}.first(fields.size())),
                              now_ms);
                if (m_trial.state() == TrialState::Fault) {
                    abort(m_trial.status());
                } else {
                    change_page(Page::Running);
                }
            }
        }
        return;
    }
    if (m_page != Page::Running) {
        return;
    }
    if (!allowed || !connected || !a_held) {
        abort("A released, controller lost, or mode changed.");
        return;
    }
    m_trial.update(now_ms);
    if (m_trial.state() == TrialState::Complete) {
        begin_fields(true);
    } else if (m_trial.state() == TrialState::Fault) {
        abort(m_trial.status());
    }
}

Screen Session::screen_layout() const {
    return {.page = m_page,
            .title = m_page == Page::Menu ? nullptr : recipe(m_test).title,
            .value = m_entry.c_str(),
            .can_proceed = m_page != Page::Entry || !m_entry.empty(),
            .menu_page = m_menu_page};
}

void Session::render(std::uint32_t now_ms, bool allowed, bool connected) {
    Screen screen = screen_layout();
    const auto &item = recipe(m_test);
    if (m_page == Page::Guide) {
        std::copy(item.guide.begin(), item.guide.end(), screen.lines.begin());
    } else if (m_page == Page::Entry) {
        const auto fields = m_entering_after ? item.after : item.before;
        const auto &field = fields[m_field];
        std::snprintf(m_lines[0].data(), m_lines[0].size(), "%s %zu/%zu | %.6g to %.6g",
                      m_entering_after ? "Result input" : "Setup", m_field + 1, fields.size(),
                      field.min, field.max);
        screen.lines[0] = field.label;
        screen.lines[1] = m_lines[0].data();
        if (m_entering_after && item.trial != TrialKind::None) {
            const auto &measured = m_trial.result();
            if (m_test == Test::Travel) {
                const double mean_deg =
                    std::accumulate(measured.delta_deg.begin(), measured.delta_deg.end(), 0.0) /
                    4.0;
                std::snprintf(m_lines[1].data(), m_lines[1].size(), "Mean encoder %.1f deg",
                              mean_deg);
            } else if (m_test == Test::SpeedPid) {
                std::snprintf(m_lines[1].data(), m_lines[1].size(), "Measured %.2f in/s",
                              std::midpoint(measured.left_in_s, measured.right_in_s));
            } else {
                std::snprintf(m_lines[1].data(), m_lines[1].size(), "IMU angle %.2f deg",
                              measured.yaw_deg);
            }
            screen.lines[2] = m_lines[1].data();
        }
        screen.lines[3] = m_notice;
    } else if (m_page == Page::Armed) {
        screen.lines[0] = "Ready for practice. Clear the test area.";
        screen.lines[1] = "Hold controller A for 2 seconds to begin.";
        screen.lines[2] = "Release A at any time to stop.";
        if (!allowed) {
            screen.lines[3] = "Unplug field control and enable Driver.";
        } else if (!connected) {
            screen.lines[3] = "Connect the controller.";
        } else if (m_hold_since_ms) {
            const double wait_s = (kArmHoldMs - (now_ms - *m_hold_since_ms)) / 1000.0;
            std::snprintf(m_lines[0].data(), m_lines[0].size(), "Starting in %.1f s", wait_s);
            screen.lines[3] = m_lines[0].data();
        }
    } else if (m_page == Page::Running) {
        screen.lines[0] = m_trial.status();
        screen.lines[1] = "Hold A until the run completes.";
        screen.lines[2] = "Release A or tap CANCEL to stop.";
    } else if (m_page == Page::Result) {
        if (m_result.error) {
            screen.lines[0] = m_result.error;
            screen.lines[1] = "Return to the test menu to try again.";
        } else {
            screen.lines[0] = "Copy to web: PID / Calibration";
            for (std::size_t i = 0; i < m_result.count; ++i) {
                const auto &value = m_result.values[i];
                std::snprintf(m_lines[i].data(), m_lines[i].size(), "%s = %.6g %s", value.key,
                              value.value, value.unit);
                screen.lines[i + 1] = m_lines[i].data();
            }
        }
    }
    draw_screen(screen);
    m_rendered_ms = now_ms;
    m_redraw = false;
}

void Session::tick(std::uint32_t now_ms, bool allowed, bool controller_connected, bool a_held) {
    handle(poll_screen_touch(screen_layout()));
    if (!m_active) {
        return;
    }
    update_trial(now_ms, allowed, controller_connected, a_held);
    const bool live = m_page == Page::Armed || m_page == Page::Running;
    if (m_redraw || (live && now_ms - m_rendered_ms >= kRenderPeriodMs)) {
        render(now_ms, allowed, controller_connected);
    }
}

} // namespace calibration
