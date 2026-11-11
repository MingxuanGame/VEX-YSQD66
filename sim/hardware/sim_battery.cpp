#include "sim/hardware/sim_battery.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

int Battery::m_open_circuit_mv = 12800;
double Battery::m_load_current_ma = 0.0;
double Battery::m_internal_resistance_mohm = 20.0;

int Battery::getVoltage() {
    const double current_a = std::abs(m_load_current_ma) / 1000.0;
    const double resistance_ohm = m_internal_resistance_mohm / 1000.0;
    const double sag_mv = current_a * resistance_ohm * 1000.0;
    const double terminal_mv = static_cast<double>(m_open_circuit_mv) - sag_mv;
    return static_cast<int>(std::max(0.0, terminal_mv));
}

int Battery::getOpenCircuitVoltage() {
    return m_open_circuit_mv;
}

void Battery::setVoltage(int mv) {
    m_open_circuit_mv = std::max(0, mv);
}

void Battery::setInternalResistanceMilliohm(double mohm) {
    m_internal_resistance_mohm = std::max(0.0, mohm);
}

void Battery::updateLoad(double current_ma) {
    m_load_current_ma = std::max(0.0, current_ma);
}

void Battery::reset() {
    m_open_circuit_mv = 12800;
    m_load_current_ma = 0.0;
    m_internal_resistance_mohm = 20.0;
}

} // namespace sim
