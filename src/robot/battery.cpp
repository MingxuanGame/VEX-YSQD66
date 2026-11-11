#include "robot/pros_battery.hpp"
#include "pros/misc.hpp"

/**
 * @file battery.cpp
 * @brief pros::battery 到 robot::Battery 的适配器。
 */

namespace robot {

namespace {

class ProsBattery : public Battery {
public:
    int getVoltageMv() const override {
        return pros::battery::get_voltage();
    }
};

} // namespace

Battery &prosBattery() {
    static ProsBattery battery;
    return battery;
}

} // namespace robot
