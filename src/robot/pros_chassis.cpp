#include "robot/pros_chassis.hpp"
#include "chassis.h"

#include <algorithm>

namespace robot {
namespace {

enum class ReadingField { Current, Velocity, Position };

double averageReading(bool left, ReadingField field) {
    const auto readings = chassis::readings();
    const int first = left ? 0 : 2;
    const int second = first + 1;
    double sum = 0.0;
    int count = 0;
    for (int i = first; i <= second; ++i) {
        if (!readings[i].valid)
            continue;
        if (field == ReadingField::Current)
            sum += readings[i].current_ma;
        else if (field == ReadingField::Velocity)
            sum += readings[i].velocity_rpm;
        else
            sum += readings[i].position_deg;
        ++count;
    }
    return count > 0 ? sum / count : 0.0;
}

} // namespace

void ProsChassis::setDrive(int left, int right) {
    const int left_mv = std::clamp(left, -127, 127) * 12000 / 127;
    const int right_mv = std::clamp(right, -127, 127) * 12000 / 127;
    chassis::set_voltage(left_mv, right_mv);
}

void ProsChassis::stop(bool brake) {
    // chassis::stop() 当前固定使用主动刹车；保留接口参数以便底盘模块以后扩展。
    (void)brake;
    chassis::stop();
}

double ProsChassis::getLeftCurrent() const {
    return averageReading(true, ReadingField::Current);
}

double ProsChassis::getRightCurrent() const {
    return averageReading(false, ReadingField::Current);
}

double ProsChassis::getLeftVelocity() const {
    return averageReading(true, ReadingField::Velocity);
}

double ProsChassis::getRightVelocity() const {
    return averageReading(false, ReadingField::Velocity);
}

double ProsChassis::getLeftPosition() const {
    return averageReading(true, ReadingField::Position) - m_left_zero_deg;
}

double ProsChassis::getRightPosition() const {
    return averageReading(false, ReadingField::Position) - m_right_zero_deg;
}

void ProsChassis::tarePosition() {
    m_left_zero_deg = averageReading(true, ReadingField::Position);
    m_right_zero_deg = averageReading(false, ReadingField::Position);
}

} // namespace robot
