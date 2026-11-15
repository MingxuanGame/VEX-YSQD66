#include "robot/pros_chassis.hpp"
#include "chassis.h"

#include <algorithm>

namespace robot {

void ProsChassis::setDrive(int left, int right) {
    // 新底盘接口只接受组件功率；调用方负责避免与 macrorec/ControlLoop 同时写入。
    m_healthy = chassis::set_power(std::clamp(left, -127, 127), std::clamp(right, -127, 127));
}

void ProsChassis::stop(bool brake) {
    // chassis::stop() 当前固定使用主动刹车；保留接口参数以便底盘模块以后扩展。
    (void)brake;
    chassis::stop();
}

// 新 chassis 接口没有暴露电流、速度或编码器反馈，不能在适配器里伪造这些值。
double ProsChassis::getLeftCurrent() const {
    return 0.0;
}

double ProsChassis::getRightCurrent() const {
    return 0.0;
}

double ProsChassis::getLeftVelocity() const {
    return 0.0;
}

double ProsChassis::getRightVelocity() const {
    return 0.0;
}

double ProsChassis::getLeftPosition() const {
    return 0.0;
}

double ProsChassis::getRightPosition() const {
    return 0.0;
}

void ProsChassis::tarePosition() {
    // 没有可读编码器，tare 只能等待底盘模块补回反馈接口。
}

bool ProsChassis::isHealthy() const {
    return m_healthy;
}

} // namespace robot
