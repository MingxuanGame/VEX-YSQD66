#include "collision/collision_detector.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"
#include "robot/clock.hpp"
#include <cmath>

namespace collision {

CollisionDetector::CollisionDetector(const CollisionConfig &config) : m_config(config) {}

CollisionDetector::CollisionDetector(robot::Chassis &chassis, robot::Imu &imu,
                                     const CollisionConfig &config)
    : m_config(config), m_chassis(&chassis), m_imu(&imu) {}

void CollisionDetector::setDependencies(robot::Chassis *chassis, robot::Imu *imu) {
    m_chassis = chassis;
    m_imu = imu;
}

const CollisionConfig &CollisionDetector::getConfig() const {
    return m_config;
}

void CollisionDetector::setConfig(const CollisionConfig &config) {
    m_config = config;
}

double CollisionDetector::readAvgCurrent() const {
    if (m_is_injecting)
        return m_fake_current;
    if (!m_chassis)
        return 0.0;
    return (m_chassis->getLeftCurrent() + m_chassis->getRightCurrent()) / 2.0;
}

double CollisionDetector::readAvgVelocity() const {
    if (m_is_injecting)
        return m_fake_velocity;
    if (!m_chassis)
        return 0.0;
    return (std::abs(m_chassis->getLeftVelocity()) + std::abs(m_chassis->getRightVelocity())) / 2.0;
}

double CollisionDetector::readGyro() const {
    if (m_is_injecting)
        return m_fake_gyro;
    if (!m_imu)
        return 0.0;
    return std::abs(m_imu->getGyroRateZ());
}

bool CollisionDetector::isTriggered() {
    if (!m_is_injecting && !m_chassis)
        return false;

    double current = readAvgCurrent();
    double velocity = readAvgVelocity();

    return (current > m_config.current_threshold_ma && velocity < m_config.velocity_threshold_rpm);
}

bool CollisionDetector::startObservation(uint32_t now_ms) {
    m_observe_state = ObservationState::Observing;
    m_observe_start_ms = now_ms;
    m_observe_last_sample_ms = now_ms;
    m_observe_sum_gyro = 0.0;
    m_observe_sum_current = 0.0;
    m_observe_sum_current_sq = 0.0;
    m_observe_samples = 0;
    m_observe_features = CollisionFeatures{};
    m_observe_result = CollisionType::None;

    m_observe_features.velocity_at_trigger = readAvgVelocity();
    sampleObservation(now_ms);
    return true;
}

bool CollisionDetector::updateObservation(uint32_t now_ms) {
    if (m_observe_state != ObservationState::Observing) {
        return m_observe_state == ObservationState::Done;
    }

    int window_ms = m_config.observe_window_ms;
    if (window_ms < 10)
        window_ms = 10;
    if (window_ms > 2000)
        window_ms = 2000;

    // 采样间隔 10ms，但每帧只取一次，避免主循环被拖慢。
    if (static_cast<int32_t>(now_ms - m_observe_last_sample_ms) >= 10) {
        sampleObservation(now_ms);
    }

    if (static_cast<int32_t>(now_ms - m_observe_start_ms) >= window_ms) {
        finishObservation(now_ms);
        return true;
    }
    return false;
}

bool CollisionDetector::isObserving() const {
    return m_observe_state == ObservationState::Observing;
}

bool CollisionDetector::isObservationDone() const {
    return m_observe_state == ObservationState::Done;
}

CollisionType CollisionDetector::getObservationResult() const {
    return m_observe_result;
}

const CollisionFeatures &CollisionDetector::getObservationFeatures() const {
    return m_observe_features;
}

void CollisionDetector::sampleObservation(uint32_t now_ms) {
    const double gyro = readGyro();
    const double current = readAvgCurrent();

    if (gyro > m_observe_features.max_gyro)
        m_observe_features.max_gyro = gyro;
    m_observe_sum_gyro += gyro;
    if (gyro > m_config.gyro_threshold_deg_s) {
        m_observe_features.gyro_above_count++;
    }

    m_observe_sum_current += current;
    m_observe_sum_current_sq += current * current;
    m_observe_samples++;
    m_observe_last_sample_ms = now_ms;
}

CollisionType CollisionDetector::finishObservation(uint32_t now_ms) {
    m_observe_features.duration_ms = now_ms - m_observe_start_ms;
    if (m_observe_samples > 0) {
        m_observe_features.avg_gyro = m_observe_sum_gyro / m_observe_samples;
        m_observe_features.avg_current = m_observe_sum_current / m_observe_samples;
        const double mean_sq = m_observe_sum_current_sq / m_observe_samples;
        m_observe_features.current_variance =
            mean_sq - m_observe_features.avg_current * m_observe_features.avg_current;
        if (m_observe_features.current_variance < 0.0) {
            m_observe_features.current_variance = 0.0;
        }
    }

    m_observe_features.velocity_at_end = readAvgVelocity();
    m_observe_features.velocity_recovered =
        (m_observe_features.velocity_at_end > m_config.cone_velocity_recovered);

    m_observe_result = CollisionType::Unknown;
    if (m_observe_features.gyro_above_count >= m_config.gyro_above_count &&
        m_observe_features.current_variance > m_config.current_variance_threshold) {
        m_observe_result = CollisionType::Car;
    } else if (m_observe_features.avg_current > m_config.cone_current_min &&
               m_observe_features.avg_current < m_config.cone_current_max &&
               m_observe_features.gyro_above_count <= m_config.cone_gyro_above_max &&
               m_observe_features.velocity_recovered) {
        m_observe_result = CollisionType::Cone;
    } else if (m_observe_features.gyro_above_count <= m_config.cone_gyro_above_max &&
               m_observe_features.avg_current > m_config.current_threshold_ma) {
        m_observe_result = CollisionType::Deadzone;
    }

    m_observe_state = ObservationState::Done;
    return m_observe_result;
}

CollisionType CollisionDetector::detect(CollisionFeatures *out_features) {
    // 兼容旧调用：内部仍是阻塞的，主循环请改用 startObservation/updateObservation。
    startObservation(robot::nowMs());
    while (!updateObservation(robot::nowMs())) {
        robot::delayMs(10);
    }
    if (out_features)
        *out_features = m_observe_features;
    return m_observe_result;
}

// 注入负数表示退出注入模式；任一值为负即退出。
void CollisionDetector::injectFakeData(double fake_current, double fake_velocity,
                                       double fake_gyro) {
    if (fake_current < 0.0 || fake_velocity < 0.0 || fake_gyro < 0.0) {
        m_is_injecting = false;
        return;
    }

    m_is_injecting = true;
    m_fake_current = fake_current;
    m_fake_velocity = fake_velocity;
    m_fake_gyro = fake_gyro;
}

} // namespace collision
