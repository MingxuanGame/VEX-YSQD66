#pragma once
#include "collision/collision_types.hpp"

/**
 * @file collision_detector.hpp
 * @brief 碰撞检测器：通过多特征融合，判断是否碰撞并分类。
 *        检测与恢复分离，确保调整判断阈值时不会误改恢复逻辑。
 */

namespace robot {
class Chassis;
class Imu;
} // namespace robot

namespace collision {

class CollisionDetector {
public:
    enum class ObservationState { Idle, Observing, Done };

    explicit CollisionDetector(const CollisionConfig &config = {});

    /**
     * @brief 注入底盘与 IMU。
     * @note 真车和仿真都通过这个构造函数接入，检测器内部不再依赖任何具体硬件。
     */
    CollisionDetector(robot::Chassis &chassis, robot::Imu &imu, const CollisionConfig &config = {});

    /**
     * @brief 运行期替换依赖，便于测试或换车。
     */
    void setDependencies(robot::Chassis *chassis, robot::Imu *imu);

    /**
     * @brief 轻量级前置检查：判断是否触发碰撞条件。
     * @return true 表示可能发生碰撞，需要进一步调用 detect()。
     * @note 放在主循环中调用，过滤掉正常行驶状态，避免频繁进入耗时逻辑。
     */
    bool isTriggered();

    /**
     * @brief 非阻塞观察窗口：先 startObservation()，之后每帧 updateObservation()。
     *        ControlLoop 应使用这一组接口，不要在 50Hz 主循环里调用 detect()。
     */
    bool startObservation(uint32_t now_ms);
    bool updateObservation(uint32_t now_ms);
    bool isObserving() const;
    bool isObservationDone() const;
    CollisionType getObservationResult() const;
    const CollisionFeatures &getObservationFeatures() const;

    /**
     * @brief 兼容旧代码的阻塞式检测：内部仍会 delay，不要在 50Hz 主循环调用。
     * @param out_features 可选指针，传入则填充特征数据。
     */
    CollisionType detect(CollisionFeatures *out_features = nullptr);

    const CollisionConfig &getConfig() const;
    void setConfig(const CollisionConfig &config);

    // ==================== 故障注入接口 ====================
    /**
     * @brief 注入假数据，用于不接硬件在桌面上测试分类逻辑是否正确。
     * @param fake_current 假电流值
     * @param fake_velocity 假速度值
     * @param fake_gyro 假角速度值
     * @note 注入后，检测器将忽略真实电机和 IMU 数据。传负数退出注入模式。
     */
    void injectFakeData(double fake_current, double fake_velocity, double fake_gyro);

private:
    CollisionConfig m_config;

    robot::Chassis *m_chassis = nullptr;
    robot::Imu *m_imu = nullptr;

    bool m_is_injecting = false;
    double m_fake_current = 0.0;
    double m_fake_velocity = 0.0;
    double m_fake_gyro = 0.0;

    ObservationState m_observe_state = ObservationState::Idle;
    uint32_t m_observe_start_ms = 0;
    uint32_t m_observe_last_sample_ms = 0;
    double m_observe_sum_gyro = 0.0;
    double m_observe_sum_current = 0.0;
    double m_observe_sum_current_sq = 0.0;
    int m_observe_samples = 0;
    CollisionFeatures m_observe_features;
    CollisionType m_observe_result = CollisionType::None;

    void sampleObservation(uint32_t now_ms);
    CollisionType finishObservation(uint32_t now_ms);

    double readAvgCurrent() const;
    double readAvgVelocity() const;
    double readLeftCurrent() const;
    double readRightCurrent() const;
    double readLeftVelocity() const;
    double readRightVelocity() const;
    double readGyro() const;
};

} // namespace collision
