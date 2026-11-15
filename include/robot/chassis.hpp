#pragma once

/**
 * @file chassis.hpp
 * @brief 底盘抽象接口。
 *        由于底盘实现代码可能多变，所有上层逻辑（碰撞、自动赛、PID）必须通过此接口操作底盘，
 *        严禁直接操作 pros::Motor。这保证了未来换底盘结构时，上层代码零修改。
 */

namespace robot {

class Chassis {
public:
    virtual ~Chassis() = default;

    /**
     * @brief 坦克驾驶（左右轮独立控制）
     * @param left 左轮输出 -127~127
     * @param right 右轮输出 -127~127
     */
    virtual void setDrive(int left, int right) = 0;

    /**
     * @brief 停止底盘
     * @param brake 是否开启主动刹车（撞墙恢复时需要踩死，平时停车可滑行）
     */
    virtual void stop(bool brake = true) = 0;

    // ==================== 传感器数据读取 ====================
    // 为什么要在底盘接口里读电流/速度？因为电机直接挂在底盘上，上层不该去关心是左前还是左后电机。

    virtual double getLeftCurrent() const = 0;  // 左侧平均电流 (mA)
    virtual double getRightCurrent() const = 0; // 右侧平均电流 (mA)

    virtual double getLeftVelocity() const = 0;  // 左侧平均速度 (rpm)
    virtual double getRightVelocity() const = 0; // 右侧平均速度 (rpm)

    virtual double getLeftPosition() const = 0; // 左侧平均编码器位置
    virtual double getRightPosition() const = 0;

    /**
     * @brief 清零底盘编码器
     * @note 自动赛开始前必须调用，否则里程计会继承上一场的脏数据。
     */
    virtual void tarePosition() = 0;

    /**
     * @brief 最近一次驱动命令是否成功。
     *
     * 默认返回 true，兼容仿真/Fake 实现；真机适配器应在 SDK 写入失败时返回 false。
     * ControlLoop 只把它作为进入 SafeWait 的依据，不把 0 反馈当成实测值。
     */
    virtual bool isHealthy() const {
        return true;
    }
};

} // namespace robot
