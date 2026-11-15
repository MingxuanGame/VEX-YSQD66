#pragma once

/**
 * @file mechanism.hpp
 * @brief 上层机构抽象接口（抬升、夹取、推板）。
 *        为了支持不同机械结构的快速迭代，控制层只调用这些语义化函数。
 */

namespace robot {

class Mechanism {
public:
    virtual ~Mechanism() = default;

    // ==================== 抬升 ====================
    /**
     * @brief 抬升到指定高度
     * @param pos 目标位置（编码器度数或自定义高度单位）
     * @param max_speed 最大速度限制 0~127
     * @note 为什么需要 max_speed？因为直接满速抬升会导致机械臂剧烈晃动，容易翻车。
     */
    virtual void liftTo(double pos, int max_speed = 100) = 0;

    virtual void liftManual(int speed) = 0; // 遥控手动控制
    virtual void liftHold() = 0;            // 停在当前高度

    // ==================== 夹取 ====================
    virtual void clampOpen() = 0;
    virtual void clampClose() = 0;
    // 如果是舵机，需要给具体角度；如果是气缸，内部自行处理
    virtual void clampSetAngle(int angle) = 0;

    // ==================== 推板 ====================
    // 如果推板是固定机械件（没电机），这个函数内部留空即可
    virtual void pushExtend() = 0;
    virtual void pushRetract() = 0;

    // 机构总电流（mA），用于主控全局电流预算；没有电流传感器的机构返回 0。
    virtual double getCurrentMa() const {
        return 0.0;
    }

    /**
     * @brief 机构是否没有限位/故障回调报告的故障。
     *
     * 默认返回 true 兼容仿真；ProsMechanism 可通过注入的 fault 回调覆盖。
     */
    virtual bool isHealthy() const {
        return true;
    }
};

} // namespace robot
