#pragma once
#include <cstdint>

/**
 * @file collision_types.hpp
 * @brief 碰撞系统的核心数据结构。
 *        将所有数据类型集中在此，避免检测器、恢复器、日志和调参模块之间出现循环依赖。
 */

namespace collision {

// ==================== 碰撞类型枚举 ====================
// 区分不同碰撞原因是制定恢复策略的前提：
// 撞车可能是对方会退，撞墙必须自己退，撞锥则应该直接抓取。
enum class CollisionType {
    None,     // 未发生碰撞
    Car,      // 撞车：伴随晃动/反弹，需要观察对方是否退让
    Deadzone, // 撞墙/死区：持续堵转，需要后退绕行
    Cone,     // 撞锥：锥被推开，速度会迅速恢复
    Unknown   // 特征不明显，走保底的安全容错
};

// ==================== 碰撞特征数据 ====================
// 这些特征用于在无额外传感器的情况下，通过多维度数据区分撞车、撞墙和撞锥。
struct CollisionFeatures {
    double max_gyro = 0.0;         // 峰值：撞车有剧烈晃动，撞墙则是瞬间冲击后快速稳定
    double avg_gyro = 0.0;         // 均值：判断晃动是否持续
    int gyro_above_count = 0;      // 超阈值次数：过滤单次尖峰造成的误判
    double avg_current = 0.0;      // 均值：判断是否持续堵转
    double current_variance = 0.0; // 方差：对方移动会导致电流波动，撞墙则波动小
    uint32_t duration_ms = 0;      // 耗时：确保观察窗口在复杂环境下没有失真

    double velocity_at_trigger = 0.0; // 触发瞬间速度：用于判断恢复后是否脱困
    double velocity_at_end = 0.0;     // 窗口结束速度
    bool velocity_recovered = false;  // 速度是否恢复：锥被推开后轮速会重新起来，这是锥的特征
};

// ==================== 碰撞配置参数 ====================
// 抽离成独立结构体，是为了支持比赛现场通过遥控器/调参系统实时修改，无需重新编译。
struct CollisionConfig {
    // ---- 触发条件 ----
    double current_threshold_ma = 2500.0; // 高于此值认为可能碰撞
    double velocity_threshold_rpm = 10.0; // 低于此值认为可能堵转

    // ---- 观察窗口 ----
    int observe_window_ms = 200; // 200ms 足够区分反弹和死区，太长会拖慢自动赛节奏

    // ---- 晃动判断 ----
    double gyro_threshold_deg_s = 15.0;           // 区分底盘正常转向与真实撞击
    int gyro_above_count = 10;                    // 200ms 内超阈值 10 次（共 20 个采样）算持续晃动
    double current_variance_threshold = 200000.0; // 方差大说明对方在动

    // ---- 撞锥判断 ----
    // 锥较轻且容易推开，电流不会持续极高，且脱离后速度恢复快
    double cone_current_min = 1200.0;
    double cone_current_max = 2500.0;
    int cone_gyro_above_max = 5;
    double cone_velocity_recovered = 20.0;

    // ---- 撞车恢复 ----
    int car_wait_ms = 200;             // 给对手一点反应时间
    double car_clear_current = 2000.0; // 电流降到此值说明路可能通了
    double car_clear_velocity = 20.0;

    // ---- 脱离动作 ----
    int escape_back_distance = 3;
    int escape_speed = 40;
    int escape_back_ms = 300;    // 单次脱离动作持续时间（没有里程计时用它代替距离）
    int escape_timeout_ms = 800; // 单次脱离动作的超时，防止卡死
    double escape_clear_current = 2000.0;
    int path_probe_ms = 100;        // 试探路是否通畅的时间
    int path_probe_speed = 30;      // 试探时给的油门
    bool enable_side_escape = true; // 直退不行时，允许左后/右后斜退尝试

    // ---- 防推挤（AntiPush） ----
    double ant_push_current_threshold = 2000.0; // 持续高电流说明被对方顶着
    int ant_push_time_ms = 500;                 // 持续被顶多久开始卸力
    int ant_push_escape_speed = -60;            // 卸力时的反向速度，避免电机烧毁

    // ---- 通用 ----
    int recover_back_distance = 2;
    int recover_wait_ms = 500;
    int max_retries = 2; // 重试限制，防止陷入死循环
};

} // namespace collision
