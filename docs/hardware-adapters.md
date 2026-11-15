# 真机适配器状态

## 结论

`ProsChassis`、`ProsImu`、`ProsMechanism` 目前是**精简接线级适配器**，用于把现有硬件模块接入
`robot::Chassis` / `robot::Imu` / `robot::Mechanism` 接口，**不是比赛级实现**。

## 已有内容

- `ProsChassis`
  - `setDrive`：-127..127 直接交给 `chassis::set_power`
  - `stop`：调用 `chassis::stop`
  - `isHealthy`：记录 `chassis::set_power` 返回失败
  - 电流、速度、位置：当前 chassis 接口没有反馈，返回 0
  - `tarePosition`：当前无编码器反馈，空操作
- `ProsImu`
  - 包装 `pros::Imu` 的 heading / rotation / gyro Z
  - `isHealthy`：检查 PROS status 且不在校准中
  - 空指针和非有限值返回 0
- `ProsMechanism`
  - 升降使用一个 `pros::Motor*`
  - 夹爪、推板、电流反馈通过静态回调注入
  - `isHealthy`、升降上下限和总故障通过可选回调注入
  - 不猜测未确定的端口和传动

## 尚未达到比赛级

- chassis 没有暴露电流、速度、编码器反馈，`ProsChassis` 无法做防推挤或运动闭环
- 没有物理编码器 tare
- `setDrive` 和 `liftTo` 无法向控制层返回硬件失败
- IMU 状态已接入，但实机校准和故障验证未完成
- 升降限位只提供回调入口，阈值、方向和堵转/电流超时仍需实机标定
- 夹爪/推板回调没有统一到位反馈
- 没有实机 HIL 验证

## 边界

- `src/main.cpp` 目前由 `macrorec` 接管手动底盘，适配器尚未接入主循环
- `ProsChassis` 的反馈 getter 返回 0，只能作为“不可用”占位
- `ControlLoop` 会在底盘、IMU 或机构报告不健康时进入 SafeWait
- `pros build` 通过仅代表编译和链接通过，不代表比赛可靠性

## 下一步

1. 由 `main.cpp` 构造适配器并接入 `ControlLoop`
2. 补 `ProsChassis` 的物理 tare 和故障接口
3. 补 `ProsImu` 校准/在线状态
4. 补 `ProsMechanism` 限位、电流和超时
5. 使用场控和实机完成启用/禁用、自动转手动、断线、限位验证
