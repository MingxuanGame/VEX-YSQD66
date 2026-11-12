# 真机适配器状态

## 结论

`ProsChassis`、`ProsImu`、`ProsMechanism` 目前是**精简接线级适配器**，用于把现有硬件模块接入
`robot::Chassis` / `robot::Imu` / `robot::Mechanism` 接口，**不是比赛级实现**。

## 已有内容

- `ProsChassis`
  - `setDrive`：-127..127 换算为 ±12000mV，调用 `chassis::set_voltage`
  - `stop`：调用 `chassis::stop`
  - 电流、速度、位置：读取 `chassis::readings`
  - `tarePosition`：软件零点，不物理清零编码器
- `ProsImu`
  - 包装 `pros::Imu` 的 heading / rotation / gyro Z
  - 空指针和非有限值返回 0
- `ProsMechanism`
  - 升降使用一个 `pros::Motor*`
  - 夹爪、推板、电流反馈通过静态回调注入
  - 不猜测未确定的端口和传动

## 尚未达到比赛级

- 没有物理编码器 tare
- `setDrive` 和 `liftTo` 无法向控制层返回硬件失败
- IMU 没有校准/在线状态上报，错误会被 0 掩盖
- 没有反馈缓存，单个控制周期可能多次读取四电机
- 升降没有软/硬限位、电流、堵转和到位超时检查
- 夹爪/推板回调没有统一到位反馈
- 没有实机 HIL 验证

## 边界

- `src/auton/*` 和 `src/main.cpp` 未修改
- 适配器尚未接入 `opcontrol` 主循环
- `pros build` 通过仅代表编译和链接通过，不代表比赛可靠性

## 下一步

1. 由 `main.cpp` 构造适配器并接入 `ControlLoop`
2. 补 `ProsChassis` 的物理 tare 和故障接口
3. 补 `ProsImu` 校准/在线状态
4. 补 `ProsMechanism` 限位、电流和超时
5. 使用场控和实机完成启用/禁用、自动转手动、断线、限位验证
