# 四轮麦轮底盘参考实现

## 这个目录是什么

`sim/ref/` 是**不参与编译**的参考实现，不是当前生产底盘代码。

保留 `.ref` 后缀，避免 `src/`、`sim/` 的构建 glob 把它当成真实源文件收进去，造成重复定义或端口冲突。

当前生产代码仍是：

```text
include/chassis.h
src/chassis.cpp
```

本目录只回答一个问题：

> 如果要把底盘改成 4 个独立麦轮，接口和实现应该是什么形状？

---

## 机械假设

按 `xize.txt` 和照片：

```text
4 个麦克纳姆轮：FL / FR / BL / BR
4 个独立 V5 智能电机
每轮两级齿轮减速
无悬挂
主控居中
```

因此参考实现覆盖：

1. 运行时可校验的四轮配置
2. 四个电机只构造一次
3. 四轮功率/速度输出
4. 四轮电流/速度/位置/温度反馈
5. 四轮物理 tare 和统一 stop
6. 功率/速度模式仲裁
7. 过流、过温、命令超时故障
8. 功率斜率限制
9. 初始化自检、显式 `clearFault()`

运动学、车体速度、上层 PID、里程计放到上层，不塞进 chassis。

---

## 端口与方向必须先填

端口在 `Config::wheels[i].port` 中：

```text
正数 = 正转
负数 = 构造时反转
```

`initialize()` 会检查：

- 端口绝对值是否在 1..21
- 四轮端口是否重复
- 减速比、最大轮速、电流/温度阈值是否合法

不会检查实际电机是否真的在线；这需要 `read_state()` 和台架测试。

---

## 文件

- `chassis_reference.hpp.ref`：建议的 chassis 公共接口
- `chassis_reference.cpp.ref`：四电机实现样板
- `mecanum_reference.md.ref`：上层如何使用四轮接口和 `MecanumKinematics`
- `wheel_chassis_reference.hpp.ref`：四轮底盘抽象接口
- `pros_wheel_chassis_reference.hpp.ref` / `.cpp.ref`：未来 `ProsWheelChassis` 适配层
- `control_integration_reference.md.ref`：四轮控制会话顺序和安全边界

复制到真实工程时：

```text
chassis_reference.hpp.ref -> include/chassis.h
chassis_reference.cpp.ref -> src/chassis.cpp
wheel_chassis_reference.hpp.ref -> include/robot/wheel_chassis.hpp
pros_wheel_chassis_reference.hpp.ref -> include/robot/pros_wheel_chassis.hpp
pros_wheel_chassis_reference.cpp.ref -> src/robot/pros_wheel_chassis.cpp
```

但不要覆盖现有文件而不做端口、方向和齿轮比核对。

---

## 安全边界

参考实现负责：

- 禁用/自动阶段拒绝输出
- 功率和轮端转速限幅
- 功率斜率限制
- SDK 写入失败后停止
- 四轮统一停止
- 过流/过温/命令超时进入 Fault
- 只在 Stopped 状态允许物理 tare
- 反馈时间戳和有效性

不负责：

- 车体速度分配
- 上层加减速曲线
- 碰撞恢复
- 电压补偿
- 任务调度

这些属于上层。

---

## 完整度

```text
接口/模式/故障/反馈骨架：完整
四轮驱动基础实现：完整
比赛级参数和实机验证：未完成
```

未完成项：

- 实际端口、方向、齿比、轮径、轮距、轴距
- 电机在线自检和方向自检
- 台架/实车 HIL
- 温度模型的长期降额
- 与上层 `MecanumDriveController`/`MecanumOdometry` 的实际接线

参考代码只做了 ARM 语法检查，未做编译链接和实机验证。

---

## 标定清单

上真车前必须确认：

```text
四个电机端口
四个电机方向
电机齿轮箱颜色
两级齿轮总减速比
麦轮直径
轮距
轴距
每个轮子的最大安全转速
```

没有这些数据之前，参考代码只能作为结构样板，不能作为比赛参数。
