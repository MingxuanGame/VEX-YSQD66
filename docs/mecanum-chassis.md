# 四轮麦轮底盘适配方向

## 结论

当前代码与新的机械方案有明显不匹配：

```text
机械：4 个独立麦轮 + 4 个独立电机 + 两级齿轮减速
代码：左右两侧差速 + 两路功率 + 差速里程计
```

现有 `chassis::set_power(left, right)`、`macro` 的左右功率录制、`ChassisKinematics` 的左右编码器模型都不能直接支持麦轮全向移动。

---

## 必须先标定的机械参数

这些值不能再使用占位值，必须实测/数齿：

| 参数 | 当前代码 | 需要 |
|---|---|---|
| 总减速比 | `kDriveGearRatio = 1.0` | 两级齿轮实际齿数比 |
| 轮径 | 3.25 in | 实测麦轮外径 |
| 轮距 | 12.0 in | 左右轮中心距 |
| 轴距 | 未建模 | 前后轮中心距 |
| 电机最大转速 | 600 rpm | 实际齿轮箱和负载后轮端转速 |
| 四个轮子的端口/方向 | 左右各一组 | FL / FR / BL / BR 独立 |
| 电机正方向 | 未逐轮验证 | 每个轮子单独取反配置 |

两级减速还会带来：

- 明显背隙
- 换向空程
- 四轮摩擦和装配差异
- 高速打滑和里程漂移

这些都需要在控制层补偿，不能只改一个齿轮比。

---

## P0：底盘接口必须先升级

当前接口：

```cpp
bool set_power(int left_power, int right_power);
void stop();
```

对麦轮不够。建议在保留兼容入口的同时增加四轮接口：

```cpp
enum class Wheel { FrontLeft, FrontRight, BackLeft, BackRight };

bool set_wheel_power(Wheel wheel, int power);
bool set_wheel_power(int fl, int fr, int bl, int br);

struct WheelState {
    double current_ma;
    double velocity_rpm;
    double position_deg;
    bool valid;
};

struct ChassisState {
    WheelState wheel[4];
    bool valid;
};

bool read_state(ChassisState &out);
bool tare_position();
```

也可以保留：

```cpp
bool set_power(int left, int right); // 仅作为兼容包装
```

但麦轮控制不能再走左右分组。

---

## P0：麦轮运动学

建议把底盘原始四轮接口和运动学分开：

```text
chassis：四轮功率/速度 + 反馈
motion：麦轮逆解/正解 + 归一化
control：摇杆/宏/自动输出 vx, vy, omega
```

逆解示意：

```text
FL = vx - vy - k * omega
FR = vx + vy + k * omega
BL = vx + vy - k * omega
BR = vx - vy + k * omega
```

其中：

```text
vx    前进
vy    横移
omega 旋转
k     与轮距、轴距有关的比例系数
```

解算后必须归一化：

```text
四轮中最大目标值 <= 允许最大值
```

否则横移和旋转叠加时会单轮过载。

正解用于里程计，必须使用：

- 轮端真实转角
- 总减速比
- 麦轮正解
- IMU 航向融合

不能再直接用左右编码器差算航向。

---

## P1：四轮速度闭环

xize 明确要求四轮独立速度闭环。当前 `set_power()` 是开环功率。

建议：

- 使用电机内置速度闭环
- 四轮分别给 `move_velocity`
- 上层做轮速归一化
- 保留功率模式作为低速调试/故障降级

同时增加：

- 速度指令死区
- 线加速度限制
- 角加速度限制
- 换向背隙补偿
- 原地旋转单独限幅

当前 `DriveController` 的 `max_delta_per_update = 40` 只作用于左右两侧，需要迁移到 `vx/vy/omega` 或四轮目标。

---

## P1：宏录制要升级

当前宏只记录：

```text
left_x, left_y, right_x, right_y
chassis_left_power, chassis_right_power
```

它只能复现差速左右功率，不能复现：

- 横移
- 麦轮四轮差速
- 四轮速度闭环

需要新映射版本，例如：

```csv
mapping,mecanum_chassis_v1
t_ms,vx,vy,omega,...
```

或者直接记录四轮目标速度：

```csv
t_ms,fl,fr,bl,br,...
```

旧宏 `chassis_arcade_v1` 应保留读取兼容，但新录制必须使用麦轮映射。

---

## P1：手柄与仿真输入

当前手动只有：

```text
左摇杆 Y：前后
右摇杆 X：旋转
```

麦轮需要增加横移：

```text
左摇杆 X：横移
左摇杆 Y：前后
右摇杆 X：旋转
```

或：

```text
左摇杆：平移
右摇杆 X：旋转
```

仿真 UI 也要相应增加横移输入，不能继续把左右键当转向。

---

## P1：IMU 与里程计

需要：

- IMU 上电校准，带超时
- 航向零点和自动开始重设
- IMU + 四轮正解融合
- 编码器异常跳变过滤
- 滑动平均/低通滤波

麦轮打滑时，单纯编码器里程会漂移，必须用 IMU 修正航向。

---

## P1：打滑、堵转和齿隙保护

建议参数化：

```text
轮速目标 vs 实际差 -> 打滑
电流高 + 速度近零 -> 堵转
换向过零 -> 背隙补偿脉冲
```

保护动作：

- 单轮降功率
- 四轮同步降额
- 连续堵转停机
- 上报故障状态
- 进入 SafeWait

---

## P2：任务结构

xize 提到“基于 V5 多核心拆分为独立任务”。这里要谨慎：

- PROS 任务不是按核心绑定的硬实时分区
- 四个轮子各开一个任务会引入调度抖动和共享状态竞争
- 推荐一个确定性的底盘控制任务（100Hz 或 50Hz）
- 传感器/UI/SD 可以独立任务
- 共享状态必须有所有权或快照，不能多个任务同时写电机

不要为了“多任务”把控制链拆碎。

---

## P2：规则与结构尺寸

必须实测：

```text
起始 18 × 18 × 18 英寸
比赛水平展开 24 × 24 英寸
高度 50 英寸
```

照片中的高立柱和侧向齿轮/轮组需要重点核对起始尺寸和展开后的外廓。

同时确认：

- 4 个底盘电机总额定功率不超过 55W
- 预装锥 `<SG5>`
- 自动分界线 `<SG7>`
- 持有限制 `<SG6>`

---

## 已备好的纯模块

这些模块不访问底盘，已可编译和测试，等四轮接口就绪后接入：

```text
MecanumKinematics          四轮逆解 / 正解 / 归一化 / RPM
WheelVelocityController    四轮独立 PID
MecanumDriveController     车体速度 -> 四轮目标 -> 四轮 PID
MecanumOdometry            四轮正解 + IMU 航向优先 / 掉线回退
```

它们当前不在主循环中，因此不会增加现有路径的运行时开销。

## 建议实施顺序

```text
1. 数齿、量轮径/轮距/轴距，确定四轮端口和方向
2. 底盘增加四轮功率/速度接口和四轮反馈
3. 实现麦轮逆解 + 正解 + 归一化
4. 四轮速度闭环、死区、加速度限制、背隙补偿
5. 手柄/仿真增加横移
6. 宏升级为 mecanum 映射，保留旧宏读取
7. IMU 校准、航向融合、打滑/堵转保护
8. 18/24/50 尺寸检查和实机 HIL
```
