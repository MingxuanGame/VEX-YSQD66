# 协作边界：底盘与宏模块

## 代码规范优先级

已确定采用方案 A：

```text
AGENTS.md / .clang-format / .clang-tidy 优先
培训 PPT 作为背景材料，不作为逐条验收标准
```

因此：

- 新模块继续使用 `snake_case` 函数和普通变量
- 已有 `camelCase` 模块保留原风格，不做全仓重命名
- 全局可变对象使用 `_g_` 前缀
- 私有/保护成员使用 `m_`
- 常量使用 `kPascalCase`
- 列宽使用项目 `.clang-format` 的 100 列
- 4 空格缩进、K&R 大括号、函数间空行

不为了和 PPT 的字面示例一致而制造大面积无关 diff。

## 结论

其他组员可以继续修改底盘和宏录制逻辑，本边不把这些文件视为“只能由我们修改”。

允许对方修改：

```text
include/chassis.h
src/chassis.cpp
src/macro/*
```

本边会同步更新：

```text
include/robot/pros_chassis.hpp
src/robot/pros_chassis.cpp
include/robot/pros_imu.hpp
src/robot/pros_imu.cpp
include/robot/pros_mechanism.hpp
src/robot/pros_mechanism.cpp
src/control/*
tests/*
```

不会因为底盘接口变化而回退对方的改动；只更新适配层和测试。

## 底盘接口约定

当前手动/宏控制的唯一底盘输出入口是：

```cpp
chassis::set_power(int left_power, int right_power);
chassis::stop();
```

新的机械方案是 4 个独立麦轮，`set_power()` 的左右分组已经不够。建议底盘模块增加四轮接口，同时保留旧入口作兼容包装：

```cpp
enum class Wheel { FrontLeft, FrontRight, BackLeft, BackRight };

bool set_wheel_power(int fl, int fr, int bl, int br);
bool set_wheel_velocity(int fl_rpm, int fr_rpm, int bl_rpm, int br_rpm);

struct WheelState {
    double current_ma;
    double velocity_rpm;
    double position_deg;
    bool valid;
};

struct State {
    WheelState wheel[4];
    bool valid;
};

bool read_state(State &out);
bool tare_position();
```

驱动、反馈和 tare 的实际实现由底盘维护者决定；本边按新接口更新 `ProsChassis`、麦轮运动学和测试。

四轮接口的完整参考样板在 [`sim/ref/`](sim/ref/README.md)：
`chassis_reference.hpp.ref` / `chassis_reference.cpp.ref`。它们不参与编译，不能直接覆盖生产文件。

后续如果需要：

- ControlLoop 防推挤
- 速度/位置闭环
- 物理编码器 tare
- 电流/温度保护

建议底盘模块**增加只读接口**，而不是删除 `set_power()`：

```cpp
struct State {
    double left_current_ma;
    double right_current_ma;
    double left_velocity_rpm;
    double right_velocity_rpm;
    double left_position_deg;
    double right_position_deg;
    bool valid;
};

bool read_state(State &out);
bool tare_position();
```

新增接口不会破坏宏录制；如果必须改 `set_power()` 的签名，请在同一提交更新本说明，本边再同步改适配器。

## 控制权

同一时刻只能有一个底盘写入者：

```text
macrorec 手动/宏回放
或
ControlLoop
```

目前 `src/main.cpp` 由 `macrorec` 接管。ControlLoop 还没有接入；接入前必须先把控制权规则写清楚，不能后台另起任务直接写 `set_power()`。

## 注意事项

- 不要在适配器里重复构造 `pros::Motor` / `MotorGroup`
- 不要把 `move()` 的 [-127,127] 和 `move_voltage()` 的 mV 混用
- 改变端口、方向或齿轮箱时同步更新端口表
- 新增底盘反馈后，`ProsChassis` 的 getter 才能从 0 占位改为真实值
- 场控、禁用、自动阶段的拒绝逻辑仍由底盘/上层任一侧保留，不能绕过
