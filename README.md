# VEX-yuanshen

基于 PROS V5（kernel 4.2.3）与 [LemLib](https://github.com/LemLib/LemLib)（v0.5.6）的 VEX V5 机器人 C/C++ 工程，包含四电机双侧直轮差速底盘、双电机升降与软件限位、开环夹爪及自动赛入口。

## 功能概览

- **LemLib 底盘与定位**：沿用四电机双侧直轮底盘（左前 2、左后 1，右前 3、右后 4 反转，蓝齿轮 600 rpm）及端口 5 的 IMU；不外接从动定位轮，由 `lemlib::Chassis` 自动通过左右驱动电机组编码器与 IMU 进行里程计解算与闭环运动控制。
- **手动驾驶**：在 `opcontrol()` 中以 20ms 周期通过 `io::Controller` 读取手柄，左摇杆前后与右摇杆转向经 `lemlib::ExpoDriveCurve` 指数曲线和 `lemlib::Chassis::arcade()` 混控驱动底盘；按住 **X** 上升、**B** 下降控制升降，按住 **R1** 张开、**R2** 闭合控制夹爪。
- **升降与软件限位**：9、10 号口使用绿色齿轮电机（200 rpm）。Brain 屏幕只保留升降限位测量界面（`[SET LOW]` / `[SET UP]` / `[USE SAVED]`），将限位行程保存到 SD 卡（`/usd/lift_limits.csv`）；每次上电将升降置于原下端后点击 `[USE SAVED]` 恢复参考；自动赛额外提供基于 `lemlib::PID`、`lemlib::Timer` 和 `lemlib::ExitCondition` 的 `lifting::move_to()` 与 `lifting::move_for()` 辅助接口。
- **自动赛入口**：`src/autons.cpp` 仅保留唯一公开函数 `autons::run()`，由 `autonomous()` 调用；可在其中通过 LemLib Path Planner 与 `C++ Custom` 组装自动路线，默认保持停车。

> 机构接线与标定步骤见 [机构模块](docs/mechanisms.md)。赛制与编程规范见 [AGENTS.md](AGENTS.md) 与 [规则索引](docs/rules/README.md)。

## 代码结构

```text
include/
  autons.hpp             自动赛唯一公开函数 autons::run() 声明
  brain_ui.hpp           Brain LLEMU 升降限位测量界面
  chassis.h              基于 LemLib 的四电机底盘与里程计控制接口
  claw.hpp               夹爪开环接口（手动与自动）
  io/
    controller.hpp       手柄抽象基类 io::Controller
    pros_controller.hpp  PROS 手柄适配器工厂声明
  lifting.hpp            双电机升降、软件限位与 LemLib 辅助闭环接口
src/
  main.cpp               PROS 五个竞赛回调与 20ms 主循环调度
  autons.cpp             自动赛唯一公开函数 autons::run() 实现
  brain_ui.cpp           Brain LLEMU 升降限位显示与点动标定逻辑
  chassis.cpp            四电机端口 (2,1 / -3,-4)、端口 5 IMU 与 LemLib 底盘参数配置
  claw.cpp               夹爪开环电机（端口 8 占位）
  io/pros_controller.cpp ProsController 继承实现
  lifting.cpp            双绿齿轮升降电机、编码器限位、SD 存取与 LemLib PID/Timer 运动
static/
  example.txt            LemLib Pure Pursuit 路径资源示例
```

## 环境配置与构建

在仓库根目录执行环境检测与源码检查：

```bash
python tools/configure_env.py
python tools/configure_env.py --check
```

格式化、编译与上传：

```bash
clang-format -i <本次修改的自有文件>
clang-format --dry-run --Werror <本次修改的自有文件>
pros build all   # 或直接使用 make all -j4
pros upload
pros terminal
```
