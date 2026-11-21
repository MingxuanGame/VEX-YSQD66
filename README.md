# VEX-yuanshen

基于 PROS V5（kernel 4.2.3）与 [LemLib](https://github.com/LemLib/LemLib)（v0.5.6）的 VEX V5 机器人 C/C++ 工程，包含四电机双侧直轮差速底盘、双电机升降、气动夹爪及自动赛入口。

## 功能概览

- **LemLib 底盘与定位**：沿用四电机双侧直轮底盘（左前 2、左后 1，右前 3、右后 4 反转，蓝齿轮 600 rpm）及端口 5 的 IMU；不外接从动定位轮，由 `lemlib::Chassis` 自动通过左右驱动电机组编码器与 IMU 进行里程计解算与闭环运动控制。
- **手动驾驶**：在 `opcontrol()` 中以 20ms 周期通过 `io::Controller` 读取手柄，左摇杆前后与右摇杆转向经 `lemlib::ExpoDriveCurve` 指数曲线和 `lemlib::Chassis::arcade()` 混控驱动底盘；按住 **L2** 上升、**L1** 下降控制升降。程序启动时夹爪保持夹紧，场控启动自动赛、手动按 **A** 启动自动路线或按 **R1** 时解锁。手动按住 **R1** 夹紧，松手放开。
- **Brain 触屏界面**：第一排选择自动赛预设（`RED LEFT` / `RED DOWN` / `BLUE RIGHT` / `BLUE UP`），第二排选择 `Plan 1` 或 `Plan 2`。开机默认 `RED LEFT` 与 `Plan 1`。
- **自动赛入口**：`autons::run()` 根据所选预设和方案运行路线。以 `RED LEFT` 为例，`Plan 1` 调用 `run_red_left()`，`Plan 2` 调用 `run_red_left_2()`。四个 `_2` 函数待填入路线动作。

> 机构接线与标定步骤见 [机构模块](docs/mechanisms.md)。赛制与编程规范见 [AGENTS.md](AGENTS.md) 与 [规则索引](docs/rules/README.md)。

## 代码结构

```text
include/
  autons.hpp             自动赛唯一公开函数 autons::run() 声明
  brain_ui.hpp           Brain 触屏自动预设选择界面
  chassis.h              基于 LemLib 的四电机底盘与里程计控制接口
  claw.hpp               气动夹爪接口（手动与自动）
  io/
    controller.hpp       手柄抽象基类 io::Controller
    pros_controller.hpp  PROS 手柄适配器工厂声明
  lifting.hpp            双电机升降功率与角度控制
  toggle.hpp             拉翻色桶机构接口
src/
  main.cpp               PROS 五个竞赛回调与 20ms 主循环调度
  autons.cpp             自动赛方案调度与各预设的路线函数
  brain_ui.cpp           LVGL 触屏预设与方案按钮
  chassis.cpp            四电机端口 (2,1 / -3,-4)、端口 5 IMU 与 LemLib 底盘参数配置
  claw.cpp               单气缸夹爪（Brain 三线 E 口数字输出）
  io/pros_controller.cpp ProsController 继承实现
  lifting.cpp            双电机升降、手动功率与自动角度控制
  toggle.cpp             拉翻色桶机构（Brain 三线 D 口数字输出）
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
