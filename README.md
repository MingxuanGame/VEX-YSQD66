# VEX-yuanshen

基于 PROS V5（kernel 4.2.3）的 VEX V5 机器人 C/C++ 工程，包含四电机 Arcade 底盘、双电机升降与 Brain 触屏宏录放。自动赛默认加载空白路线模板并保持停车。

## 功能概览

- **手动底盘驾驶**：在 Home 界面下以 20ms 周期读取手柄，通过左摇杆前后与右摇杆转向混控驱动四电机蓝齿轮底盘，并在底盘层统一做左右偏置补偿。
- **升降与软件限位**：9、10 号口使用绿色齿轮电机。通过 Brain 的 `LIFT LIMITS` 测定上下限并保存到 SD，开机和进入自动赛时加载。每次上电在原下端点按 `USE SAVED` 恢复参考。返回 Home 后按住 X 上升、按住 B 下降，宏录制保存限位处理后的升降功率。
- **Brain 触屏宏录放**：脱离场控且启用手动模式时，支持 3 秒倒计时、20ms 一帧共 725 帧（14.5 秒）录制、末尾 0.5 秒停车重置及 SD 卡保存（`/usd/macro_NNNNNN.csv`）。
- **检查点续录**：录制中按 L1 添加检查点；加载已有宏后可从任意检查点回放前缀、停车 3 秒再续录剩余帧并另存新编号。
- **反馈纠偏**：每帧记录端口 5 的 IMU 连续转角与四电机编码器角度，回放通过编码器与 IMU 的 PID 叠加修正功率；增益使用 `constexpr` 定义，初值待实机标定。上电停车校准 IMU（3 秒超时），状态重置时清零转角参考。
- **Brain 测量工具**：从主屏幕进入，按屏幕步骤试跑或填写外部实测值，结果显示对应的网页标定字段和单位。
- **显示独占**：`Makefile` 将 `liblvgl.a` 排除出冷包，由 `pros::screen` 独占绘制按钮与计时页面，防止 LVGL 后台任务覆盖界面。

> 详细协议、CSV 格式与实机待验证清单见 [宏录制协议](docs/macro-protocol.md)。标定步骤和接线见 [机构模块](docs/mechanisms.md)。赛制与编程规范见 [AGENTS.md](AGENTS.md) 与 [规则索引](docs/rules/README.md)。

## 代码结构

```text
include/
  calibration/           测量项目、试跑、键盘和会话接口
  chassis.h              底盘控制接口
  lifting.hpp            双电机升降与软件限位
  claw.hpp               夹爪开环接口
  io/
    controller.hpp       手柄抽象基类 io::Controller
    pros_controller.hpp  PROS 手柄适配器工厂声明
  math/pid.hpp           带积分限幅与饱和抑制的离散 PID
  macro/
    engine.hpp           硬件抽象 macrorec::Hardware 与非阻塞状态机 Engine
    screen.hpp           Brain 触屏绘图与触摸轮询
    session.hpp          竞赛回调接入的会话调度入口
    storage.hpp          v5 CSV 读写、v4 兼容与目录编号解析
src/
  main.cpp               PROS 五个竞赛回调与主循环调度
  autonomous/            自动赛路线模板、曲线跟踪、里程计与动作执行
  calibration/           标定试跑、数据换算与 Brain 界面
  chassis.cpp            四电机端口 (2,1 / -3,-4)、蓝齿轮、偏置补偿与端口 5 的 IMU
  lifting.cpp            双绿齿轮升降电机、编码器限位与标定点动
  claw.cpp               夹爪开环电机
  io/pros_controller.cpp ProsController 继承实现
  math/pid.cpp           PID 计算与状态重置
  macro/                 engine / screen / session / storage 及中文字形数据
```

## 编辑自动路线

在网页编辑和校验路线，点击「导出 C++」。将导出的完整 `program.cpp` 放到车辆工程的 `src/autonomous/program.cpp`，再运行 `pros build`。[Override Auto Studio](https://override-auto-studio.vercel.app/)

## Brain 测量工具

在 Brain 主屏幕点击「测量工具」，选择项目并按屏幕说明操作。带动力的试跑需要脱离场控并启用手动。按住手柄 A 两秒开始，持续按到试跑结束。松开 A、点击停止、手柄掉线或模式切换都会中止试跑并刹车。

建议先测每转行程和静摩擦，再测匀速、加速与 PID。测得的每转行程、kS、kV、巡航速度和停车容差会在本次开机期间预填到相关项目。每转行程用尺量地面距离，停车位置误差用尺或俯拍测量。横向偏差、滑行量和最小弯道均使用外部实测值。

| Brain 项目 | 结果字段 | 网页「PID / 标定」位置 |
| --- | --- | --- |
| Travel per rev | `travel_per_rev` | 每电机轴转行程 |
| Static power kS | `static_power` | 静摩擦 kS |
| Cruise kV / trim | `velocity_gain`、`left_trim`、`cruise_speed` | 速度增益 kV、左右动力微调、巡航速度 |
| Acceleration kA | `acceleration_gain`、`acceleration` | 加速度增益 kA、加速 |
| Heading PID / Speed PID | `heading_p/i/d`、`speed_p/i/d` | 航向 PID、速度 PID，填入起调值后复跑 |
| Stop tolerances | `position_tolerance`、`heading_tolerance` | 停车容差、航向容差 |
| Tracking / gap | `tracking_limit`、`clearance` | 偏差急停、几何余量 |
| Lookahead / Min turn radius | `lookahead`、`min_radius` | 跟踪前视、最小转弯半径 |

## 环境配置与构建

在仓库根目录执行环境检测与源码检查：

```bash
python tools/configure_env.py
python tools/configure_env.py --check
```

脚本会自动查找本机的 `clangd`、`arm-none-eabi-gcc/g++`、`make` 与可选的 `pros` CLI，生成 `compile_commands.json`、`toolchain.local.json` 和 `.vscode/` 本机配置（均已在 `.gitignore` 中忽略）。可用 `--clangd`、`--cc`、`--cxx`、`--pros` 显式指定路径。

编译、格式化与上传：

```bash
clang-format -i <本次修改的自有文件>
clang-format --dry-run --Werror <本次修改的自有文件>
pros build all   # 或直接使用 make all -j4
pros upload
pros terminal
```

- 修改 `Makefile` 的冷包排除项后，请使用 `pros build all`（或 `make all`）重新编译冷包和热包并上传。
- VS Code 可直接使用 `.vscode/tasks.example.json` 同步生成的 `PROS: Build`、`PROS: Rebuild`、`PROS: Check sources` 与 `PROS: Upload` 任务。

## 常见问题

- **找不到工具**：将工具链 `bin` 目录加入 `PATH`，或通过 `configure_env.py --clangd/--cc/--cxx/--pros` 指定路径。
- **clangd 找不到头文件或仍用旧路径**：重新运行 `python tools/configure_env.py`，并在 VS Code 中执行 `clangd: Restart language server`。
- **`pros c fetch` 提示模板路径不存在**：`project.pros` 中记录的模板 `location` 可能来自原开发机，不影响 `pros build` / `make` 构建；仅需更新模板时重新获取对应 kernel。
