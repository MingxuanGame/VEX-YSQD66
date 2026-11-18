# VEX-yuanshen

基于 PROS V5（kernel 4.2.3）的 VEX V5 机器人 C/C++ 工程。当前实车实现四电机 Arcade 底盘驾驶与 Brain 触屏宏录制/回放；`autonomous()` 留空。抬升和夹爪提供独立的开环控制模块，尚未接入主循环。

## 功能概览

- **手动底盘驾驶**：在 Home 界面下以 20ms 周期读取手柄，通过左摇杆前后与右摇杆转向混控驱动四电机蓝齿轮底盘，并在底盘层统一做左右偏置补偿。
- **Brain 触屏宏录放**：脱离场控且启用手动模式时，支持 3 秒倒计时、20ms 一帧共 725 帧（14.5 秒）录制、末尾 0.5 秒停车重置及 SD 卡保存（`/usd/macro_NNNNNN.csv`）。
- **检查点续录**：录制中按 L1 添加检查点；加载已有宏后可从任意检查点回放前缀、停车 3 秒再续录剩余帧并另存新编号。
- **反馈纠偏**：每帧记录端口 5 的 IMU 连续转角与四电机编码器角度，回放通过编码器与 IMU 的 PID 叠加修正功率；增益使用 `constexpr` 定义，初值待实机标定。上电停车校准 IMU（3 秒超时），状态重置时清零转角参考。
- **显示独占**：`Makefile` 将 `liblvgl.a` 排除出冷包，由 `pros::screen` 独占绘制按钮与计时页面，防止 LVGL 后台任务覆盖界面。

> 详细协议、CSV 格式与实机待验证清单见 [宏录制协议](docs/macro-protocol.md)；抬升与夹爪的精简开环标准见 [机构模块](docs/mechanisms.md)；赛制与编程规范见 [AGENTS.md](AGENTS.md) 与 [规则索引](docs/rules/README.md)。

## 代码结构

```text
include/
  chassis.h              底盘控制接口
  lift.hpp               抬升开环接口
  claw.hpp               夹爪开环接口
  io/
    controller.hpp       手柄抽象基类 io::Controller
    pros_controller.hpp  PROS 手柄适配器工厂声明
  math/pid.hpp           带积分限幅与饱和抑制的离散 PID
  macro/
    engine.hpp           硬件抽象 macrorec::Hardware 与非阻塞状态机 Engine
    screen.hpp           Brain 触屏绘图与触摸轮询
    session.hpp          竞赛回调接入的会话调度入口
    storage.hpp          v4 CSV 读写与目录编号解析（旧版宏需重新录制）
src/
  main.cpp               PROS 五个竞赛回调与主循环调度
  chassis.cpp            四电机端口 (2,1 / -3,-4)、蓝齿轮、偏置补偿与端口 5 的 IMU
  lift.cpp               抬升开环电机
  claw.cpp               夹爪开环电机
  io/pros_controller.cpp ProsController 继承实现
  math/pid.cpp           PID 计算与状态重置
  macro/                 engine / screen / session / storage 及中文字形数据
```

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
