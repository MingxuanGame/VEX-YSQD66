# VEX-yuanshen

基于 PROS 的 VEX V5 机器人 C/C++ 工程。

## 当前阶段

- PC 仿真、控制层、碰撞/调参测试可用；`sim/bin` 下有最新可执行文件。
- 真机支持手动底盘和 Brain 触屏宏录制/回放：3 秒准备、20ms 一帧、14.5 秒动作和 0.5 秒停止重置，编号递增保存到 SD。当前仅转换/执行底盘指令，其他机构留 TODO；`autonomous()` 留空。该模块与仿真 `ControlLoop` 独立。
- 仿真路线文件自动赛、多车动态碰撞尚未实现。
- 在硬件方案确定前，请把本仓库当作“仿真 + 控制层验证工程”，不要直接当作比赛固件上传。

启动后 Brain 显示“新建宏录制”和“使用宏”。录制/回放限于脱离场控且启用的手动练习；宏文件格式、微调示例、停止行为和机构扩展点见 [宏录制协议](docs/macro-protocol.md)。Home 页面可正常驾驶，列表和结果页保持停车。

真机适配器 `ProsChassis` / `ProsImu` / `ProsMechanism` 目前只是**精简接线级**，未接入主循环，也不是比赛级安全实现，详见 [真机适配器状态](docs/hardware-adapters.md)。

其他组员可以修改底盘和宏模块，本边会同步适配；接口边界见 [协作边界](docs/collaboration.md)。后续优先级见 [方向清单](docs/roadmap.md)。

## 开发规范与编辑器配置

- `.clang-format`：C/C++ 四空格缩进、大括号不换行、函数间空行；`.clang-format-ignore` 为独立 clang-format 排除 SDK、固件和生成物，编辑器中也仅格式化自有代码。
- `.editorconfig`：UTF-8、LF、文件末尾换行；Makefile 保留 Tab，JSON 使用两空格。
- `.gitattributes`：统一 Git 文本换行为 LF，固件、归档和图片按二进制处理。
- `.clang-tidy`：检查部分命名规则、命名空间导入及常见误用；端口和比赛行为仍需代码检视及实机验证。
- `.vscode/settings.example.json`：通过 clangd 按仓库规则保存时格式化 C/C++，关闭自动推断缩进。运行下方配置脚本后同步到本机设置。

按 VS Code 扩展建议安装 clangd 和 EditorConfig 扩展。使用独立格式化命令时，需要 `clang-format`；VS Code 的 clangd 可直接提供格式化。命名诊断由 clangd 的 clang-tidy 集成提供，也可使用独立 `clang-tidy`。这些配置已用 LLVM 22 验证，旧版本出现未知选项时先核对工具版本。

仅对本次修改的自有 C/C++ 文件执行格式化，例如：

```powershell
clang-format -i src/chassis.cpp include/chassis.h
clang-format --dry-run --Werror src/chassis.cpp include/chassis.h
```

## 自动配置环境

在项目根目录执行：

```powershell
python tools/configure_env.py
```

脚本从当前电脑查找 clangd、ARM GCC/G++ 和可选的 PROS CLI，打印找到的路径并生成本地配置。查找顺序为：命令行指定的路径、当前 PATH、常见安装目录、已有本地配置中的有效路径。

常见目录包括 Windows 的 MSYS2、LLVM、PROS、部分 WinGet 安装目录和 VS Code 扩展工具目录，以及 Linux/macOS 的系统 bin、LLVM 和 Homebrew 目录。脚本只搜索指定目录，不扫描整块磁盘。自定义位置未被发现时，将工具的 bin 目录加入 PATH，或直接指定：

```powershell
python tools/configure_env.py --clangd "D:/LLVM/bin/clangd.exe" --cc "D:/toolchain/bin/arm-none-eabi-gcc.exe" --cxx "D:/toolchain/bin/arm-none-eabi-g++.exe"
```

可用 `--pros PATH` 指定 PROS CLI。未找到 PROS CLI 时仍可配置 clangd 并直接用 `make` 构建；只有上传/终端需要 CLI。

配置并检查所有源文件：

```powershell
python tools/configure_env.py --check
```

脚本验证编译器目标为 ARM 裸机平台，读取当前 Makefile 的目标参数、预处理宏和语言标准，再用 GCC 查询对应 CPU/FPU 的标准库头文件目录。`--check` 调用 clangd 检查 `src`、`sim`、`tests` 下的源文件；clangd 22 偶尔会在内部 tweak（如 ExtractFunction）上失败，脚本会识别这类“源码 0 错误、仅 tweak 失败”的情况并打印 warning，不会误报为源码错误。

在 VS Code 中打开项目文件夹，然后执行 `clangd: Restart language server`。

## 编译和上传

VS Code 的 `PROS: Build`、`PROS: Rebuild`、`PROS: Clean` 任务直接调用本机检测到的 `make`，并自动带上 ARM 工具链和 `PROS_TOOLCHAIN`，不依赖 PROS CLI。`PROS: Upload` / `PROS: Terminal` / `PROS: Build & Upload` 仍然需要 PROS CLI；`PROS: Update compile_commands` 会重新运行配置脚本。

`PROS: Check sources` 会运行 `tools/configure_env.py --check`，用于配置并检查所有源文件。

若 PROS 已在终端 PATH 中，也可以直接使用：

```powershell
pros build
pros build all
pros upload
pros terminal
```

PROS CLI 通过 `PROS_TOOLCHAIN` 查找 `usr/bin` 下的 `arm-none-eabi-*` 和 `make`；`tools/configure_env.py` 会自动写入这个变量。

## 模块与接口

- `robot::Chassis` / `robot::Imu` / `robot::Clock` 是业务层唯一依赖的硬件接口，定义在 `include/robot/`。
- 真机实现留在 `src/`（`chassis.cpp` 属于硬件层，负责端口、齿轮和电机方向）；仿真实现是 `sim/compat/sim_robot.hpp`。
- 碰撞检测、碰撞恢复通过构造函数注入 `robot::Chassis` / `robot::Imu`，不再直接包含 `pros/*`，因此同一份逻辑可以在真车和 PC 上运行。
- 调参参数通过 `ParamApply::bind("参数名", handler)` 绑定到控制器，再由 `ParamApply::applyAllDirty()` 推送；未绑定的参数会保持 dirty。
- 运动控制层在 `include/motion/`：`ChassisKinematics`（里程计）、`TrapezoidProfile`（规划）、`VelocityController`（轮速 PID）、`MotionEngine`（非阻塞状态机）。
- `MotionTuning` 把 `vel_kp / vel_ki / vel_kd` 注册到 `ParamsManager`，再通过 `ParamApply` 绑定到具体 `MotionEngine`。
- `robot::Battery` 抽象用于电压补偿；真机实现是 `src/robot/battery.cpp`，仿真用 `SimBattery`。

## 运动控制层

```text
include/motion/
  chassis_kinematics.hpp   // 编码器差分 + IMU 融合，输出 Pose
  motion_profile.hpp       // 复用 vxmath::TrapezoidProfile
  pid.hpp                  // dt 防零、积分限幅、输出限幅
  velocity_controller.hpp  // 左右轮速度闭环
  motion_engine.hpp        // Idle/Driving/Turning/Waiting/Fault 非阻塞状态机
```

`MotionEngine` 每 10ms 调一次 `update()`，不会阻塞主循环；`startDrive/startTurn` 自动计算 deadline，超时进入 `Fault` 并刹车。断联时主控调用 `update(now, false)` 即可立即停车。

当前 `vex_sim_motion` 扫参得到的默认速度环参数：

```text
vel_kp = 2.2
vel_ki = 0.0
vel_kd = 0.02
```

配合 `kAngleToleranceDeg = 0.2`，90° 转向在仿真中落在 89.8°~90.2° 内。

## 主控调度层

`include/control/` 下是 50Hz 主控：

```text
mode_manager.hpp        // DRIVE / TUNE / AUTO / SAFE_WAIT
drive_controller.hpp    // 死区、指数、差速、脱困、防推挤、Assist、电流预算
mechanism_controller.hpp// R1 夹爪 / R2 推板 / L1/L2 抬升
control_loop.hpp        // 读输入 -> 模式 -> 驾驶/自动/安全等待 + UI 调度
```

- `ModeManager`：长按 Y 500ms 进入 TUNE；B 退出并触发参数下发；断开/故障进入 SAFE_WAIT，A/B 恢复。
- `DriveController`：摇杆死区 + 指数曲线，蓝齿轮输出限制 100；L1+L2 脱困输出限制 60；高电流零速度且操作手未硬推时自动卸力，猛推立即让位。
- `MechanismController`：Drive 模式下 R1 切换夹爪、R2 切换推板、L1/L2 手动抬升；L1+L2 优先让给底盘脱困。
- `Assist`：按住 X 锁住当前航向并自动修 turn；视觉检测到锥体后可通过 `setAssistTargetHeading()` 注入目标航向。
- `ControlLoop`：每 20ms 调一次；断联或 SAFE_WAIT 强制全轮刹车；AUTO 阶段完全屏蔽摇杆，交给 `MotionEngine`。
- `ControlLoop::endAuto()`：先硬刹车、停 MotionEngine 并清 PID 残留，再交还 Drive。
- `ControlLoop::renderUI()`：只在 TUNE 模式以 100ms 周期刷新屏幕，显示当前选中参数；Tune 内可用上下选参数、左右切组、X/Y 微调、A 保存。
- 软件看门狗：主循环间隔超过 100ms 时强制刹车并进入 SAFE_WAIT；MotionEngine Fault 也会收敛到 SAFE_WAIT。
- 碰撞检测提供非阻塞 `startObservation()/updateObservation()`，碰撞脱困提供 `startEscapeAsync()/updateEscapeAsync()`；50Hz 主循环不要调用旧的 `detect()`/`escapeCollision()`。
- 手动模式也做低电压补偿，补偿后再走全局电流预算。
- 全局电流预算：四轮电流 + `Mechanism::getCurrentMa()`，超过 `max_total_current_ma` 时等比例缩小两轮输出，保留转向比例。
- 自动赛仍保留断联刹车与 L1+L2 脱困；自动赛脱困松手后进入 SAFE_WAIT。
- 真机屏幕通过 `robot::Display` 抽象接入；`robot::prosDisplay()` 是 `pros::lcd` 适配器，仿真/测试可用 FakeDisplay。

## PC 仿真

`CMakeLists.txt` 会构建 `vex_sim`（完整仿真）、`vex_sim_tuning`（只测调参链路）、`vex_sim_motion`（运动控制闭环）、`vex_sim_ui`（赛场+手柄 UI）和 `vex_tests`：

```powershell
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug --output-on-failure
.\sim\bin\vex_sim.exe
.\sim\bin\vex_sim_tuning.exe
.\sim\bin\vex_sim_motion.exe
.\sim\bin\vex_sim_ui.exe
```

可执行文件统一输出到 `sim/bin`，不放在 CMake 中间目录。

`vex_sim_ui` 会打开一个比赛场地 + 手柄窗口：

- 启动后先进入放置模式：鼠标点击/拖拽选择底盘位置，拖拽过程不写轨迹；`Enter` 或点击 `START` 后锁定并开始仿真。
- 左侧 12ft × 12ft 场地：灰色地垫、2ft 网格、9 个得分座、8 个围边节点、4 个三色筒、4 个导入桶、自动分界线、4 组起始锥和底盘轨迹。
- 得分座与 4 组起始锥都有碰撞体；撞锥会先减速，推得动时锥会被推开并解除碰撞。
- `ConeField` 负责锥的位移、摩擦和出界状态；出界的锥在 UI 上显示红叉。
- 右侧：VEX 手柄、模式/位姿/速度/电流/机构状态。
- 顶部比赛提示：`AUTO 15s | DRIVE 1:45`。
- `R` 或场上的 `RESTART` 按钮会复位时钟、电池、机构、轨迹并回到放置模式。

```text
方向键   -> DRIVE：左摇杆 8 方向；TUNE：参数导航
W/S/Q/E  -> TUNE：按住 L1 时低速点动
I/J/K/L  -> 手柄方向键
A/B/X/Y  -> 手柄 A/B/X/Y
1/2      -> L1/L2
3/4      -> R1/R2
Enter    -> 放置模式下确认开始
R        -> 重启并回到放置模式
ESC      -> 退出
```

鼠标操作：

```text
点击/拖拽场地    -> 选择底盘位置（仅放置模式）
点击 START       -> 锁定位置并开始仿真
点击 RESTART     -> 重启并重新放置
```

窗口底部同时带一个 Tune 演示：Tune 下用方向键切参数，按住 L1 再用 W/S/Q/E 低速点动；X/Y 调整、A 保存。图形摇杆只做左摇杆 8 方向；右摇杆暂未实现。

界面测试截图在 [sim/ui/sim-ui-instances](sim/ui/sim-ui-instances/)。

`vex_tests` 覆盖：

- `test_tuning.cpp`：参数注册、序列化、默认值、ParamApply
- `test_collision.cpp`：碰撞检测、恢复、AntiPush
- `test_motion.cpp`：里程计、PID、MotionTuning、状态机
- `test_control.cpp`：模式切换、断联急停、脱困、防推挤
- `test_library.cpp`：vxmath、io、Robot、碰撞调参
- `test_sim.cpp`：电池压降、确定性噪声、底盘位姿积分、锥体碰撞与机构
- `test_integration.cpp`：物理仿真 + ControlLoop + MotionEngine + 调参的端到端测试
- `test_macro.cpp`：底盘指令转换、3 秒屏蔽、725 帧录放、500ms 重置、取消/断联/故障、CSV 校验与禁止覆盖

`CMakePresets.json` 使用 PATH 中的 `g++` 和 `ninja`，不再写死本机绝对路径；换机时把这两个工具放进 PATH 即可。也可以手动指定：

```powershell
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build
ctest --test-dir build --output-on-failure
```

仿真通过 `sim::SimChassis` / `sim::SimImu` 注入业务层，`sim::delay()` 会推进虚拟时钟和物理引擎，不会占用真实等待时间。

当前仿真增强：

- 确定性 RNG：`Motor` / `Imu` / `ChassisSim` 都支持 `setRandomSeed()`，同一场景可复现。
- 电池内阻压降：`sim::Battery` 支持开路电压、内阻和负载电流，低电压 + 大电流会真实掉压。
- 底盘位姿积分：`ChassisSim` 提供 `getPoseXMeters/YMeters/HeadingDeg()`，可用于回放和定位测试。
- 总电流统计：`ChassisSim::getTotalCurrentMa()` 汇总四轮，并反馈给电池模型。
- 机构仿真：`SimMechanism` 模拟三级级联抬升、舵机夹爪角度和推板行程；参数集中在 `sim/config/sim_config.hpp` 的 `SimConfig`。

机构默认配置：

```cpp
struct SimConfig {
    LiftSimConfig lift;   // 级联级数、每级高度、最大速度、电流
    ClampSimConfig clamp; // 舵机开合角度、角速度
    PushSimConfig push;   // 推板行程时间
};
```

拿到实机后只改 `defaultSimConfig()` 里的端口、级数、高度和速度，不需要修改 `SimMechanism`。UI 会显示：

```text
lift=0.11m  clamp=150  push=1.00
```


## 常见问题

- **找不到工具**：安装对应工具、添加 bin 目录到 PATH，或用 `--clangd` / `--cc` / `--cxx` / `--pros` 指定路径。
- **找不到头文件**：重新生成配置，确认 ARM 工具链带有标准库头文件。不要从其他机器复制编译数据库。
- **clangd 无法启动或仍使用旧路径**：查看 VS Code 的 clangd 输出面板，重新配置并重启语言服务器，检查用户或工作区设置是否覆盖项目配置。
- **Make 或 shell 缺失**：补全 PROS 构建环境；clangd 检查通过不代表完整固件构建环境已就绪。
- **PROS CLI 未安装**：`PROS: Build/Rebuild/Clean` 仍然可用（直接调用 make）；上传/终端需要 PROS CLI。配置脚本会优先使用 `%LOCALAPPDATA%\Programs\pros-cli\Scripts\pros.exe`，也可以通过 `--pros PATH` 指定。`project.pros` 里的 template `location` 是原机器的绝对路径，`pros build` 不受影响，但 `pros c fetch`/模板升级可能需要重新安装 kernel/liblvgl 模板。
