# Simulation binaries

`vex_sim_*` 的可执行文件统一放在这里，避免和 CMake 中间文件混在一起。

| 文件 | 用途 |
|---|---|
| `vex_sim_ui.exe` | 赛场 + 手柄 + 机构仿真 UI |
| `vex_sim.exe` | 底盘物理 / 碰撞演示 |
| `vex_sim_tuning.exe` | 调参链路演示 |
| `vex_sim_motion.exe` | 运动控制 PID / 90° 转向演示 |
| `vex_tests.exe` | Host 单元测试 |

运行：

```powershell
.\sim\bin\vex_sim_ui.exe
```

CMake 目标已设置 `RUNTIME_OUTPUT_DIRECTORY` 到 `sim/bin`：

```powershell
cmake --build --preset host-debug --target vex_sim_ui
```

UI 键位：

```text
方向键   DRIVE：左摇杆 8 方向；TUNE：参数导航
W/S/Q/E  TUNE：按住 L1 时低速点动
I/J/K/L  手柄方向键
A/B/X/Y  手柄四色按钮
1/2      L1/L2
3/4      R1/R2
长按 Y   进入 TUNE
ESC      退出
```
