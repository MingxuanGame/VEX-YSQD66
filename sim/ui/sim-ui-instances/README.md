# Sim UI instances

`vex_sim_ui` 的界面测试截图，便于后续对比 UI 改动。

| 文件 | 说明 |
|---|---|
| `01-field-and-controller.png` | 12ft × 12ft 场地 + 手柄的初始界面 |
| `02-mechanism-state.png` | 抬升 / 夹爪 / 推板状态显示 |
| `03-joystick-forward.png` | 方向键 `↑`，左摇杆前进 |
| `04-joystick-diagonal.png` | 方向键 `↑ + →`，左摇杆右前方向 |
| `05-tune-jog.png` | TUNE 模式下按住 L1 + W/S/Q/E 低速点动 |
| `06-real-field.png` | 按比赛俯视图重建后的场地初始画面 |

当前场地是按比赛手册 FO-1 俯视图做的结构占位；拿到真实场地图/CAD 后再调整得分座、三色筒和导入桶的精确坐标。

UI 生成入口：

```powershell
.\sim\bin\vex_sim_ui.exe
```

按键：

```text
方向键   DRIVE：左摇杆 8 方向；TUNE：参数导航
W/S/Q/E  TUNE：按住 L1 时低速点动
I/J/K/L  手柄方向键
A/B/X/Y  手柄四色按钮
1/2      L1/L2
3/4      R1/R2
ESC      退出
```
