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
| `07-placement.png` | 启动后的鼠标放置模式，拖拽不留轨迹 |
| `08-placement-started.png` | 确认位置后开始仿真 |
| `09-restart.png` | R / RESTART 后回到放置模式 |
| `10-mouse-start.png` | 鼠标点击 START 开始仿真 |

当前场地是按比赛手册 FO-1 俯视图和用户批注做的结构复刻；拿到官方 CAD 尺寸后继续调整得分座、三色筒和导入桶的精确坐标。

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
Enter    放置模式确认开始
R        重启并重新放置
ESC      退出
```
