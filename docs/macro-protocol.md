# Brain 底盘宏录制与文件协议 v4

`autonomous()` 当前留空，本模块在手动练习阶段提供 Brain 触屏宏录制、回放与检查点续录功能，使用 PROS V5 4.2.3 接口，无额外后台控制任务。

## 操作流程

上电时停车并初始化端口 5 的 IMU，保持机器人静止完成校准。程序会接续已开始的校准或使用 VEXos 已完成的自动校准，最多等待 3 秒。等待超时后，若 VEXos 随后报告自动校准完成，下次停车清零时可重建转角参考。宏需要有效的 IMU 和编码器反馈，持续失败时检查接线并重启。程序启动、开始录放或动作结束时，统一停止底盘、保持刹车模式并清零四电机编码器和 IMU 转角参考（不执行机械归零、不重新校准 IMU）。检查点暂停只停车，保留原参考。当前仅接入底盘组件；进入 Home 页面可正常驾驶，进入列表或结果页保持停车。

1. 将 FAT32 SD 卡插入 Brain 并开机，Home 页显示“新建宏录制”和“使用宏”。
2. 脱离场控并处于启用的手动模式时，点击“新建宏录制”：先停车并倒计时 **3 秒**（此时屏蔽手柄输入），倒计时结束时重置传感器参考，随后每 **20ms** 在下发功率前采样手柄、IMU 和四电机编码器，共 **725 帧（14.5 秒）**。
3. 录制过程中按下 **L1** 可在当前帧标记检查点（长按只记一次，松手后再按可继续标记）；末尾 **500ms** 自动停车并重置状态，随后保存至 `/usd/macro_NNNNNN.csv`（取现有最大编号加一，上限 `999999`，不覆盖旧文件）。
4. 点击“使用宏”浏览兼容的 v4 宏（每页 4 项），选中后先完整校验文件并进入检查点列表（包含末尾 `END 14.50s`）。v3 及更旧文件缺少反馈，不显示、不回放，需要重新录制。
5. **将机器人放回原录制起点和朝向**，选择某检查点后先倒计时 **3 秒**，从第 0 帧回放至该检查点、停车 **3 秒**再交还手柄续录剩余帧，完成后另存新编号文件；选择 `END` 则只完整回放并停车，不产生新文件。
6. 录放期间点击 `STOP / CANCEL` 可立即取消并丢弃未完成的录制；若 SD 保存失败，完整录制仍留在内存中，插卡后可在结果页点击 `RETRY SD SAVE` 重试。

> 练习边界：Override 2.1 `<R9>`（英文第 61 页）与 `<GG12>`/`<GG13>`（第 46-47 页）要求服从场控并限制自动时段人为干预；接入场控、禁用或进入自动模式时，程序自动拒绝或中止宏录放。

## v4 文件格式

ASCII CSV（无 BOM，LF 或 CRLF 换行，末尾须有换行符，不含注释或空行）。头部必须精确为以下 6 行：

```csv
VEX_MACRO,4
period_ms,20
record_ms,14500
reset_ms,500
mapping,chassis_arcade_imu_encoder_v2
t_ms,left_x,left_y,right_x,right_y,buttons,chassis_left_power,chassis_right_power,imu_rotation_deg,left_front_deg,left_rear_deg,right_front_deg,right_rear_deg,checkpoint
```

其后紧跟 **725 行**帧数据（`t_ms` 从 `0` 递增到 `14480`，步长 `20`），最后一行必须为 `END,725`。

| 字段 | 范围与说明 |
| --- | --- |
| `t_ms` | 毫秒，`0, 20, ..., 14480` |
| `left_x,left_y,right_x,right_y` | 原始摇杆整数 `[-127, 127]`（仅供追溯，回放不重算） |
| `buttons` | 12 位按键掩码 `0..4095`（Bit 0–11 依次为 Up/Down/Left/Right/A/B/X/Y/L1/L2/R1/R2） |
| `chassis_left_power,chassis_right_power` | PROS `move()` 基础功率 `[-127, 127]`，回放叠加 PID 修正后下发 |
| `imu_rotation_deg` | 相对宏起点的 IMU 连续转角，度，顺时针为正；保留多圈角度，不转换为 `0..360` |
| `left_front_deg,left_rear_deg,right_front_deg,right_rear_deg` | 相对宏起点的四电机轴编码器角度，度；已按电机端口反转方向统一符号 |
| `checkpoint` | `0` 或 `1`，标记该帧结束处是否作为可续录检查点 |

五个反馈字段必须是有限数值，拒绝 `NaN`/无穷；以 17 位有效数字保存并逐值回读校验。反馈采样在该帧功率下发前，回放使用相同采样顺序，不等待单帧到位。

说明：PROS 4.2.3 下对不存在文件调用 `fopen(..., "rb")` 会返回 `errno=23`（`ENFILE`），因此真机保存时通过 `pros::usd::list_files()` 目录枚举判重并按 FAT32 忽略大小写比较。

## 底盘映射与离线微调

手动与录制沿用 Arcade 混控方向，经 `chassis::set_power()` 统一做偏置补偿与限幅（当前偏移为 `0`）：

```text
forward = -left_y
turn    = -right_x
left    = forward + turn
right   = forward - turn
scale   = max(127, abs(left), abs(right))
power   = (left * 127 / scale, right * 127 / scale)
```

| 组件 | 智能端口 | 齿轮组 | 方向 | 编码器单位 |
| --- | --- | --- | --- | --- |
| 左底盘电机组（前、后） | 2、1 | 蓝 600rpm | 正向 | degrees |
| 右底盘电机组（前、后） | -3、-4 | 蓝 600rpm | 端口 3、4 反转 | degrees |
| V5 惯性传感器 | 5 | 不适用 | 连续转角，顺时针为正；安装方向待实机确认 | degrees |

回放每 20ms 使用同一帧保存的目标反馈减去当前反馈：左、右侧各取前后电机角度误差的均值，分别输入编码器 PID；IMU 连续转角误差输入航向 PID。输出为：

```text
left  = recorded_left  + left_encoder_pid  + heading_pid
right = recorded_right + right_encoder_pid - heading_pid
```

混控后等比缩放到 `[-127,127]`，再交给底盘层。`src/macro/engine.cpp` 中的 `constexpr` PID 参数目前为待标定初值：编码器 `Kp=0.1`、航向 `Kp=1.0`，`Ki=Kd=0`，各 PID 修正上限为 20 功率；增益和积分限值单位见源码注释。`math::Pid` 支持积分限幅、饱和时阻止同向累积，以及重置后首帧免微分冲击。取消、故障、完整回放结束、检查点暂停都会清除 PID 历史。

开始录放时建立共同零点；检查点暂停及续录保留此零点，尾段记录实际到达位置。IMU 或任一编码器读取失败会中止宏并停车，模式切换仍立即停止。回放以 14.5 秒为上限，不会因未到位延长动作。此纠偏依据相对电机转角和朝向，不测量场地绝对坐标，仍需将车辆放回相同起点和朝向；轮胎打滑引起的平移误差不能由这些传感器完全恢复。

离线微调可修改 `chassis_left_power` 和 `chassis_right_power` 两列（从 0 开始第 6、7 列），并保持精确 725 帧与连续 `t_ms`；反馈列是闭环目标，仅改功率时 PID 仍会尝试跟随原轨迹。更改轨迹应重新录制，或同步规划全部反馈目标。

```python
import csv
from pathlib import Path

path = Path("macro_000001.csv")
rows = list(csv.reader(path.read_text(encoding="ascii").splitlines()))
assert rows[0] == ["VEX_MACRO", "4"] and len(rows) == 6 + 725 + 1
rows[6 + 100][6:8] = ["80", "60"]  # 修改第 100 帧 (t_ms=2000) 的左右功率
with path.open("w", encoding="ascii", newline="") as f:
    csv.writer(f, lineterminator="\n").writerows(rows)
```

## 模块职责与待扩展项

- `include/macro/engine.hpp`、`src/macro/engine.cpp`：抽象接口 `macrorec::Hardware` 与非阻塞状态机 `macrorec::Engine`
- `include/macro/storage.hpp`、`src/macro/storage.cpp`：v4 CSV 读写、编号提取与写后回读校验
- `include/math/pid.hpp`、`src/math/pid.cpp`：不依赖 PROS 的离散 PID
- `include/macro/screen.hpp`、`src/macro/screen.cpp`、`src/macro/labels.inc`：`pros::screen` 触屏菜单与内嵌中文字形
- `include/macro/session.hpp`、`src/macro/session.cpp`：`Components` 硬件适配器、场控检查、SD 读写与主循环调度
- **机构扩展 TODO**：接入抬升/夹爪/推板时，先标定端口与限位，再扩展 `Command`、`map_input()`、`Components` 并升级协议版本。

## 实机待验证项

1. 四轮实际转向、前后编码器顺序与正负方向；5 号端口 IMU 静止校准、超时失败及转角清零。
2. Brain 触屏菜单、中文按钮、翻页与 L1 检查点标记/续录流程。
3. SD 卡枚举、热插拔重试保存与旧版本文件隐藏。
4. 录制/回放中接入场控、切换禁用状态或拔掉 IMU/任一电机的即时停车行为。
5. IMU 跨过 `0/360`、多圈转向、检查点续录后的坐标连续性，以及不同负载下 PID 增益与纠偏方向标定。
