# Brain 底盘宏录制与文件协议 v3

`autonomous()` 当前留空，本模块在手动练习阶段提供 Brain 触屏宏录制、回放与检查点续录功能，使用 PROS V5 4.2.3 接口，无额外后台控制任务。

## 操作流程

程序启动、开始录放、进入暂停或动作结束时，统一停止底盘、保持刹车模式并清零编码器参考（不执行机械归零）。当前仅接入底盘组件；进入 Home 页面可正常驾驶，进入列表或结果页保持停车。

1. 将 FAT32 SD 卡插入 Brain 并开机，Home 页显示“新建宏录制”和“使用宏”。
2. 脱离场控并处于启用的手动模式时，点击“新建宏录制”：先停车并倒计时 **3 秒**（此时屏蔽手柄输入），随后每 **20ms** 采样并执行一帧，共 **725 帧（14.5 秒）**。
3. 录制过程中按下 **L1** 可在当前帧标记检查点（长按只记一次，松手后再按可继续标记）；末尾 **500ms** 自动停车并重置状态，随后保存至 `/usd/macro_NNNNNN.csv`（取现有最大编号加一，上限 `999999`，不覆盖旧文件）。
4. 点击“使用宏”浏览兼容的 v3 宏（每页 4 项），选中后先完整校验文件并进入检查点列表（包含末尾 `END 14.50s`）。
5. **将机器人放回原录制起点和朝向**，选择某检查点后先倒计时 **3 秒**，从第 0 帧回放至该检查点、停车 **3 秒**再交还手柄续录剩余帧，完成后另存新编号文件；选择 `END` 则只完整回放并停车，不产生新文件。
6. 录放期间点击 `STOP / CANCEL` 可立即取消并丢弃未完成的录制；若 SD 保存失败，完整录制仍留在内存中，插卡后可在结果页点击 `RETRY SD SAVE` 重试。

> 练习边界：Override 2.1 `<R9>`（英文第 61 页）与 `<GG12>`/`<GG13>`（第 46-47 页）要求服从场控并限制自动时段人为干预；接入场控、禁用或进入自动模式时，程序自动拒绝或中止宏录放。

## v3 文件格式

ASCII CSV（无 BOM，LF 或 CRLF 换行，末尾须有换行符，不含注释或空行）。头部必须精确为以下 6 行：

```csv
VEX_MACRO,3
period_ms,20
record_ms,14500
reset_ms,500
mapping,chassis_arcade_v1
t_ms,left_x,left_y,right_x,right_y,buttons,chassis_left_power,chassis_right_power,checkpoint
```

其后紧跟 **725 行**帧数据（`t_ms` 从 `0` 递增到 `14480`，步长 `20`），最后一行必须为 `END,725`。

| 字段 | 范围与说明 |
| --- | --- |
| `t_ms` | 毫秒，`0, 20, ..., 14480` |
| `left_x,left_y,right_x,right_y` | 原始摇杆整数 `[-127, 127]`（仅供追溯，回放不重算） |
| `buttons` | 12 位按键掩码 `0..4095`（Bit 0–11 依次为 Up/Down/Left/Right/A/B/X/Y/L1/L2/R1/R2） |
| `chassis_left_power,chassis_right_power` | PROS `move()` 功率 `[-127, 127]`，回放直接下发 |
| `checkpoint` | `0` 或 `1`，标记该帧结束处是否作为可续录检查点 |

说明：PROS 4.2.3 下对不存在文件调用 `fopen(..., "rb")` 会返回 `errno=23`（`ENFILE`），因此真机保存时通过 `pros::usd::list_files()` 目录枚举判重并按 FAT32 忽略大小写比较。

## 底盘映射与离线微调

手动与录制沿用 Arcade 混控方向，经 `chassis::set_power()` 统一做右侧 `+10` 功率偏置补偿与等幅限幅：

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

离线微调应直接修改 `chassis_left_power` 和 `chassis_right_power` 两列（第 6、7 列），并保持精确 725 帧与连续 `t_ms`：

```python
import csv
from pathlib import Path

path = Path("macro_000001.csv")
rows = list(csv.reader(path.read_text(encoding="ascii").splitlines()))
assert rows[0] == ["VEX_MACRO", "3"] and len(rows) == 6 + 725 + 1
rows[6 + 100][6:8] = ["80", "60"]  # 修改第 100 帧 (t_ms=2000) 的左右功率
with path.open("w", encoding="ascii", newline="") as f:
    csv.writer(f, lineterminator="\n").writerows(rows)
```

## 模块职责与待扩展项

- `include/macro/engine.hpp`、`src/macro/engine.cpp`：抽象接口 `macrorec::Hardware` 与非阻塞状态机 `macrorec::Engine`
- `include/macro/storage.hpp`、`src/macro/storage.cpp`：v3 CSV 读写、编号提取与写后回读校验
- `include/macro/screen.hpp`、`src/macro/screen.cpp`、`src/macro/labels.inc`：`pros::screen` 触屏菜单与内嵌中文字形
- `include/macro/session.hpp`、`src/macro/session.cpp`：`Components` 硬件适配器、场控检查、SD 读写与主循环调度
- **机构扩展 TODO**：接入抬升/夹爪/推板时，先标定端口与限位，再扩展 `Command`、`map_input()`、`Components` 并升级协议版本。

## 实机待验证项

1. 四轮实际转向、右侧 `+10` 补偿直行效果及编码器清零。
2. Brain 触屏菜单、中文按钮、翻页与 L1 检查点标记/续录流程。
3. SD 卡枚举、热插拔重试保存与旧版本文件隐藏。
4. 录制/回放中接入场控或切换禁用状态的即时停车行为。
