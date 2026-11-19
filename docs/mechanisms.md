# 抬升与夹爪模块

## 定位

`lift` 和 `claw` 是当前机构的**最小可用开环控制模块**，按文档中“精简、能用、开环”的标准实现，不再使用GPT生成Slop，不按比赛级闭环机构框架扩展。

它们的手动接入代码已写入 `macrorec` 会话，但当前用 `#if 0` 注释关闭，暂不响应手柄；宏录制/回放仍只记录底盘。

---

## 精简标准

这两个模块明确保持以下边界：

```text
单电机
直接 move() 功率输出
无编码器位置闭环
无限位开关
无电流/温度/堵转保护
无额外状态机
无动态内存分配
停止使用 brake
```

也就是说：

```text
给功率就动
松开/stop 就刹车
到机械限位不会自动停
```

---

## 文件

```text
include/lift.hpp
src/lift.cpp

include/claw.hpp
src/claw.cpp
```

---

## 抬升

接口：

```cpp
namespace lift {
void initialize();
void toggle();
void update(bool pressed);
void set_power(int power);
void stop();
}
```

行为：

```text
第一次 toggle -> 上升
第二次 toggle -> 下降
第三次 toggle -> 上升
stop          -> 刹车并回到空闲
update(pressed) 只在按下的上升沿触发 toggle
```

当前参数：

```text
端口：7（占位）
齿轮箱：blue（占位）
上升功率：+80
下降功率：-80
停止：brake
```

> IMU 固定使用端口 5，抬升不得再占用端口 5。当前 7 只是占位，接线确认后必须修改。

手动接入代码：

```text
占位按键：R1（bit 10）-> toggle
主循环调用 update(controller.digital(R1))
```

代码当前位于 `src/macro/session.cpp` 的 `#if 0` 块内，确认端口和按键后再启用。  
宏运行期间机构保持停止，不记录、不回放。

---

## 夹爪

接口：

```cpp
namespace claw {
void initialize();
void open();
void close();
void set_power(int power);
void stop();
}
```

当前参数：

```text
端口：8（占位）
齿轮箱：blue（占位）
张开：+80
闭合：-80
停止：brake
```

结构假设：

```text
两个蓝色齿轮啮合
一个电机驱动
另一个齿轮随动
中间连杆/丝杆完成开合
无位置传感器
```

手动接入代码：

```text
占位按键：L2（bit 9）-> open
占位按键：R2（bit 11）-> close
松开 -> stop
```

代码当前位于 `src/macro/session.cpp` 的 `#if 0` 块内，确认端口和按键后再启用。  
宏运行期间机构保持停止，不记录、不回放。

---

## 开环限制

```text
不知道机构实际位置
不知道是否已经到限位
夹住物体后可能持续堵转
换电池/负载后位置会漂移
宏回放不包含 lift/claw 动作
```

因此当前版本适合：

```text
结构调试
手动开合测试
确认方向和端口
```

不适合作为比赛级自动路线里的精确机构控制，但够用。