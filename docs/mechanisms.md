# 抬升与夹爪模块

## 定位

`lift` 和 `claw` 是当前机构的**最小可用开环控制模块**，按文档中“精简、能用、开环”的标准实现，不再使用GPT生成Slop，不按比赛级闭环机构框架扩展。

它们目前不接入 `main.cpp`，因此不会自动响应手柄；接入时由主循环或后续宏协议调用。

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
void set_power(int power);
void stop();
}
```

当前参数：

```text
端口：5（占位）
齿轮箱：blue（占位）
功率：-127..127
停止：brake
```

建议手动控制：

```text
R1 按住 -> 抬升
R2 按住 -> 下降
松开   -> stop
```

但这组按键**当前没有写进 main**。

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
端口：6（占位）
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

建议手动控制：

```text
R1 按住 -> open
R2 按住 -> close
松开   -> stop
```

同样，这组按键**当前没有写进 main**。

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