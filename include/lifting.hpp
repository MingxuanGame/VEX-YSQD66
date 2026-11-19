#pragma once

#include <cstdint>

namespace lifting {

// 设置双电机保持制动模式。
void initialize();

// 启用手动时下发功率 [-127, 127]；零功率刹车。
int set_power(int power);

// 启用自动时按功率运行指定毫秒数，结束或退出模式时刹车。
bool move_for(int power, std::uint32_t duration_ms);

// 刹车并保持当前位置。
void stop();

} // namespace lifting
