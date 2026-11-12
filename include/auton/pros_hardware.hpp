#pragma once

#include "auton/kit.hpp"

namespace auton {
// 返回真机唯一适配器；非阻塞。未完成传感器、机构和限位 TODO 前 ready() 返回 false。
Hardware &pros_hardware();
} // namespace auton
