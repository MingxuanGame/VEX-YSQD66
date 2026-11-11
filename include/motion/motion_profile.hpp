#pragma once
#include "vxmath/speed_profile.hpp"

/**
 * @file motion_profile.hpp
 * @brief 运动规划层入口。复用 vxmath::TrapezoidProfile，避免重复实现。
 */

namespace motion {

using TrapezoidProfile = vxmath::TrapezoidProfile;

} // namespace motion
