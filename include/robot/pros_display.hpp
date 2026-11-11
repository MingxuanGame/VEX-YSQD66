#pragma once

namespace robot {

class Display;

/**
 * @brief 真机屏幕适配器，返回 pros::lcd 的 robot::Display 包装。
 */
Display &prosDisplay();

} // namespace robot
