// 在 Override Auto Studio 编辑自动路线，然后点击「导出 C++」。
// https://override-auto-studio.vercel.app/
// 用导出的完整 program.cpp 替换车辆工程中的 src/autonomous/program.cpp。
#include "autonomous/program.hpp"

#include "autonomous/route.hpp"

namespace auto_route {

void run_program() {
    // 空白路线模板在自动时段保持停车。
    stop();
}

} // namespace auto_route
