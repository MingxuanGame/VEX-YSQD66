#include "io/input_reader.hpp"

namespace io {

void readInput(Controller &master, InputState &in, uint32_t now) {
    // 1. 首先检测物理连接状态
    in.isConnected = master.isConnected();

    // 2. 如果断联，强制执行安全保护（Fail-Safe）
    if (!in.isConnected) {
        in.leftY = 0;
        in.rightY = 0;
        in.leftX = 0;
        in.rightX = 0;

        in.up.update(false, now);
        in.down.update(false, now);
        in.left.update(false, now);
        in.right.update(false, now);
        in.a.update(false, now);
        in.b.update(false, now);
        in.x.update(false, now);
        in.y.update(false, now);
        in.l1.update(false, now);
        in.l2.update(false, now);
        in.r1.update(false, now);
        in.r2.update(false, now);
        return;
    }

    // 3. 正常数据读取（全部通过抽象接口，不依赖 PROS）
    in.leftY = master.analog(Analog::LeftY);
    in.rightY = master.analog(Analog::RightY);
    in.leftX = master.analog(Analog::LeftX);
    in.rightX = master.analog(Analog::RightX);

    in.up.update(master.digital(Digital::Up), now);
    in.down.update(master.digital(Digital::Down), now);
    in.left.update(master.digital(Digital::Left), now);
    in.right.update(master.digital(Digital::Right), now);

    in.a.update(master.digital(Digital::A), now);
    in.b.update(master.digital(Digital::B), now);
    in.x.update(master.digital(Digital::X), now);
    in.y.update(master.digital(Digital::Y), now);

    in.l1.update(master.digital(Digital::L1), now);
    in.l2.update(master.digital(Digital::L2), now);
    in.r1.update(master.digital(Digital::R1), now);
    in.r2.update(master.digital(Digital::R2), now);
}

} // namespace io
