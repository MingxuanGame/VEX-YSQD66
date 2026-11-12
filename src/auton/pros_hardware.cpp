#include "auton/pros_hardware.hpp"
#include "chassis.h"

namespace auton {
namespace {
class ProsHardware final : public Hardware {
public:
    bool ready() const override;
    bool heading(double &angle_deg) const override;
    bool contact(bool &pressed) const override;
    bool drive_voltage(int left_mv, int right_mv) override;
    bool lift_voltage(int voltage_mv) override;
    bool action(Action action) override;
    void stop() override;
};

bool ProsHardware::ready() const {
    // TODO：确定 IMU、撞桩传感器、升降电机和夹爪/toggle 设备端口，检查绝对端口冲突。
    // TODO：初始化并限时校准 IMU，确认设备在线、升降上下限、机构方向后才返回 true。
    // 不因底盘能运行就声称整条自动路线硬件已就绪。
    return false;
}

bool ProsHardware::heading(double &angle_deg) const {
    (void)angle_deg;
    // TODO：读向右为正的 IMU 累积角度；校准中、未连接、非有限值均返回 false。
    return false;
}

bool ProsHardware::contact(bool &pressed) const {
    (void)pressed;
    // TODO：接入实际撞桩传感器并去抖；不能用超时或任意堵转假装撞桩成功。
    return false;
}

bool ProsHardware::drive_voltage(int left_mv, int right_mv) {
    return chassis::set_voltage(left_mv, right_mv);
}

bool ProsHardware::lift_voltage(int voltage_mv) {
    // TODO：接入升降电压驱动，正为抬升，负为下降；检查限位/故障/禁用，错误返回 false。
    // Override 2.1 <SG2>/<SG3>，第 30-31 页：限位必须覆盖机构中间姿态。
    return voltage_mv == 0;
}

bool ProsHardware::action(Action action) {
    (void)action;
    // TODO：Toggle（一次三色筒操作）、Open（松抓）、Close（抓 pin+cup）的实际机构动作。
    // 返回 true 只表示已启动；Runner 随后等待该步 action_ms，不能在这里阻塞。
    return false;
}

void ProsHardware::stop() {
    chassis::stop();
    // TODO：停止升降输出、取消机构运动目标；夹爪保持原状态，不在禁用时主动松物。
}
} // namespace

Hardware &pros_hardware() {
    static ProsHardware hardware;
    return hardware;
}
} // namespace auton
