#include "vxmath/units.hpp"
#include "vxmath/math_utils.hpp"

namespace vxmath {

double ticksToInches(double ticks) {
    return ticks * cfg::kInchPerTick;
}

double inchesToTicks(double inches) {
    return inches / cfg::kInchPerTick;
}

double rpmToInchesPerSec(double rpm) {
    return rpm * cfg::kWheelCircumferenceInch / (60.0 * cfg::kDriveGearRatio);
}

double inchesPerSecToRpm(double ips) {
    return ips * 60.0 * cfg::kDriveGearRatio / cfg::kWheelCircumferenceInch;
}

int clampServoAngle(int angle) {
    return clampVal(angle, cfg::kServoMinAngle, cfg::kServoMaxAngle);
}

int clampMotorOutput(int out) {
    return clampVal(out, cfg::kMotorReverseMaxOutput, cfg::kMotorMaxOutput);
}

double clampMotorOutput(double out) {
    return clampVal(out, static_cast<double>(cfg::kMotorReverseMaxOutput),
                    static_cast<double>(cfg::kMotorMaxOutput));
}

int percentToMotor(double percent) {
    return clampMotorOutput(static_cast<int>(percent * cfg::kMotorMaxOutput / cfg::kPercentBase));
}

double motorToPercent(int out) {
    return static_cast<double>(clampMotorOutput(out)) * cfg::kPercentBase / cfg::kMotorMaxOutput;
}

} // namespace vxmath
