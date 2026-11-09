#include "chassis.h"
#include "main.h"

/**
 * Left joystick Y controls forward/backward motion.
 * Right joystick X controls turning.
 */
namespace chassis {
void move() {
  static pros::Controller master(pros::E_CONTROLLER_MASTER);
  static pros::MotorGroup left_mg({3, 4}, pros::MotorGears::blue);
  static pros::MotorGroup right_mg({1, 2}, pros::MotorGears::blue);

  int dir = master.get_analog(ANALOG_LEFT_Y);
  int turn = master.get_analog(ANALOG_RIGHT_X);

  left_mg.move(dir + turn);
  right_mg.move(dir - turn);
}
} // namespace chassis