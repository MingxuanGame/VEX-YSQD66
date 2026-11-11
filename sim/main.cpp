#ifndef ENABLE_SIMULATOR
#define ENABLE_SIMULATOR
#endif

#include "collision/collision_detector.hpp"
#include "compat/pros_compat.hpp"
#include "compat/sim_robot.hpp"
#include "sim/physics/chassis_sim.hpp"
#include <iostream>

// Virtual hardware used by the simulation demo.
pros::Motor left_front(1, pros::E_MOTOR_GEARSET_18);
pros::Motor left_back(2, pros::E_MOTOR_GEARSET_18);
pros::Motor right_front(3, pros::E_MOTOR_GEARSET_18, true);
pros::Motor right_back(4, pros::E_MOTOR_GEARSET_18, true);
pros::Imu imu(5);

sim_physics::ChassisSim chassis(&left_front, &left_back, &right_front, &right_back, &imu);

void physics_tick(double dt_sec) {
    chassis.step(dt_sec);
}

int main() {
    sim::init_clock();
    sim::register_physics_tick(physics_tick);
    imu.setNoiseLevel(0.1);

    sim::SimChassis sim_chassis(&left_front, &left_back, &right_front, &right_back);
    sim::SimImu sim_imu(&imu);
    collision::CollisionDetector detector(sim_chassis, sim_imu);

    std::cout << "=== Simulation start ===" << std::endl;

    // Full-throttle straight drive.
    left_front.move(127);
    left_back.move(127);
    right_front.move(127);
    right_back.move(127);
    pros::delay(1500);

    std::cout << "Forward 1.5s -> speed: " << chassis.getChassisVelocityMps() << " m/s"
              << std::endl;
    std::cout << "Forward 1.5s -> current: " << left_front.get_current_draw() << " mA" << std::endl;

    // Wall collision: the chassis should stall and current should spike.
    std::cout << "\n[Event] Wall collision" << std::endl;
    chassis.setCollisionObject(sim_physics::CollisionObject::Wall);
    pros::delay(200);

    std::cout << "At wall -> speed: " << chassis.getChassisVelocityMps() << " m/s" << std::endl;
    std::cout << "At wall -> current: " << left_front.get_current_draw() << " mA" << std::endl;

    if (detector.isTriggered()) {
        collision::CollisionFeatures features;
        const collision::CollisionType type = detector.detect(&features);
        std::cout << "\n[Collision] type=" << static_cast<int>(type)
                  << " avg_current=" << features.avg_current << " max_gyro=" << features.max_gyro
                  << std::endl;
    } else {
        std::cout << "\n[Collision] not triggered" << std::endl;
    }

    // Turn in place after leaving the wall.
    std::cout << "\n[Event] Leave wall, turn in place" << std::endl;
    chassis.setCollisionObject(sim_physics::CollisionObject::None);
    left_front.move(100);
    left_back.move(100);
    right_front.move(-100);
    right_back.move(-100);
    pros::delay(500);

    std::cout << "IMU rotation: " << imu.get_rotation() << " deg" << std::endl;
    std::cout << "Pose: x=" << chassis.getPoseXMeters() << " y=" << chassis.getPoseYMeters()
              << " heading=" << chassis.getPoseHeadingDeg() << " deg" << std::endl;
    std::cout << "=== Simulation end ===" << std::endl;
    return 0;
}
