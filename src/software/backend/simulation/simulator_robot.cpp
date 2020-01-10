#include "software/backend/simulation/simulator_robot.h"

#include "app/world/chicker.h"
#include "app/world/dribbler.h"
#include "app/world/wheel.h"

std::optional<unsigned int> SimulatorRobot::robot_id = std::nullopt;
std::vector<std::weak_ptr<PhysicsRobot>> SimulatorRobot::physics_robots = {};

void SimulatorRobot::setRobotId(unsigned int id)
{
    // TODO: you are here. Need to do final test of this class and fix ball
    // trying to find dulpicate static members?
    robot_id = std::make_optional<unsigned int>(id);
}

void SimulatorRobot::setPhysicsRobots(const std::vector<std::weak_ptr<PhysicsRobot>>& robots) {
    physics_robots = robots;
}

FirmwareRobot_t* SimulatorRobot::createFirmwareRobot()
{
    Chicker_t* chicker = app_chicker_create(
        &(SimulatorRobot::kick), &(SimulatorRobot::chip),
        &(SimulatorRobot::enableAutokick), &(SimulatorRobot::enableAutochip),
        &(SimulatorRobot::disableAutokick), &(SimulatorRobot::disableAutochip));

    Dribbler_t* dribbler =
        app_dribbler_create(&(SimulatorRobot::setDribblerSpeed),
                            &(SimulatorRobot::getDribblerTemperatureDegC));

    Wheel_t* front_left_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceFrontLeft));
    Wheel_t* front_right_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceFrontRight));
    Wheel_t* back_left_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceBackLeft));
    Wheel_t* back_right_wheel =
        app_wheel_create(&(SimulatorRobot::applyWheelForceBackRight));

    FirmwareRobot_t* firmware_robot =
        app_firmware_robot_create(chicker, dribbler, &(SimulatorRobot::getPositionX),
                                  &(SimulatorRobot::getPositionY), front_right_wheel,
                                  front_left_wheel, back_right_wheel, back_left_wheel);

    return firmware_robot;
}

float SimulatorRobot::getPositionX() {
    // Temporary implementation for testing
    auto robot = getCurrentPhysicsRobot();
    if(auto robot_lock = robot.lock()) {
        return static_cast<float>(robot_lock->getRobotId());
    }
    return 0.0;
}

float SimulatorRobot::getPositionY() {}

void SimulatorRobot::kick(float speed_m_per_s) {}

void SimulatorRobot::chip(float distance_m) {}

void SimulatorRobot::enableAutokick(float speed_m_per_s) {}

void SimulatorRobot::enableAutochip(float distance_m) {}

void SimulatorRobot::disableAutokick() {}

void SimulatorRobot::disableAutochip() {}

void SimulatorRobot::setDribblerSpeed(uint32_t rpm) {}

unsigned int SimulatorRobot::getDribblerTemperatureDegC()
{
    // Return a somewhat arbitrary "room temperature" temperature.
    // This is an ideal simulation so the dribbler will not overheat
    return 25;
}

void SimulatorRobot::applyWheelForceFrontLeft(float force_in_newtons) {}

void SimulatorRobot::applyWheelForceBackLeft(float force_in_newtons) {}

void SimulatorRobot::applyWheelForceBackRight(float force_in_newtons) {}

void SimulatorRobot::applyWheelForceFrontRight(float force_in_newtons) {}

std::weak_ptr<PhysicsRobot> SimulatorRobot::getCurrentPhysicsRobot() {
    if(!robot_id.has_value()) {
        throw std::invalid_argument("Robot ID has no value");
    }

    unsigned int robot_id_value = *robot_id;
    auto robot_id_comparator = [robot_id_value](std::weak_ptr<PhysicsRobot> robot_ptr) {
        if(auto robot_ptr_lock = robot_ptr.lock()) {
            return robot_ptr_lock->getRobotId() == robot_id_value;
        }
        return false;
    };
    auto physics_robot_iter = std::find_if(physics_robots.begin(), physics_robots.end(), robot_id_comparator);
    if(physics_robot_iter == physics_robots.end()) {
        throw std::invalid_argument("Robot ID not found in list of Physics Robots");
    }

    return *physics_robot_iter;
}
