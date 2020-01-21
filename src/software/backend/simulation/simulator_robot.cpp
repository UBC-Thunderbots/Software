#include "software/backend/simulation/simulator_robot.h"

extern "C"
{
#include "app/world/chicker.h"
#include "app/world/dribbler.h"
#include "app/world/wheel.h"
}

std::optional<unsigned int> SimulatorRobotSingleton::robot_id = std::nullopt;
std::vector<std::weak_ptr<PhysicsRobot>> SimulatorRobotSingleton::physics_robots = {};

void SimulatorRobotSingleton::setRobotId(unsigned int id)
{
    robot_id = std::make_optional<unsigned int>(id);
}

void SimulatorRobotSingleton::setPhysicsRobots(
    const std::vector<std::weak_ptr<PhysicsRobot>>& robots)
{
    physics_robots = robots;
}

std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>
SimulatorRobotSingleton::createFirmwareRobot()
{
    // TODO: Make sure all objects de-allocated properly
    // See issue https://github.com/UBC-Thunderbots/Software/issues/1128
    Chicker_t* chicker = app_chicker_create(&(SimulatorRobotSingleton::kick),
                                            &(SimulatorRobotSingleton::chip),
                                            &(SimulatorRobotSingleton::enableAutokick),
                                            &(SimulatorRobotSingleton::enableAutochip),
                                            &(SimulatorRobotSingleton::disableAutokick),
                                            &(SimulatorRobotSingleton::disableAutochip));

    Dribbler_t* dribbler =
        app_dribbler_create(&(SimulatorRobotSingleton::setDribblerSpeed),
                            &(SimulatorRobotSingleton::getDribblerTemperatureDegC));

    Wheel_t* front_left_wheel =
        app_wheel_create(&(SimulatorRobotSingleton::applyWheelForceFrontLeft));
    Wheel_t* front_right_wheel =
        app_wheel_create(&(SimulatorRobotSingleton::applyWheelForceFrontRight));
    Wheel_t* back_left_wheel =
        app_wheel_create(&(SimulatorRobotSingleton::applyWheelForceBackLeft));
    Wheel_t* back_right_wheel =
        app_wheel_create(&(SimulatorRobotSingleton::applyWheelForceBackRight));

    FirmwareRobot_t* firmware_robot = app_firmware_robot_create(
        chicker, dribbler, &(SimulatorRobotSingleton::getPositionX),
        &(SimulatorRobotSingleton::getPositionY), front_right_wheel, front_left_wheel,
        back_right_wheel, back_left_wheel);

    return std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter>(firmware_robot,
                                                                  FirmwareRobotDeleter());
}

float SimulatorRobotSingleton::getPositionX()
{
    // Temporary implementation for testing
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        return static_cast<float>(robot->getRobotId());
    }
    return 0.0;
}

float SimulatorRobotSingleton::getPositionY()
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::kick(float speed_m_per_s)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::chip(float distance_m)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::enableAutokick(float speed_m_per_s)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::enableAutochip(float distance_m)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::disableAutokick()
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::disableAutochip()
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::setDribblerSpeed(uint32_t rpm)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

unsigned int SimulatorRobotSingleton::getDribblerTemperatureDegC()
{
    // Return a somewhat arbitrary "room temperature" temperature.
    // This is an ideal simulation so the dribbler will not overheat
    return 25;
}

void SimulatorRobotSingleton::applyWheelForceFrontLeft(float force_in_newtons)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::applyWheelForceBackLeft(float force_in_newtons)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::applyWheelForceBackRight(float force_in_newtons)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

void SimulatorRobotSingleton::applyWheelForceFrontRight(float force_in_newtons)
{
    if (auto robot = getCurrentPhysicsRobot().lock())
    {
        // TODO: Implement me
    }
}

std::weak_ptr<PhysicsRobot> SimulatorRobotSingleton::getCurrentPhysicsRobot()
{
    if (!robot_id.has_value())
    {
        return std::weak_ptr<PhysicsRobot>();
    }

    unsigned int robot_id_value = *robot_id;
    auto robot_id_comparator = [robot_id_value](std::weak_ptr<PhysicsRobot> robot_ptr) {
        if (auto robot_ptr_lock = robot_ptr.lock())
        {
            return robot_ptr_lock->getRobotId() == robot_id_value;
        }
        return false;
    };
    auto physics_robot_iter =
        std::find_if(physics_robots.begin(), physics_robots.end(), robot_id_comparator);
    if (physics_robot_iter == physics_robots.end())
    {
        return std::weak_ptr<PhysicsRobot>();
    }

    return *physics_robot_iter;
}
