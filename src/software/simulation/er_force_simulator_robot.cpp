#include "software/simulation/er_force_simulator_robot.h"

#include "shared/constants.h"
#include "software/proto/message_translation/ssl_simulation_robot_control.h"

ErForceSimulatorRobot::ErForceSimulatorRobot(const RobotStateWithId& robot_state_with_id)
    : dribbler_ball_contact(true),
      id(robot_state_with_id.id),
      robot_state(robot_state_with_id.robot_state)
{
}

unsigned int ErForceSimulatorRobot::getRobotId()
{
    return id;
}

void ErForceSimulatorRobot::setRobotState(const RobotState& robot_state)
{
    this->robot_state = robot_state;
}

std::unique_ptr<sslsim::RobotCommand> ErForceSimulatorRobot::getRobotCommand()
{
    auto move_command =
        createRobotMoveCommand(wheel_speed_front_right, wheel_speed_front_left,
                               wheel_speed_back_left, wheel_speed_back_right);

    if (dribbler_ball_contact)
    {
        if (isAutochipEnabled())
        {
            chip(autochip_distance_m.value());
        }
        else if (isAutokickEnabled())
        {
            kick(autokick_speed_m_per_s.value());
        }
    }

    return createRobotCommand(id, std::move(move_command), kick_speed, kick_angle,
                              dribbler_speed);
}

void ErForceSimulatorRobot::setRobotFeedback(
    const SSLSimulationProto::RobotFeedback& robot_feedback)
{
    // Keep the robot dribbler contact always true
}

void ErForceSimulatorRobot::reset()
{
    dribbler_speed = std::nullopt;
    // TODO: investigate if these values need to be
    // explicitly reset to 0 after each kick/chip
    kick_speed = std::nullopt;
    kick_angle = std::nullopt;
}

float ErForceSimulatorRobot::getPositionX()
{
    return static_cast<float>(robot_state.position().x());
}

float ErForceSimulatorRobot::getPositionY()
{
    return static_cast<float>(robot_state.position().y());
}

float ErForceSimulatorRobot::getOrientation()
{
    return static_cast<float>(robot_state.orientation().toRadians());
}

float ErForceSimulatorRobot::getVelocityX()
{
    return static_cast<float>(robot_state.velocity().x());
}

float ErForceSimulatorRobot::getVelocityY()
{
    return static_cast<float>(robot_state.velocity().y());
}

float ErForceSimulatorRobot::getVelocityAngular()
{
    return static_cast<float>(robot_state.angularVelocity().toRadians());
}

float ErForceSimulatorRobot::getBatteryVoltage()
{
    return ROBOT_MAX_BATTERY_VOLTAGE;
}

void ErForceSimulatorRobot::kick(float speed_m_per_s)
{
    kick_speed = speed_m_per_s;
    kick_angle =
        std::nullopt;  // The field kick_angle in RobotCommand has a default value of 0
}

void ErForceSimulatorRobot::chip(float distance_m)
{
    Angle chip_angle = Angle::fromDegrees(ROBOT_CHIP_ANGLE_DEGREES);
    // Use the formula for the Range of a parabolic projectile
    // Rearrange to solve for the initial velocity.
    // https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
    float range = distance_m;
    float numerator =
        range * static_cast<float>(ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED);
    float denominator = static_cast<float>(2.0f * (chip_angle * 2.0f).sin());
    float chip_speed  = static_cast<float>(std::sqrt(numerator / denominator));

    kick_speed = chip_speed;
    kick_angle = chip_angle.toDegrees();
}

void ErForceSimulatorRobot::enableAutokick(float speed_m_per_s)
{
    autokick_speed_m_per_s = speed_m_per_s;
    disableAutochip();
}

void ErForceSimulatorRobot::enableAutochip(float distance_m)
{
    autochip_distance_m = distance_m;
    disableAutokick();
}

void ErForceSimulatorRobot::disableAutokick()
{
    autokick_speed_m_per_s = std::nullopt;
}

void ErForceSimulatorRobot::disableAutochip()
{
    autochip_distance_m = std::nullopt;
}

bool ErForceSimulatorRobot::isAutokickEnabled()
{
    return autokick_speed_m_per_s.has_value();
}

bool ErForceSimulatorRobot::isAutochipEnabled()
{
    return autochip_distance_m.has_value();
}

void ErForceSimulatorRobot::setDribblerSpeed(uint32_t rpm)
{
    dribbler_speed = rpm;
}

void ErForceSimulatorRobot::dribblerCoast() {}

unsigned int ErForceSimulatorRobot::getDribblerTemperatureDegC()
{
    // Return a nominal value
    return 25;
}

void ErForceSimulatorRobot::setTargetRPMFrontLeft(float rpm)
{
    wheel_speed_front_left = rpm;
}

void ErForceSimulatorRobot::setTargetRPMBackLeft(float rpm)
{
    wheel_speed_back_left = rpm;
}

void ErForceSimulatorRobot::setTargetRPMBackRight(float rpm)
{
    wheel_speed_back_right = rpm;
}

void ErForceSimulatorRobot::setTargetRPMFrontRight(float rpm)
{
    wheel_speed_front_right = rpm;
}

float ErForceSimulatorRobot::getMotorSpeedFrontLeft()
{
    return getMotorSpeeds()[0];
}

float ErForceSimulatorRobot::getMotorSpeedBackLeft()
{
    return getMotorSpeeds()[1];
}

float ErForceSimulatorRobot::getMotorSpeedBackRight()
{
    return getMotorSpeeds()[2];
}

float ErForceSimulatorRobot::getMotorSpeedFrontRight()
{
    return getMotorSpeeds()[3];
}

void ErForceSimulatorRobot::coastMotorBackLeft() {}

void ErForceSimulatorRobot::coastMotorBackRight() {}

void ErForceSimulatorRobot::coastMotorFrontLeft() {}

void ErForceSimulatorRobot::coastMotorFrontRight() {}

void ErForceSimulatorRobot::brakeMotorBackLeft()
{
    wheel_speed_back_left = 0;
}

void ErForceSimulatorRobot::brakeMotorBackRight()
{
    wheel_speed_back_right = 0;
}

void ErForceSimulatorRobot::brakeMotorFrontLeft()
{
    wheel_speed_front_left = 0;
}

void ErForceSimulatorRobot::brakeMotorFrontRight()
{
    wheel_speed_front_right = 0;
}

void ErForceSimulatorRobot::startNewPrimitive(
    std::shared_ptr<FirmwareWorld_t> firmware_world,
    const TbotsProto_Primitive& primitive_msg)
{
    app_primitive_manager_startNewPrimitive(primitive_manager.get(), firmware_world.get(),
                                            primitive_msg);
}

void ErForceSimulatorRobot::runCurrentPrimitive(
    std::shared_ptr<FirmwareWorld_t> firmware_world)
{
    app_primitive_manager_runCurrentPrimitive(primitive_manager.get(),
                                              firmware_world.get());
}

std::array<float, 4> ErForceSimulatorRobot::getMotorSpeeds() const
{
    auto local_velocity = robot_state.velocity().rotate(-robot_state.orientation());
    float robot_local_speed[3]{
        static_cast<float>(local_velocity.x()), static_cast<float>(local_velocity.y()),
        static_cast<float>(robot_state.angularVelocity().toRadians() * ROBOT_RADIUS)};
    float wheel_speeds[4]{0.0, 0.0, 0.0, 0.0};
    speed3_to_speed4(robot_local_speed, wheel_speeds);
    std::array<float, 4> motor_speeds = {0.0, 0.0, 0.0, 0.0};
    for (unsigned int i = 0; i < 4; i++)
    {
        // Convert to m/s to RPM then divide by gear ratio to get motor speed
        motor_speeds[i] =
            (wheel_speeds[i] * 60.0f / (2.0f * (float)M_PI * WHEEL_RADIUS)) / GEAR_RATIO;
    }
    return motor_speeds;
}
