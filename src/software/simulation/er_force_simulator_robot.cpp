#include "software/simulation/er_force_simulator_robot.h"

#include "proto/message_translation/ssl_simulation_robot_control.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "software/logger/logger.h"

ErForceSimulatorRobot::ErForceSimulatorRobot(const RobotStateWithId& robot_state_with_id,
                                             RobotConstants_t robot_constants,
                                             WheelConstants_t wheel_constants)
    : dribbler_ball_contact(false),
      id(robot_state_with_id.id),
      robot_state(robot_state_with_id.robot_state),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants)
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

std::unique_ptr<SSLSimulationProto::RobotCommand> ErForceSimulatorRobot::getRobotCommand()
{
    auto move_command = createRobotMoveCommand(
        *direct_control, robot_constants.front_wheel_angle_deg,
        robot_constants.back_wheel_angle_deg, wheel_constants.wheel_radius_meters);

    // If there are robots on the field that don't have a primitive, the wheel control
    // will be empty. In this case we return a blank RobotMoveCommand so that robots don't
    // move.
    if (direct_control->wheel_control_case() ==
        TbotsProto::DirectControlPrimitive::WHEEL_CONTROL_NOT_SET)
    {
        move_command = std::make_unique<SSLSimulationProto::RobotMoveCommand>();
    }

    switch (direct_control->chick_command_case())
    {
        case TbotsProto::DirectControlPrimitive::kKickSpeedMPerS:
        {
            kick(direct_control->kick_speed_m_per_s());
            break;
        }
        case TbotsProto::DirectControlPrimitive::kChipDistanceMeters:
        {
            chip(direct_control->chip_distance_meters());
            break;
        }
        case TbotsProto::DirectControlPrimitive::kAutokickSpeedMPerS:
        {
            kick(direct_control->autokick_speed_m_per_s());
            break;
        }
        case TbotsProto::DirectControlPrimitive::kAutochipDistanceMeters:
        {
            chip(direct_control->autochip_distance_meters());
            break;
        }
        case TbotsProto::DirectControlPrimitive::CHICK_COMMAND_NOT_SET:
        {
            // Command not set: lets not do anything here
            // chip(0.0f);
            // kick(0.0f);
        }
    }

    return createRobotCommand(id, std::move(move_command), kick_speed, kick_angle,
                              direct_control->dribbler_speed_rpm());
}

std::optional<TbotsProto::RobotStatus> ErForceSimulatorRobot::getRobotStatus() const
{
    auto robot_status = std::make_optional<TbotsProto::RobotStatus>();
    robot_status->set_robot_id(id);

    auto break_beam_status = TbotsProto::BreakBeamStatus();
    break_beam_status.set_ball_in_beam(dribbler_ball_contact);
    *(robot_status->mutable_break_beam_status()) = break_beam_status;

    return robot_status;
}

void ErForceSimulatorRobot::setRobotFeedback(
    const SSLSimulationProto::RobotFeedback& robot_feedback)
{
    dribbler_ball_contact = robot_feedback.dribbler_ball_contact();
}

void ErForceSimulatorRobot::reset()
{
    kick_speed = std::nullopt;
    kick_angle = std::nullopt;
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

void ErForceSimulatorRobot::startNewPrimitive(const TbotsProto::Primitive& primitive)
{
    primitive_executor.startPrimitive(robot_constants, primitive);
}

void ErForceSimulatorRobot::runCurrentPrimitive()
{
    direct_control = primitive_executor.stepPrimitive(robot_state);
}
