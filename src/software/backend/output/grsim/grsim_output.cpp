#include "software/backend/output/grsim/grsim_output.h"

#include <chrono>
#include <optional>

#include "shared/constants.h"
#include "software/backend/output/grsim/command_primitive_visitor/grsim_command_primitive_visitor.h"
#include "software/backend/output/grsim/command_primitive_visitor/motion_controller.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/primitive/primitive.h"
#include "software/proto/grSim_Commands.pb.h"
#include "software/proto/grSim_Replacement.pb.h"
#include "software/util/variant_visitor/variant_visitor.h"
#include "software/world/team.h"


using namespace boost::asio;

GrSimOutput::GrSimOutput(std::string network_address, unsigned short port,
                         std::shared_ptr<const RefboxConfig> config)
    : network_address(network_address), port(port), config(config), socket(io_service)
{
    socket.open(ip::udp::v4());
    remote_endpoint =
        ip::udp::endpoint(ip::address::from_string(network_address), this->port);
}

GrSimOutput::~GrSimOutput()
{
    socket.close();
}

void GrSimOutput::sendPrimitives(
    const std::vector<std::unique_ptr<Primitive>>& primitives, const Team& friendly_team,
    const Ball& ball)
{
    // TODO: Can't replace this timestamp as part of issue #228 because the Timestamp
    // class doesn't support absolute "wall time". This function will need to be
    // changed to make use of the timestamps stored with the robots
    // https://github.com/UBC-Thunderbots/Software/issues/279
    //
    // initial timestamp for bang-bang set as current time
    static auto bangbang_timestamp = std::chrono::steady_clock::now();

    std::chrono::duration<double> delta_time =
        std::chrono::steady_clock::now() - bangbang_timestamp;

    MotionController motionController(ROBOT_MAX_SPEED_METERS_PER_SECOND,
                                      ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND,
                                      ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                                      ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED);

    for (auto& prim : primitives)
    {
        if (friendly_team.getRobotById(prim->getRobotId()))
        {
            Robot robot = *friendly_team.getRobotById(prim->getRobotId());

            GrsimCommandPrimitiveVisitor grsim_command_primitive_visitor =
                GrsimCommandPrimitiveVisitor(robot, ball);
            prim->accept(grsim_command_primitive_visitor);

            std::variant<MotionController::PositionCommand,
                         MotionController::VelocityCommand>
                motion_controller_command =
                    grsim_command_primitive_visitor.getMotionControllerCommand();

            MotionController::Velocity robot_velocities =
                motionController.bangBangVelocityController(robot, delta_time.count(),
                                                            motion_controller_command);

            double kick_speed_meters_per_second;
            bool chip_instead_of_kick;
            bool dribbler_on;

            std::visit(
                overload{[&kick_speed_meters_per_second, &chip_instead_of_kick,
                          &dribbler_on](MotionController::PositionCommand& command) {
                             kick_speed_meters_per_second =
                                 command.kick_speed_meters_per_second;
                             chip_instead_of_kick = command.chip_instead_of_kick;
                             dribbler_on          = command.dribbler_on;
                         },

                         [&kick_speed_meters_per_second, &chip_instead_of_kick,
                          &dribbler_on](MotionController::VelocityCommand& command) {
                             kick_speed_meters_per_second =
                                 command.kick_speed_meters_per_second;
                             chip_instead_of_kick = command.chip_instead_of_kick;
                             dribbler_on          = command.dribbler_on;
                         }},
                motion_controller_command);

            // send the velocity data via grsim_packet
            grSim_Packet grsim_packet = createGrSimPacketWithRobotVelocity(
                prim->getRobotId(), config->FriendlyColorYellow()->value(),
                robot_velocities.linear_velocity, robot_velocities.angular_velocity,
                kick_speed_meters_per_second, chip_instead_of_kick, dribbler_on);

            sendGrSimPacket(grsim_packet);
        }
    }

    // timestamp of when the motion controller was last run (to be used for calculating
    // delta_time in the future)
    bangbang_timestamp = std::chrono::steady_clock::now();
}

grSim_Packet GrSimOutput::createGrSimPacketWithRobotVelocity(
    unsigned int robot_id, bool is_yellow, Vector robot_velocity,
    AngularVelocity angular_velocity, double kick_speed_meters_per_second, bool chip,
    bool dribbler_on) const
{
    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(is_yellow);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* robot_command = packet.mutable_commands()->add_robot_commands();

    robot_command->set_id(robot_id);

    // We set a robot velocity, not individual wheel velocities
    robot_command->set_wheelsspeed(false);

    // veltangent moves the robot forward and backward
    robot_command->set_veltangent(static_cast<float>(robot_velocity.x()));
    // velnormal strafes the robots left and right
    robot_command->set_velnormal(static_cast<float>(robot_velocity.y()));
    robot_command->set_velangular(static_cast<float>(angular_velocity.toRadians()));

    robot_command->set_kickspeedx(static_cast<float>(kick_speed_meters_per_second));
    // The vertical component of kicks (used to create chips) are applied separately. We
    // use the same value as the kick speed to get a chip angle of roughly 45 degrees
    robot_command->set_kickspeedz(
        static_cast<float>(chip ? kick_speed_meters_per_second : 0.0));
    robot_command->set_spinner(dribbler_on);

    return packet;
}

void GrSimOutput::sendGrSimPacket(const grSim_Packet& packet)
{
    boost::system::error_code err;
    socket.send_to(
        buffer(packet.SerializeAsString(), static_cast<size_t>(packet.ByteSize())),
        remote_endpoint, 0, err);
}
