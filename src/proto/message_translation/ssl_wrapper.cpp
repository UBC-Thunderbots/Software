#include "proto/message_translation/ssl_wrapper.h"

#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"

static constexpr float DEFAULT_FIELD_LINE_THICKNESS = 0.01f;

std::unique_ptr<SSLProto::SSL_WrapperPacket> createSSLWrapperPacket(
    std::unique_ptr<SSLProto::SSL_GeometryData> geometry_data,
    std::unique_ptr<SSLProto::SSL_DetectionFrame> detection_frame)
{
    auto wrapper_packet = std::make_unique<SSLProto::SSL_WrapperPacket>();
    if (geometry_data)
    {
        *(wrapper_packet->mutable_geometry()) = *geometry_data;
    }
    if (detection_frame)
    {
        *(wrapper_packet->mutable_detection()) = *detection_frame;
    }

    return wrapper_packet;
}
std::unique_ptr<SSLProto::SSL_WrapperPacket> createSSLWrapperPacket(
    const World& world, TeamColour friendly_team_colour)
{
    constexpr auto robot_to_robotstate_with_id_fn = [](const Robot& robot) {
        return RobotStateWithId{.id = robot.id(), .robot_state = robot.currentState()};
    };

    std::vector<RobotStateWithId> friendly_robot_states;
    friendly_robot_states.reserve(world.friendlyTeam().numRobots());
    std::transform(world.friendlyTeam().getAllRobots().begin(),
                   world.friendlyTeam().getAllRobots().end(),
                   std::back_inserter(friendly_robot_states),
                   robot_to_robotstate_with_id_fn);

    std::vector<RobotStateWithId> enemy_robot_states;
    enemy_robot_states.reserve(world.enemyTeam().numRobots());
    std::transform(
        world.enemyTeam().getAllRobots().begin(), world.enemyTeam().getAllRobots().end(),
        std::back_inserter(enemy_robot_states), robot_to_robotstate_with_id_fn);

    const auto& yellow_robot_states = friendly_team_colour == TeamColour::YELLOW
                                          ? friendly_robot_states
                                          : enemy_robot_states;

    const auto& blue_robot_states = friendly_team_colour == TeamColour::BLUE
                                        ? friendly_robot_states
                                        : enemy_robot_states;

    auto ssl_detectionframe = createSSLDetectionFrame(
        std::numeric_limits<uint32_t>::max(), world.getMostRecentTimestamp(),
        std::numeric_limits<uint32_t>::max(), {world.ball().currentState()},
        yellow_robot_states, blue_robot_states);

    auto ssl_geometrydata =
        createGeometryData(world.field(), DEFAULT_FIELD_LINE_THICKNESS);

    return createSSLWrapperPacket(std::move(ssl_geometrydata),
                                  std::move(ssl_detectionframe));
}
