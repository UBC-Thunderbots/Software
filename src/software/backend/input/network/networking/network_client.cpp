#include "software/backend/input/network/networking/network_client.h"

#include <boost/bind.hpp>
#include <limits>

#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/parameter/config.hpp"
#include "software/parameter/dynamic_parameters.h"

NetworkClient::NetworkClient(std::string vision_multicast_address,
                             int vision_multicast_port,
                             std::string gamecontroller_multicast_address,
                             int gamecontroller_multicast_port,
                             std::function<void(World)> received_world_callback,
                             std::shared_ptr<const RefboxConfig> refbox_config,
                             std::shared_ptr<const CameraConfig> camera_config)
    : network_filter(refbox_config),
      last_valid_t_capture(std::numeric_limits<double>::max()),
      initial_packet_count(0),
      received_world_callback(received_world_callback),
      refbox_config(refbox_config),
      camera_config(camera_config)
{
    ssl_vision_client =
        std::make_unique<ThreadedProtoMulticastListener<SSL_WrapperPacket>>(
            vision_multicast_address, vision_multicast_port,
            boost::bind(&NetworkClient::filterAndPublishVisionDataWrapper, this, _1));

    ssl_gamecontroller_client = std::make_unique<ThreadedProtoMulticastListener<Referee>>(
        gamecontroller_multicast_address, gamecontroller_multicast_port,
        boost::bind(&NetworkClient::filterAndPublishGameControllerData, this, _1));
}

void NetworkClient::filterAndPublishVisionDataWrapper(SSL_WrapperPacket packet)
{
    // We analyze the first 60 packets we receive to find the "real" starting time.
    // The real starting time is the smaller value of the ones we receive
    if (initial_packet_count < 60)
    {
        initial_packet_count++;
        if (packet.has_detection() &&
            packet.detection().t_capture() < last_valid_t_capture)
        {
            last_valid_t_capture = packet.detection().t_capture();
        }
    }
    else
    {
        // We pass all packets without a detection to the logic (since they are likely
        // geometry packet). Packets with detection timestamps are compared to the last
        // valid timestamp to make sure they are close enough before the data is passed
        // along. This ensures we ignore any of the garbage packets grsim sends that
        // are thousands of seconds in the future.
        if (!packet.has_detection())
        {
            filterAndPublishVisionData(packet);
        }
        else if (std::fabs(packet.detection().t_capture() - last_valid_t_capture) < 100)
        {
            last_valid_t_capture = packet.detection().t_capture();
            filterAndPublishVisionData(packet);
        }
    }
}

void NetworkClient::filterAndPublishVisionData(SSL_WrapperPacket packet)
{
    if (packet.has_geometry())
    {
        const auto& latest_geometry_data = packet.geometry();
        field = network_filter.getFieldData(latest_geometry_data);
    }

    if (field)
    {
        if (packet.has_detection())
        {
            SSL_DetectionFrame detection = *packet.mutable_detection();
            bool camera_disabled         = false;

            // We invert the field side if we explicitly choose to override the values
            // provided by refbox. The 'defending_positive_side' parameter dictates the
            // side we are defending if we are overriding the value
            if (refbox_config->OverrideRefboxDefendingSide()->value() &&
                refbox_config->DefendingPositiveSide()->value())
            {
                invertFieldSide(detection);
            }
            // TODO remove as part of
            // https://github.com/UBC-Thunderbots/Software/issues/960
            switch (detection.camera_id())
            {
                case 0:
                    camera_disabled = camera_config->IgnoreCamera_0()->value();
                    break;
                case 1:
                    camera_disabled = camera_config->IgnoreCamera_1()->value();
                    break;
                case 2:
                    camera_disabled = camera_config->IgnoreCamera_2()->value();
                    break;
                case 3:
                    camera_disabled = camera_config->IgnoreCamera_3()->value();
                    break;
                default:
                    LOG(WARNING)
                        << "An unknown camera id was detected, disabled by default "
                        << "id: " << detection.camera_id() << std::endl;
                    camera_disabled = true;
                    break;
            }

            if (!camera_disabled)
            {
                std::optional<TimestampedBallState> ball_state =
                    network_filter.getFilteredBallData({detection});
                if (ball_state)
                {
                    if (ball)
                    {
                        ball->updateState(*ball_state);
                    }
                    else
                    {
                        ball = Ball(*ball_state);
                    }
                }

                friendly_team = network_filter.getFilteredFriendlyTeamData({detection});
                int friendly_goalie_id = refbox_config->FriendlyGoalieId()->value();
                friendly_team.assignGoalie(friendly_goalie_id);

                enemy_team = network_filter.getFilteredEnemyTeamData({detection});
                int enemy_goalie_id = refbox_config->EnemyGoalieId()->value();
                enemy_team.assignGoalie(enemy_goalie_id);
            }
        }
    }
    if (field && ball)
    {
        received_world_callback(World(*field, *ball, friendly_team, enemy_team));
    }
}

void NetworkClient::filterAndPublishGameControllerData(Referee packet)
{
    if (field && ball)
    {
        RefboxGameState game_state = network_filter.getRefboxGameState(packet);
        World world(*field, *ball, friendly_team, enemy_team);
        world.updateGameState(game_state);
        received_world_callback(world);
    }
}

void NetworkClient::invertFieldSide(SSL_DetectionFrame& frame)
{
    for (SSL_DetectionBall& ball : *frame.mutable_balls())
    {
        ball.set_x(-ball.x());
        ball.set_y(-ball.y());
    }
    for (const auto& team : {frame.mutable_robots_yellow(), frame.mutable_robots_blue()})
    {
        for (SSL_DetectionRobot& robot : *team)
        {
            robot.set_x(-robot.x());
            robot.set_y(-robot.y());
            robot.set_orientation(robot.orientation() + M_PI);
        }
    }
}
