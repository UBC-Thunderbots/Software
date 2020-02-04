#include "software/backend/input/network/network_client.h"

#include <boost/bind.hpp>
#include <g3log/g3log.hpp>
#include <limits>

#include "software/constants.h"
#include "software/parameter/dynamic_parameters.h"

NetworkClient::NetworkClient(
    std::string vision_multicast_address, int vision_multicast_port,
    std::string gamecontroller_multicast_address, int gamecontroller_multicast_port,
    std::function<void(VisionDetection)> received_vision_detection_callback,
    std::function<void(RefboxData)> received_refbox_data_callback)
    : ssl_protobuf_reader(),
      io_service(),
      last_valid_t_capture(std::numeric_limits<double>::max()),
      initial_packet_count(0),
      received_vision_detection_callback(received_vision_detection_callback),
      received_refbox_data_callback(received_refbox_data_callback)
{
    setupVisionClient(vision_multicast_address, vision_multicast_port);

    setupGameControllerClient(gamecontroller_multicast_address,
                              gamecontroller_multicast_port);

    startIoServiceThreadInBackground();
}

void NetworkClient::setupVisionClient(std::string vision_address, int vision_port)
{
    // Set up our connection over udp to receive vision packets
    try
    {
        ssl_vision_client = std::make_unique<SSLVisionClient>(
            io_service, vision_address, vision_port,
            boost::bind(&NetworkClient::publishVisionDataWrapper, this, _1));
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate this process
        LOG(FATAL) << "An error occured while setting up the SSL Vision Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }
}

void NetworkClient::setupGameControllerClient(std::string gamecontroller_address,
                                              int gamecontroller_port)
{
    // Set up our connection over udp to receive gamecontroller packets
    try
    {
        ssl_gamecontroller_client = std::make_unique<SSLGameControllerClient>(
            io_service, gamecontroller_address, gamecontroller_port,
            boost::bind(&NetworkClient::publishGameControllerData, this, _1));
    }
    catch (const boost::exception& ex)
    {
        // LOG(FATAL) will terminate this process
        LOG(FATAL) << "An error occured while setting up the SSL GameController Client:"
                   << std::endl
                   << boost::diagnostic_information(ex) << std::endl;
    }
}

void NetworkClient::startIoServiceThreadInBackground()
{
    // Start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

NetworkClient::~NetworkClient()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}

void NetworkClient::publishVisionDataWrapper(SSL_WrapperPacket packet)
{
    // We analyze the first 60 packets we receive to find the "real" starting time,
    // because GrSim will sometimes give us garbage packets.
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
            publishVisionData(packet);
        }
        else if (std::fabs(packet.detection().t_capture() - last_valid_t_capture) < 100)
        {
            last_valid_t_capture = packet.detection().t_capture();
            publishVisionData(packet);
        }
    }
}

void NetworkClient::publishVisionData(SSL_WrapperPacket packet)
{
    std::vector<BallDetection> ball_detections;
    std::vector<RobotDetection> friendly_team_detections;
    std::vector<RobotDetection> enemy_team_detections;
    std::optional<Field> field_detection;
    Timestamp latest_timestamp;

    if (packet.has_geometry())
    {
        const auto& latest_geometry_data = packet.geometry();
        Field field = ssl_protobuf_reader.getFieldData(latest_geometry_data);
        world.updateFieldGeometry(field);
        field_detection = field;
    }

    if (packet.has_detection())
    {
        SSL_DetectionFrame detection = *packet.mutable_detection();

        // We invert the field side if we explicitly choose to override the values
        // provided by refbox. The 'defending_positive_side' parameter dictates the side
        // we are defending if we are overriding the value
        // TODO remove as part of https://github.com/UBC-Thunderbots/Software/issues/960
        if (Util::DynamicParameters->getAIControlConfig()
                ->getRefboxConfig()
                ->OverrideRefboxDefendingSide()
                ->value() &&
            Util::DynamicParameters->getAIControlConfig()
                ->getRefboxConfig()
                ->DefendingPositiveSide()
                ->value())
        {
            invertFieldSide(detection);
        }

        if (isCameraEnabled(detection))
        {
            // filter protos into internal data structures
            ball_detections = ssl_protobuf_reader.getBallDetections({detection});
            friendly_team_detections =
                ssl_protobuf_reader.getTeamDetections({detection}, TeamType::FRIENDLY);
            enemy_team_detections =
                ssl_protobuf_reader.getTeamDetections({detection}, TeamType::ENEMY);
        }

        latest_timestamp = Timestamp::fromSeconds(detection.t_capture());
    }

    VisionDetection vision_detection(ball_detections, friendly_team_detections,
                                     enemy_team_detections, field_detection,
                                     latest_timestamp);
    received_vision_detection_callback(vision_detection);
}

void NetworkClient::publishGameControllerData(Referee packet)
{
    RefboxData refbox_data = ssl_protobuf_reader.getRefboxData(packet);
    received_refbox_data_callback(refbox_data);
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

bool NetworkClient::isCameraEnabled(const SSL_DetectionFrame& detection)
{
    bool camera_disabled = false;
    switch (detection.camera_id())
    {
        // TODO: create an array of dynamic params to index into with camera_id()
        // may be resolved by https://github.com/UBC-Thunderbots/Software/issues/960
        case 0:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_0()->value();
            break;
        case 1:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_1()->value();
            break;
        case 2:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_2()->value();
            break;
        case 3:
            camera_disabled =
                Util::DynamicParameters->getCameraConfig()->IgnoreCamera_3()->value();
            break;
        default:
            LOG(WARNING) << "An unkown camera id was detected, disabled by default "
                         << "id: " << detection.camera_id() << std::endl;
            camera_disabled = true;
            break;
    }
    return !camera_disabled;
}
