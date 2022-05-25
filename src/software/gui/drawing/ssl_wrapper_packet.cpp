#include "software/gui/drawing/ssl_wrapper_packet.h"

#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"
#include "software/gui/drawing/ball.h"
#include "software/gui/drawing/colors.h"
#include "software/gui/drawing/field.h"
#include "software/gui/drawing/robot.h"

void drawSSLWrapperPacket(QGraphicsScene* scene,
                          const SSLProto::SSL_WrapperPacket& ssl_wrapper_packet,
                          const RobotConstants_t& robot_constants)
{
    if (ssl_wrapper_packet.has_geometry())
    {
        auto geometry = ssl_wrapper_packet.geometry();
        auto field    = createField(geometry);
        if (field)
        {
            drawField(scene, field.value());
        }
    }

    if (ssl_wrapper_packet.has_detection())
    {
        const auto detection = ssl_wrapper_packet.detection();

        auto ball_detections = createBallDetections({detection});
        for (const auto& ball : ball_detections)
        {
            drawBall(scene, ball);
        }

        auto yellow_robot_detections =
            createTeamDetection({detection}, TeamColour::YELLOW);
        for (const auto& robot : yellow_robot_detections)
        {
            drawRobot(scene, robot, yellow_robot_color, robot_constants);
        }

        auto blue_robot_detections = createTeamDetection({detection}, TeamColour::BLUE);
        for (const auto& robot : blue_robot_detections)
        {
            drawRobot(scene, robot, blue_robot_color, robot_constants);
        }
    }
}

WorldDrawFunction getDrawSSLWrapperPacketFunction(
    const SSLProto::SSL_WrapperPacket& ssl_wrapper_packet,
    const RobotConstants_t& robot_constants)
{
    auto draw_function = [ssl_wrapper_packet, robot_constants](QGraphicsScene* scene) {
        drawSSLWrapperPacket(scene, ssl_wrapper_packet, robot_constants);
    };
    return WorldDrawFunction(draw_function);
}
