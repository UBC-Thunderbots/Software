#include "software/gui/drawing/ssl_wrapper_packet.h"

#include "software/gui/drawing/ball.h"
#include "software/gui/drawing/colors.h"
#include "software/gui/drawing/field.h"
#include "software/gui/drawing/robot.h"
#include "software/proto/message_translation/ssl_detection.h"
#include "software/proto/message_translation/ssl_geometry.h"

void drawSSLWrapperPacket(QGraphicsScene* scene,
                          const SSLProto::SSL_WrapperPacket& ssl_wrapper_packet)
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

        auto yellow_robot_detections =
            createTeamDetection({detection}, TeamColour::YELLOW);
        for (const auto& robot : yellow_robot_detections)
        {
            drawRobot(scene, robot, yellow_robot_color);
        }

        auto blue_robot_detections = createTeamDetection({detection}, TeamColour::BLUE);
        for (const auto& robot : blue_robot_detections)
        {
            drawRobot(scene, robot, blue_robot_color);
        }

        auto ball_detections = createBallDetections({detection});
        for (const auto& ball : ball_detections)
        {
            drawBall(scene, ball);
        }
    }
}

WorldDrawFunction getDrawSSLWrapperPacketFunction(
    const SSLProto::SSL_WrapperPacket& ssl_wrapper_packet)
{
    auto draw_function = [ssl_wrapper_packet](QGraphicsScene* scene) {
        drawSSLWrapperPacket(scene, ssl_wrapper_packet);
    };
    return WorldDrawFunction(draw_function);
}
