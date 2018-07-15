#ifndef THUNDERBOTS_VISION_UTIL_H
#define THUNDERBOTS_VISION_UTIL_H

#include "geom/point.h"
#include "proto/messages_robocup_ssl_geometry.pb.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"

class VisionUtil
{
   public:
    /**
     * Creates a Field msg given raw SSL Field Geometry data
     *
     * @param field_data the SSL Geometry packet containing field geometry data
     *
     * @return a Field message containing the field geometry information
     */
    static thunderbots_msgs::Field createFieldMsg(
        const SSL_GeometryFieldSize &field_data);

    /**
     * Creates a Ball msg given the position and velocity of a ball
     *
     * @param position the position of the ball
     * @param velocity the velocity of the ball
     *
     * @return a Ball message containing the ball information
     */
    static thunderbots_msgs::Ball createBallMsg(
        const Point &position, const Point &velocity);
};

#endif  // THUNDERBOTS_VISION_UTIL_H
