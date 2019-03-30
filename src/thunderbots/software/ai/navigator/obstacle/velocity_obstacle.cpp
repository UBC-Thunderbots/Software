#include "ai/navigator/obstacle/velocity_obstacle.h"
Polygon VelocityObstacle::getBoundaryPolygon(const Robot& robot,
                                             double robot_radius_scaling,
                                             double velocity_projection_scaling)
{
    double radius             = ROBOT_MAX_RADIUS_METERS * robot_radius_scaling;
    Point velocity_projection = robot.velocity() * velocity_projection_scaling;
    Point velocity_direction  = robot.velocity().norm();

    if (velocity_projection.len() > radius)
    {
        return Polygon(
            {// sides of robot
             robot.position() + radius * velocity_direction.rotate(Angle::quarter()),
             robot.position() + radius * velocity_direction.rotate(Angle::threeQuarter()),
             // velocity projections
             robot.position() + radius * velocity_direction.rotate(Angle::quarter()) +
                 velocity_projection,
             robot.position() +
                 radius * velocity_direction.rotate(Angle::threeQuarter()) +
                 velocity_projection,
             // back of robot
             robot.position() + radius * velocity_direction.rotate(Angle::ofDegrees(150)),
             robot.position() +
                 radius * velocity_direction.rotate(Angle::ofDegrees(210))});
    }
    else
    {
        // force velocity direction to the +x direction (arbitrary)
        velocity_direction = Point(1, 0);
        return Polygon(
            {// sides of robot
             robot.position() + radius * velocity_direction.rotate(Angle::quarter()),
             robot.position() + radius * velocity_direction.rotate(Angle::threeQuarter()),
             // front of robot
             robot.position() + radius * velocity_direction.rotate(Angle::ofDegrees(30)),
             robot.position() + radius * velocity_direction.rotate(Angle::ofDegrees(330)),
             // back of robot
             robot.position() + radius * velocity_direction.rotate(Angle::ofDegrees(150)),
             robot.position() +
                 radius * velocity_direction.rotate(Angle::ofDegrees(210))});
    }
}
