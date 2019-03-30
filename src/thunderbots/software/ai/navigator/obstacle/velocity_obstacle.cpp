#include "ai/navigator/obstacle/velocity_obstacle.h"
Polygon VelocityObstacle::getBoundaryPolygon(const Robot& robot,
                                             double robot_radius_scaling,
                                             double velocity_projection_scaling)
{
    double radius = ROBOT_MAX_RADIUS_METERS * robot_radius_scaling;
    Vector velocity_projection =
        robot.velocity().norm(velocity_projection_scaling * robot.velocity().len());

    if (velocity_projection.len() > radius)
    {
        Vector velocity_direction_norm_radius = robot.velocity().norm(radius);
        return Polygon(
            {// sides of robot
             robot.position() + velocity_direction_norm_radius.rotate(Angle::quarter()),
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::threeQuarter()),
             // velocity projections
             robot.position() + velocity_direction_norm_radius.rotate(Angle::quarter()) +
                 velocity_projection,
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::threeQuarter()) +
                 velocity_projection,
             // back of robot
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::ofDegrees(150)),
             robot.position() +
                 velocity_direction_norm_radius.rotate(Angle::ofDegrees(210))});
    }
    else
    {
        // force the robot to face in +x direction
        Vector facing_direction_norm_radius = Point(1, 0).norm(radius);
        return Polygon(
            {// sides of robot
             robot.position() + facing_direction_norm_radius.rotate(Angle::quarter()),
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::threeQuarter()),
             // front of robot
             robot.position() + facing_direction_norm_radius.rotate(Angle::ofDegrees(30)),
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::ofDegrees(330)),
             // back of robot
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::ofDegrees(150)),
             robot.position() +
                 facing_direction_norm_radius.rotate(Angle::ofDegrees(210))});
    }
}
