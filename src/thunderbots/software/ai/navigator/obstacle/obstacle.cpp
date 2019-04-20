
#include "obstacle.h"

const Polygon& Obstacle::getBoundaryPolygon() const
{
    return _polygon;
}

Obstacle::Obstacle() : _polygon{Point(0, 0), Point(0, 1), Point(1, 1), Point(1, 0)} {}

Obstacle::Obstacle(Polygon polygon) : _polygon(polygon) {}

Obstacle Obstacle::createPlaceholderObstacle()
{
    return Obstacle();
}

Obstacle Obstacle::createRobotObstacle(const Robot& robot, bool enable_velocity_cushion)
{
    double tick_length = 1.0;

    double radius_cushion = getRadiusForHexagon(ROBOT_MAX_RADIUS_METERS);

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector =
        robot.velocity().norm(robot.velocity().len() * tick_length);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion);
}

Obstacle Obstacle::createRobotObstacleWithScalingParams(const Robot& robot,
                                                        bool enable_velocity_cushion,
                                                        double radius_cushion_scaling,
                                                        double velocity_cushion_scaling,
                                                        double tick_length)
{
    double radius_cushion =
        getRadiusForHexagon(ROBOT_MAX_RADIUS_METERS * radius_cushion_scaling);

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector = robot.velocity().norm(
        robot.velocity().len() * tick_length * velocity_cushion_scaling);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion_vector.len() > radius_cushion);
}

Obstacle Obstacle::createRobotObstacleWithBufferParams(
    const Robot& robot, bool enable_velocity_cushion,
    double additional_radius_cushion_buffer, double additional_velocity_cushion_buffer,
    double tick_length)
{
    double radius_cushion =
        getRadiusForHexagon(ROBOT_MAX_RADIUS_METERS + additional_radius_cushion_buffer);

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector = robot.velocity().norm(
        robot.velocity().len() * tick_length + additional_velocity_cushion_buffer);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion_vector.len() > radius_cushion);
}


Obstacle Obstacle::createRobotObstacleFromPositionAndRadiusAndVelocity(
    Point position, double radius_cushion, Vector velocity_cushion_vector,
    bool enable_velocity_cushion)
{
    if (enable_velocity_cushion)
    {
        Vector velocity_direction_norm_radius =
            velocity_cushion_vector.norm(radius_cushion);
        return Obstacle(Polygon(
            {// left side of robot
             position + velocity_direction_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             position + velocity_direction_norm_radius.rotate(Angle::ofDegrees(150)),
             // back right of robot
             position + velocity_direction_norm_radius.rotate(Angle::ofDegrees(210)),
             // right side of robot
             position + velocity_direction_norm_radius.rotate(Angle::threeQuarter()),
             // right side velocity cushions
             position + velocity_direction_norm_radius.rotate(Angle::threeQuarter()) +
                 velocity_cushion_vector,
             // left side velocity cushions
             position + velocity_direction_norm_radius.rotate(Angle::quarter()) +
                 velocity_cushion_vector}));
    }
    else
    {
        // force the robot to face in +x direction
        Vector facing_direction_norm_radius = Point(1, 0).norm(radius_cushion);
        return Obstacle(Polygon(
            {// left side of robot
             position + facing_direction_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             position + facing_direction_norm_radius.rotate(Angle::ofDegrees(150)),
             // back right of robot
             position + facing_direction_norm_radius.rotate(Angle::ofDegrees(210)),
             // right side of robot
             position + facing_direction_norm_radius.rotate(Angle::threeQuarter()),
             // front right velocity cushions
             position + facing_direction_norm_radius.rotate(Angle::ofDegrees(330)),
             // front left velocity cushions
             position + facing_direction_norm_radius.rotate(Angle::ofDegrees(30))}));
    }
}

double Obstacle::getRadiusForHexagon(double radius)
{
    // return radius so that centre to side distance is at least radius
    return radius * 4 / std::sqrt(3);
}
