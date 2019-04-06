
#include "obstacle.h"

const Polygon& Obstacle::getBoundaryPolygon()
{
    return _polygon;
}

Obstacle::Obstacle() : _polygon{Point(0, 0), Point(0, 1), Point(1, 1), Point(1, 0)} {}

Obstacle::Obstacle(Polygon polygon) : _polygon(polygon) {}

Obstacle Obstacle::createPlaceholderObstacle()
{
    return Obstacle();
}

static Obstacle Obstacle::createRobotObstacle(const Robot& robot,
                                              bool enable_velocity_cushion)
{
    // TODO: figure out how to get tick length
    double tick_length = 1.0;

    // TODO: figure out actual factor (not 2) needed to clear two robot based on hexagon
    // implementation
    double radius_cushion = ROBOT_MAX_RADIUS_METERS * 2;

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector =
        robot.velocity().norm(robot.velocity().len() * tick_length);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion.len() > radius);
}

static Obstacle Obstacle::createRobotObstacleWithScalingParams(
    const Robot& robot, double radius_cushion_scaling, double velocity_cushion_scaling,
    bool enable_velocity_cushion)
{
    // TODO: figure out how to get tick length
    double tick_length = 1.0;

    // TODO: figure out actual factor (not 2) needed to clear two robot based on hexagon
    // implementation
    double radius_cushion = ROBOT_MAX_RADIUS_METERS * 2 * robot_radius_scaling;

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector = robot.velocity().norm(
        robot.velocity().len() * tick_length * velocity_cushion_scaling);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion.len() > radius);
}

static Obstacle Obstacle::createRobotObstacleWithBufferParams(
    const Robot& robot, double radius_cushion_buffer, double velocity_cushion_buffer,
    bool enable_velocity_cushion)
{
    // TODO: figure out how to get tick length
    double tick_length = 1.0;

    // TODO: figure out actual factor (not 2) needed to clear two robot based on hexagon
    // implementation
    double radius_cushion = ROBOT_MAX_RADIUS_METERS * 2 + radius_cushion_buffer;

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector = robot.velocity().norm(
        robot.velocity().len() * tick_length + velocity_cushion_buffer);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion.len() > radius);
}


static Obstacle Obstacle::createRobotObstacleFromPositionAndRadiusAndVelocity(
    Point position, double radius_cushion, Vector velocity_cushion_vector,
    bool enable_velocity_cushion)
{
    if (enable_velocity_cushion)
    {
        Vector velocity_direction_norm_radius = velocity_cushion.norm(radius_cushion);
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
                 velocity_cushion,
             // left side velocity cushions
             position + velocity_direction_norm_radius.rotate(Angle::quarter()) +
                 velocity_cushion}));
    }
    else
    {
        // force the robot to face in +x direction
        Vector facing_direction_norm_radius = Point(1, 0).norm(radius);
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
