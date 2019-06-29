
#include "obstacle.h"

const Polygon& Obstacle::getBoundaryPolygon() const
{
    return _polygon;
}

Obstacle::Obstacle(Polygon polygon) : _polygon(polygon) {}

Obstacle Obstacle::createRobotObstacle(const Robot& robot, bool enable_velocity_cushion)
{
    double radius_cushion = getRadiusCushionForHexagon(ROBOT_MAX_RADIUS_METERS);

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector = robot.velocity().norm(robot.velocity().len());

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion);
}

Obstacle Obstacle::createVelocityObstacleWithScalingParams(Point start, Point end,
                                                           double initial_speed,
                                                           double width_scaling,
                                                           double length_scaling)
{
    double radius_cushion =
        getRadiusCushionForHexagon(ROBOT_MAX_RADIUS_METERS * width_scaling);

    Vector velocity_cushion_vector =
        (end - start)
            .norm((initial_speed + ROBOT_MAX_SPEED_METERS_PER_SECOND) / 2 *
                  length_scaling);

    Vector velocity_direction_norm_radius = velocity_cushion_vector.norm(radius_cushion);

    return Obstacle(
        Polygon({// left side of robot
                 start + velocity_direction_norm_radius.rotate(Angle::quarter()),
                 // right side of robot
                 start + velocity_direction_norm_radius.rotate(Angle::threeQuarter()),
                 // right side velocity cushions
                 start + velocity_direction_norm_radius.rotate(Angle::threeQuarter()) +
                     velocity_cushion_vector,
                 // left side velocity cushions
                 start + velocity_direction_norm_radius.rotate(Angle::quarter()) +
                     velocity_cushion_vector}));
}

Obstacle Obstacle::createRobotObstacleWithScalingParams(const Robot& robot,
                                                        double radius_cushion_scaling,
                                                        double velocity_cushion_scaling)
{
    double radius_cushion =
        getRadiusCushionForHexagon(ROBOT_MAX_RADIUS_METERS * radius_cushion_scaling);

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector =
        robot.velocity().norm(robot.velocity().len() * velocity_cushion_scaling);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        velocity_cushion_vector.len() > radius_cushion);
}

Obstacle Obstacle::createRobotObstacleWithBufferParams(
    const Robot& robot, bool enable_velocity_cushion,
    double additional_radius_cushion_buffer, double additional_velocity_cushion_buffer)
{
    double radius_cushion = getRadiusCushionForHexagon(ROBOT_MAX_RADIUS_METERS +
                                                       additional_radius_cushion_buffer);

    // vector in the direction of the velocity and with the scaled size of the velocity
    Vector velocity_cushion_vector = robot.velocity().norm(
        robot.velocity().len() + additional_velocity_cushion_buffer);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion_vector.len() > radius_cushion);
}

// TODO: using this for more then robots now, should probably rename
// This is a helper function to factor out some of the logic from the other functions
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

// this is a helper function
double Obstacle::getRadiusCushionForHexagon(double radius)
{
    // return radius cushion so that centre to side distance is at least double given
    // radius so that two robots can pass by each other this is accomplished by doubling
    // the radius and multiplying by 2/sqrt(3)
    return radius * 4.0 / std::sqrt(3);
}

Obstacle Obstacle::createBallObstacle(const Ball& ball,
                                      double additional_radius_cushion_buffer,
                                      double additional_velocity_cushion_buffer)
{
    // TODO: handle `additional_velocity_cushion_buffer` here

    double radius_cushion = getRadiusCushionForHexagon(BALL_MAX_RADIUS_METERS +
                                                       additional_radius_cushion_buffer);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        ball.position(), radius_cushion, ball.velocity(), false);
}
