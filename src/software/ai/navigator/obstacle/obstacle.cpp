
#include "software/ai/navigator/obstacle/obstacle.h"

const std::optional<Polygon> Obstacle::getBoundaryPolygon() const
{
    return _polygon;
}

const std::optional<Circle> Obstacle::getBoundaryCircle() const
{
    return _circle;
}

Obstacle::Obstacle(Polygon polygon) : _polygon(std::make_optional<Polygon>(polygon)) {}

Obstacle::Obstacle(Rectangle rectangle)
    : Obstacle({rectangle.negXNegYCorner(), rectangle.negXPosYCorner(),
                rectangle.posXPosYCorner(), rectangle.posXNegYCorner()})
{
}

Obstacle::Obstacle(Circle circle) : _polygon(std::nullopt), _circle(circle) {}

Obstacle Obstacle::createCircleObstacle(const Point& circle_centre,
                                        const double circle_radius,
                                        const double radius_scaling)
{
    // Add robot radius to account for path planning robot's size
    return Obstacle(Circle(circle_centre,
                           (circle_radius + ROBOT_MAX_RADIUS_METERS) * radius_scaling));
}

Obstacle Obstacle::createCircularRobotObstacle(const Robot& robot,
                                               double radius_cushion_scaling)
{
    return createCircleObstacle(robot.position(), ROBOT_MAX_RADIUS_METERS,
                                radius_cushion_scaling);
}


Obstacle Obstacle::createRobotObstacle(const Robot& robot, bool enable_velocity_cushion)
{
    double radius_cushion = getRadiusCushionForHexagon(ROBOT_MAX_RADIUS_METERS);

    // vector in the direction of the velocity and with the scaled size of the
    // velocity
    Vector velocity_cushion_vector =
        robot.velocity().normalize(robot.velocity().length());

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

    // velocity_cushion_vector should be at least as long as a robot width
    // with an additional factor that is half of the average of intial speed and
    // ROBOT_MAX_SPEED_METERS_PER_SECOND times the length_scaling
    Vector velocity_cushion_vector =
        (end - start)
            .normalize((initial_speed + ROBOT_MAX_SPEED_METERS_PER_SECOND) / 4 *
                           length_scaling +
                       2 * ROBOT_MAX_RADIUS_METERS * width_scaling);

    Vector velocity_direction_norm_radius =
        velocity_cushion_vector.normalize(radius_cushion);

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

    // vector in the direction of the velocity and with the scaled size of the
    // velocity
    Vector velocity_cushion_vector =
        robot.velocity().normalize(robot.velocity().length() * velocity_cushion_scaling +
                                   2 * ROBOT_MAX_RADIUS_METERS * radius_cushion_scaling);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        velocity_cushion_vector.length() > radius_cushion);
}

Obstacle Obstacle::createRobotObstacleWithBufferParams(
    const Robot& robot, bool enable_velocity_cushion,
    double additional_radius_cushion_buffer, double additional_velocity_cushion_buffer)
{
    double radius_cushion = getRadiusCushionForHexagon(ROBOT_MAX_RADIUS_METERS +
                                                       additional_radius_cushion_buffer);

    // vector in the direction of the velocity and with the scaled size of the
    // velocity
    Vector velocity_cushion_vector = robot.velocity().normalize(
        robot.velocity().length() + additional_velocity_cushion_buffer);

    return createRobotObstacleFromPositionAndRadiusAndVelocity(
        robot.position(), radius_cushion, velocity_cushion_vector,
        enable_velocity_cushion && velocity_cushion_vector.length() > radius_cushion);
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
            velocity_cushion_vector.normalize(radius_cushion);
        return Obstacle(Polygon(
            {// left side of robot
             position + velocity_direction_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             position + velocity_direction_norm_radius.rotate(Angle::fromDegrees(150)),
             // back right of robot
             position + velocity_direction_norm_radius.rotate(Angle::fromDegrees(210)),
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
        Vector facing_direction_norm_radius = Vector(1, 0).normalize(radius_cushion);
        return Obstacle(Polygon(
            {// left side of robot
             position + facing_direction_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             position + facing_direction_norm_radius.rotate(Angle::fromDegrees(150)),
             // back right of robot
             position + facing_direction_norm_radius.rotate(Angle::fromDegrees(210)),
             // right side of robot
             position + facing_direction_norm_radius.rotate(Angle::threeQuarter()),
             // front right velocity cushions
             position + facing_direction_norm_radius.rotate(Angle::fromDegrees(330)),
             // front left velocity cushions
             position + facing_direction_norm_radius.rotate(Angle::fromDegrees(30))}));
    }
}

// this is a helper function
double Obstacle::getRadiusCushionForHexagon(double radius)
{
    // return radius cushion so that centre to side distance is at least double given
    // radius so that two robots can pass by each other this is accomplished by
    // doubling the radius and multiplying by 2/sqrt(3)
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

Obstacle Obstacle::createCircularBallObstacle(const Ball& ball,
                                              double additional_radius_cushion_buffer)
{
    return createCircleObstacle(
        ball.position(), BALL_MAX_RADIUS_METERS + additional_radius_cushion_buffer, 1);
}

bool Obstacle::containsPoint(const Point& point) const
{
    if (isPolygon())
    {
        return (*_polygon).containsPoint(point);
    }
    else
    {
        return ((point - (*_circle).getOrigin()).length() < (*_circle).getRadius());
    }
}

bool Obstacle::intersects(const Segment& segment) const
{
    if (isPolygon())
    {
        return (*_polygon).intersects(segment);
    }
    else
    {
        return (dist((*_circle).getOrigin(), segment) <= (*_circle).getRadius());
    }
}

bool Obstacle::isPolygon() const
{
    return (bool)_polygon;
}
