#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"

VelocityObstacle::VelocityObstacle(Vector apex, Vector side1, Vector side2) : apex(apex)
{
    if (side1.isClockwiseOf(side2))
    {
        right_side = side1.normalize();
        left_side  = side2.normalize();
    }
    else if (side1.isCounterClockwiseOf(side2))
    {
        right_side = side2.normalize();
        left_side  = side1.normalize();
    }
    else
    {
        LOG(WARNING)
            << "VelocityObstacle: constructing a velocity obstacle with 180 degree sides can lead to undefined behaviour. { apex: "
            << apex << ", side1: " << side1 << ", side2: " << side2;
    }
}

bool VelocityObstacle::containsVelocity(const Vector& velocity) const
{
    Vector velocity_relative_to_obstacle = velocity - apex;

    return left_side.isCounterClockwiseOf(velocity_relative_to_obstacle) &&
           right_side.isClockwiseOf(velocity_relative_to_obstacle);
}

Vector VelocityObstacle::getApex() const
{
    return apex;
}

Vector VelocityObstacle::getLeftSide() const
{
    return left_side;
}

Vector VelocityObstacle::getRightSide() const
{
    return right_side;
}

VelocityObstacle VelocityObstacle::generateVelocityObstacle(
    const Circle& obstacle, const Circle& robot, const Vector& obstacle_velocity)
{
    const Vector robot_to_obstacle_vector        = obstacle.origin() - robot.origin();
    const Angle robot_relative_to_obstacle_angle = robot_to_obstacle_vector.orientation();

    // The robot is colliding with obstacle.
    // Creates Velocity Obstacle with the sides being 180 degrees
    // apart from each other (90 degrees relative to the robot to
    // obstacle vector) with center being the center of obstacle.
    // Subtracting by a slight offset to avoid the velocity obstacle
    // from being the complement of what we want (Since VOs are defined as
    // area created between the smallest angles between the two sides.
    Angle opening_angle = Angle::quarter() - Angle::fromRadians(0.0001);

    if (robot_to_obstacle_vector.lengthSquared() >
        std::pow(obstacle.radius() + robot.radius(), 2))
    {
        // opening angle of each side relative = arcsin((rad_A + rad_B) / distance)
        opening_angle = Angle::asin((robot.radius() + obstacle.radius()) /
                                    robot_to_obstacle_vector.length());
    }

    // Direction of the two edges of the velocity obstacle
    Vector side1 =
        Vector::createFromAngle(robot_relative_to_obstacle_angle - opening_angle);
    Vector side2 =
        Vector::createFromAngle(robot_relative_to_obstacle_angle + opening_angle);

    // Since the obstacle is static, the velocity obstacle (apex) is not shifted
    return VelocityObstacle(obstacle_velocity, side1, side2);
}
