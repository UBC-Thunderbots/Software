#include "software/geom/algorithms/generate_velocity_obstacle.h"

#include "software/geom/algorithms/intersects.h"

VelocityObstacle generateVelocityObstacle(const Circle& obstacle, const Circle& robot,
                                          const Vector& obstacle_velocity)
{
    const Vector robot_to_obstacle_vector        = obstacle.origin() - robot.origin();
    const Angle robot_relative_to_obstacle_angle = robot_to_obstacle_vector.orientation();

    // opening angle of each side is defined as the obstacle radius plus the robot radius
    Angle opening_angle = Angle::asin((robot.radius() + obstacle.radius()) /
                                      robot_to_obstacle_vector.length());

    if (intersects(obstacle, robot))
    {
        // The robot is colliding with obstacle.
        // Creates Velocity Obstacle with the sides being ~180 degrees
        // apart from each other (90 degrees relative to the robot to
        // obstacle vector).
        // Subtracting by a slight offset to avoid the velocity obstacle
        // from being the complement of what we want (since VOs are defined as
        // area created between the smallest angles between the two sides).
        opening_angle = Angle::quarter() - Angle::fromRadians(0.0001);
    }

    // Direction of the two edges of the velocity obstacle
    Vector side1 =
        Vector::createFromAngle(robot_relative_to_obstacle_angle - opening_angle);
    Vector side2 =
        Vector::createFromAngle(robot_relative_to_obstacle_angle + opening_angle);

    return VelocityObstacle(obstacle_velocity, side1, side2);
}

VelocityObstacle generateVelocityObstacle(const Polygon& obstacle, const Circle& robot,
                                          const Vector& obstacle_velocity)
{
    // The velocity obstacle of a polygon relative to a robot is calculated by finding the
    // two polygon vertices which create the widest opening relative to the robot position
    Angle min_opening = Angle::zero();
    Vector side1;
    Vector side2;
    for (unsigned int i = 0; i < obstacle.getPoints().size(); i++)
    {
        Vector obstacle_to_point_vec_i = obstacle.getPoints()[i] - robot.origin();
        Angle angle_i                  = obstacle_to_point_vec_i.orientation();
        for (unsigned int j = i + 1; j < obstacle.getPoints().size(); j++)
        {
            Vector obstacle_to_point_vec_j = obstacle.getPoints()[j] - robot.origin();
            Angle angle_j                  = obstacle_to_point_vec_j.orientation();

            Angle curr_opening = angle_i.minDiff(angle_j);
            if (curr_opening > min_opening)
            {
                min_opening = curr_opening;
                side1       = obstacle_to_point_vec_i;
                side2       = obstacle_to_point_vec_j;
            }
        }
    }

    // Open velocity obstacle sides by the radius of the robot
    Vector left_side  = side1;
    Vector right_side = side2;
    if (left_side.isClockwiseOf(right_side))
    {
        // Flip the left and right sides
        Vector temp = left_side;
        left_side   = right_side;
        right_side  = temp;
    }

    // Expand (i.e. rotate) velocity obstacle to take the robot radius into account
    // Constrain the inner angle of the velocity obstacle to be less than 180 degrees
    const Angle max_rotation =
        Angle::fromDegrees((179.9 - min_opening.toDegrees()) / 2.0);
    left_side = left_side.rotate(
        std::min(max_rotation, Angle::asin(robot.radius() / left_side.length())));
    right_side = right_side.rotate(
        -std::min(max_rotation, Angle::asin(robot.radius() / right_side.length())));

    return VelocityObstacle(obstacle_velocity, left_side, right_side);
}
