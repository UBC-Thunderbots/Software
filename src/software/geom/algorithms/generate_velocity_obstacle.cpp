#include "software/geom/algorithms/generate_velocity_obstacle.h"

#include "software/geom/geom_constants.h"

// TODO: Could add velocity aswell...
Agent::VelocityObstacle generateVelocityObstacle(const Circle &robot, const Circle& obstacle)
{
    Agent::VelocityObstacle velocity_obstacle;
    // Since the obstacle is static, the velocity obstacle is not shifted
    velocityObstacle.apex_ = Vector();

    const Vector robot_to_obstacle_vector = obstacle.origin() - robot.origin();
    const Angle robot_relative_to_obstacle_angle = robot_to_obstacle_vector.orientation();

    // opening angle of each side relative = arcsin((rad_A + rad_B) / distance)
    const Angle opening_angle = Angle::asin((robot.radius() + obstacle.radius()) /
                                            robot_to_obstacle_vector.length());

    if (robot_to_obstacle_vector.lengthSquared() <
        std::pow(obstacle.radius() + robot.radius(), 2))
    {
        // The robot is colliding with obstacle
        // Creates Velocity Obstacle with the sides being 180 degrees
        // apart from each other (90 degrees relative to the robot to obstacle vector)
        // TODO: Added - infront because the initial implementation had the vector backwards
        //       I thought it might mess up the left and right assumptions of side1/2
        opening_angle = Angle::quarter();
    }

    // Direction of the two edges of the velocity obstacle
    velocity_obstacle.side1_ = Vector::createFromAngle(robot_relative_to_obstacle_angle - opening_angle);
    velocity_obstacle.side2_ = Vector::createFromAngle(robot_relative_to_obstacle_angle + opening_angle);

    return velocity_obstacle;
}

Agent::VelocityObstacle generateVelocityObstacle(const Circle &robot, const Polygon &obstacle)
{
    // TODO: What happen when colliding? Random behaviour depending on which edges robot
    //       is closest to... Should be try to get away from centeroid of polygon?
    Agent::VelocityObstacle velocity_obstacle;

    // The velocity obstacle of a polygon relative to a robot is calculated by finding the
    // two polygon vertices which create the widest opening relative to the robot position
    Angle min_opening = Angle::zero();
    Vector left_side;
    Vector right_side;
    
    for (int i = 0; i < obstacle.getPoints().size(); i++)
    {
        Vector obstacle_to_point_vec_i  = obstacle.getPoints()[i] - robot.origin();
        Angle angle_i = obstacle_to_point_vec_i.orientation();
        for (int j = i + 1; j < obstacle.getPoints().size(); j++)
        {
            Vector obstacle_to_point_vec_j = obstacle.getPoints()[j] - robot.origin();
            Angle angle_j = obstacle_to_point_vec_j.orientation();

            Angle curr_opening = angle_i.minDiff(angle_j);
            if (curr_opening > min_opening)
            {
                // TODO: Update to isToTheRightOf
                if (obstacle_to_point_vec_i.determinant(obstacle_to_point_vec_j) < 0.0)
                {
                    left_side = obstacle_to_point_vec_i;
                    right_side = obstacle_to_point_vec_j;
                }
                else
                {
                    left_side = obstacle_to_point_vec_j;
                    right_side = obstacle_to_point_vec_i;
                }
            }
        }
    }

    // Open velocity obstacle sides by the radius of the robot
    velocity_obstacle.side1_ = left_side.rotate(Angle::asin(robot.radius() / left_side.length()));
    velocity_obstacle.side2_ = right_side.rotate(-Angle::asin(robot.radius() / right_side.length()));

    return velocity_obstacle;
}
