#include "software/geom/algorithms/generate_velocity_obstacle.h"

#include "software/geom/geom_constants.h"

// TODO: Could add velocity aswell...
Agent::VelocityObstacle generateVelocityObstacle(const Circle &circle, const Agent& agent)
{
    Agent::VelocityObstacle velocity_obstacle;
    velocityObstacle.apex_ = circle.origin();

    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent

        const float angle =
                (position_ - other_agent.getPosition()).orientation().toRadians();

        // opening angle = arcsin((rad_A + rad_B) / distance) TODO: Double check this, type should be angle, could probably be improved in terms of opening_angle to be tangent to circle
        const float opening_angle =
                std::asin((agent.getRadius() + circle.radius()) /
                          (circle.origin() - agent.getPosition()).length());

        // Direction of the two edges of the velocity obstacle TODO: Can probably use Vector::createFromAngle() instead
        velocity_obstacle.side1_ =
                Vector(std::cos(angle - opening_angle), std::sin(angle - opening_angle));
        velocity_obstacle.side2_ =
                Vector(std::cos(angle + opening_angle), std::sin(angle + opening_angle));
    }
    else
    {
        // This Agent is colliding with other agent
        // Creates Velocity Obstacle with the sides being 180 degrees
        // apart from each other
        velocity_obstacle.side1_ =
                (agent.getPosition() - circle.origin()).perpendicular().normalize();
        velocity_obstacle.side2_ = -velocity_obstacle.side1_;
    }
    return velocity_obstacle;
}

Agent::VelocityObstacle generateVelocityObstacle(const Rectangle &rectangle, const Agent& agent)
{
    Agent::VelocityObstacle velocity_obstacle;

    return velocity_obstacle;
}

Agent::VelocityObstacle generateVelocityObstacle(const Polygon &polygon, const Agent& agent)
{
    Agent::VelocityObstacle velocity_obstacle;

    return velocity_obstacle;
}
