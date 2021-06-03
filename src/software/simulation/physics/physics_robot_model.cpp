#include "software/simulation/physics/physics_robot_model.h"

#include <algorithm>
#include <vector>

#include "shared/constants.h"
#include "software/simulation/physics/box2d_util.h"

b2PolygonShape* PhysicsRobotModel::getMainRobotBodyShape(double total_dribbler_depth)
{
    const unsigned int num_shape_vertices = b2_maxPolygonVertices;
    b2Vec2 robot_body_vertices[num_shape_vertices];

    // Assuming the robot is at (0, 0) and facing the +x axis (aka has an orientation of
    // 0) First find the y-coordinate of the front-left edge of the body by solving x^2 +
    // y^2 = ROBOT_MAX_RADIUS_METERS^2
    double y =
        std::sqrt(std::pow(ROBOT_MAX_RADIUS_METERS, 2) -
                  std::pow(DIST_TO_FRONT_OF_ROBOT_METERS - total_dribbler_depth, 2));

    Vector starting_vector(DIST_TO_FRONT_OF_ROBOT_METERS - total_dribbler_depth, y);
    Angle starting_angle        = starting_vector.orientation();
    Angle angle_to_sweep_across = Angle::full() - (2 * starting_angle);
    // The angle increment is positive, so we rotate counter-clockwise and add points
    // to the polygon in counter-clockwise order, which is required by Box2D
    Angle angle_increment = angle_to_sweep_across / (num_shape_vertices - 1);
    for (unsigned int i = 0; i < num_shape_vertices; i++)
    {
        Angle angle_to_vertex = starting_angle + (i * angle_increment);
        Point vertex =
            Point(0, 0) +
            Vector::createFromAngle(angle_to_vertex).normalize(ROBOT_MAX_RADIUS_METERS);
        // The shape is added relative to the body, so we do not need to rotate these
        // points to match the robot's orientation
        robot_body_vertices[i] = createVec2(vertex);
    }

    b2PolygonShape* body_shape = new b2PolygonShape();
    body_shape->Set(robot_body_vertices, num_shape_vertices);
    return body_shape;
}

b2PolygonShape* PhysicsRobotModel::getRobotBodyShapeFrontLeft(double total_dribbler_depth)
{
    // These points are already give in counter-clockwise order, so we can directly create
    // a polygon (Box2D requires that polygon vertices are given in counter-clockwise
    // order)
    auto shape_points = getRobotFrontLeftShapePoints(total_dribbler_depth);

    // The shape is added relative to the body, so we do not need to rotate these points
    // to match the robot's orientation

    std::vector<b2Vec2> vertices(shape_points.size());
    for (unsigned int i = 0; i < shape_points.size(); i++)
    {
        vertices[i] = createVec2(shape_points.at(i));
    }

    b2PolygonShape* shape = new b2PolygonShape();
    shape->Set(vertices.data(), static_cast<int>(shape_points.size()));

    return shape;
}

b2PolygonShape* PhysicsRobotModel::getRobotBodyShapeFrontRight(
    double total_dribbler_depth)
{
    auto shape_points = getRobotFrontLeftShapePoints(total_dribbler_depth);

    // Mirror the points over the x-axis to get the points for the front-right shape
    std::transform(shape_points.begin(), shape_points.end(), shape_points.begin(),
                   [](const Point& p) { return Point(p.x(), -(p.y())); });

    // shape_points are initially given in counter-clockwise order, but since we
    // mirrored the points they are now in clockwise order, so we need to reverse them
    // again. Box2D requires that polygon vertices are given in counter-clockwise order
    std::reverse(shape_points.begin(), shape_points.end());

    // The shape is added relative to the body, so we do not need to rotate these points
    // to match the robot's orientation

    std::vector<b2Vec2> vertices(shape_points.size());
    for (unsigned int i = 0; i < shape_points.size(); i++)
    {
        vertices[i] = createVec2(shape_points.at(i));
    }

    b2PolygonShape* shape = new b2PolygonShape();
    shape->Set(vertices.data(), static_cast<int>(shape_points.size()));

    return shape;
}

std::vector<Point> PhysicsRobotModel::getRobotFrontLeftShapePoints(
    double total_dribbler_depth)
{
    // Assuming the robot is at (0, 0) and facing the +x axis (aka has an orientation of
    // 0) First find the y-coordinate of the front-left edge of the body by solving x^2 +
    // y^2 = ROBOT_MAX_RADIUS_METERS^2
    double y =
        std::sqrt(std::pow(ROBOT_MAX_RADIUS_METERS, 2) -
                  std::pow(DIST_TO_FRONT_OF_ROBOT_METERS - total_dribbler_depth, 2));

    // Box2D requires polygon vertices are provided in counter-clockwise order
    std::vector<Point> vertices{
        Point(DIST_TO_FRONT_OF_ROBOT_METERS - total_dribbler_depth, y),
        Point(DIST_TO_FRONT_OF_ROBOT_METERS - total_dribbler_depth,
              DRIBBLER_WIDTH_METERS / 2.0),
        Point(DIST_TO_FRONT_OF_ROBOT_METERS, DRIBBLER_WIDTH_METERS / 2.0),
        Point(DIST_TO_FRONT_OF_ROBOT_METERS, FRONT_OF_ROBOT_WIDTH_METERS / 2.0)};

    return vertices;
}
