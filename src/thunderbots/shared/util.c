#include "util.h"
#include "constants.h"

float dot2D(Vector2D first, Vector2D second)
{
    return first.x * second.x + first.y * second.y;
}

Vector2D toRobotLocalCoords(Vector2D robot_position, float robot_orientation,
                            Vector2D point)
{
    Vector2D relativePoint = {.x = point.x - robot_position.x,
                              .y = point.y - robot_position.y};

    Vector2D result = {.x = 0.0f, .y = 0.0f};

    // clamp angle to [0, 2 * pi]
    robot_orientation = fmod(robot_orientation, 2 * P_PI);

    // the Robot's X-axis is along the orientation of the robot,
    // and the Y-axis is perpendicular to it

    // rotation matrix copy-pasted from physbot.c in firmware
    Vector2D rotationMatrix[2] = {
        {.x = cosf(robot_orientation), .y = sinf(robot_orientation)},
        {.x = cosf(robot_orientation + P_PI / 2),
         .y = sinf(robot_orientation + P_PI / 2)}};

    // multiply global point by rotation matrix
    result.x = dot2D(rotationMatrix[0], relativePoint);
    result.y = dot2D(rotationMatrix[1], relativePoint);

    return result;
}