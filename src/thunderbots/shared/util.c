#include "util.h"
#include "constants.h"

float dot2D(Vector2D first, Vector2D second)
{
    return first.x * second.x + first.y * second.y;
}

Vector2D toLocalCoords(Vector2D point, float orientationAngle)
{
    Vector2D result = {.x = 0.0f, .y = 0.0f};

    // clamp angle to [0, 2 * pi]
    orientationAngle = fmod(orientationAngle, 2 * P_PI);

    // the Robot's X-axis is along the orientation of the robot,
    // and the Y-axis is perpendicular to it

    // rotation matrix copy-pasted from physbot.c in firmware
    Vector2D rotationMatrix[2] = {
            {.x = cosf(orientationAngle), .y = sinf(orientationAngle)},
            {.x = cosf(orientationAngle + P_PI / 2), .y = sinf(orientationAngle + P_PI / 2)}
    };

    // multiply global point by rotation matrix
    result.x = dot2D(rotationMatrix[0], point);
    result.y = dot2D(rotationMatrix[1], point);

    return result;
}