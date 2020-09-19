#include "detect_successful_kick.h"

bool hasBallBeenKicked(const Ball& ball, const Angle& expected_kick_direction,
                       double min_kick_speed)
{
    static constexpr double MAX_ANGLE_DIFFERENCE = 20;

    Angle kick_orientation_difference =
        ball.velocity().orientation().minDiff(expected_kick_direction);

    return (kick_orientation_difference.abs() <
                Angle::fromDegrees(MAX_ANGLE_DIFFERENCE) &&
            ball.velocity().length() > min_kick_speed);
}
