
#include "detect_successful_kick.h"

bool successfulKickDetected(const Ball& ball, const Vector& expected_kick_direction)
{
    double min_kick_speed       = 0.5;
    double max_angle_difference = 20;

    Angle expected_kick_angle = expected_kick_direction.orientation();
    Angle kick_orientation_difference =
        ball.velocity().orientation().minDiff(expected_kick_angle);

    return (kick_orientation_difference.abs() <
                Angle::fromDegrees(max_angle_difference) &&
            ball.velocity().length() > min_kick_speed);
}
