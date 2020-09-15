
#include "detect_successful_kick.h"

bool successfulKickDetected(const Ball& ball, const Angle& kick_expected_direction){

    double min_kick_speed = 0.5;
    double max_angle_difference = 20;

    Angle kick_orientation_difference =
            ball.velocity().orientation().minDiff(kick_expected_direction);
    return (kick_orientation_difference.abs() < Angle::fromDegrees(max_angle_difference) && ball.velocity().length() > min_kick_speed)

}
