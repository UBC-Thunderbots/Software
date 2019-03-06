#include "ai/hl/stp/evaluation/robot.h"



bool Evaluation::robotOrientationWithinAngleThresholdOfTarget(const Point position,
                                                              const Angle orientation,
                                                              const Point target,
                                                              Angle threshold)
{
    // Calculate the target orientation, then calculate the difference between facing
    // orientation and target orientation. The difference in angle will be in the range of
    // [0, pi]. Return true if the difference is smaller than threshold, false otherwise
    Angle target_orientation = (target - position).orientation();
    Angle diff_orientation   = orientation.diff(target_orientation);
    return diff_orientation < threshold;
}
