#include "ai/hl/stp/evaluation/player.h"

using namespace AI::HL::STP;





bool Evaluation::playerOrientationWithinAngleThresholdOfTarget(const Point position, const Angle orientation, const Point target,
                                                   Angle threshold) {

    Point pass_direction   = (target - position).norm();
    Point facing_direction = Point(1, 0).rotate(orientation);
    double direction_threshold = threshold.cos();

    return facing_direction.dot(pass_direction) > direction_threshold;
}
