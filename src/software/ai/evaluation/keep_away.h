#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/passing/pass.h"
#include "software/world/world.h"


/**
 * Find a point relatively close to the ball, that we can dribble it toward to maximize
 * the quality of a pass to the receiver point in the best pass so far, while also
 * trying to get far away from enemy robots.
 *
 * @param world the world
 * @param best_pass_so_far the best pass so far
 * @return a Point with the qualities described above.
 */
Point findKeepAwayTargetPoint(const World& world, const Pass& best_pass_so_far,
                              const TbotsProto::PassingConfig& passing_config);
