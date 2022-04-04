#pragma once

#include "shared/constants.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/circle.h"
#include "software/geom/segment.h"
#include "software/world/world.h"


std::vector<Pass> findDirectPasses(const Robot& robot, const Team& friendly_team,
                                   const Team& enemy_team);
std::vector<Pass> findIndirectPasses(const Robot& robot, const Team& friendly_team,
                                     const Team& enemy_team);
