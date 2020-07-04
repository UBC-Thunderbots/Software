#pragma once

#include <vector>

#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/segment.h"

/**
 * Function projects Circle objects onto a Segment with respect to an origin Point
 *
 *               projected Segment
 * *______X---------------------------X___________________*  <-- reference Segment
 *          .                        .
 *           .                      .
 *            .                    .
 *             .                  .
 *              .                .
 *               .              .
 *                .   Circle   .
 *                 .  /----\  .
 *                  . |    | .
 *                   .\----/.
 *                    .    .
 *                     .  .
 *                      ..
 *                       X
 *                 Reference Origin
 *
 * @param segment : The reference Segment to project onto
 * @param circles : The vector of circles to project onto the reference Segment
 * @param origin : The Point origin to calculate the projection of the Circles from
 *
 * @return vector<Segment> : All of the projections of the Circles onto the reference
 * Segment
 */
std::vector<Segment> projectCirclesOntoSegment(Segment segment,
                                               std::vector<Circle> circles, Point origin);
