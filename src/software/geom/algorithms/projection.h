#pragma once

#include <vector>

#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

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
std::vector<Segment> projectCirclesOntoSegment(const Segment &segment,
                                               const std::vector<Circle> &circles,
                                               const Point &origin);
