#pragma once

#include "software/ai/evaluation/enemy_threat.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/world/field.h"

enum DefenderAssignmentType
{
    CREASE_DEFENDER,
    PASS_DEFENDER
};

// This struct stores the concept of a defender assignment, which describes
// the location on the field that a defender could target to block a potential
// enemy pass or shot on net
struct DefenderAssignment
{
    // The type of defender for this assignment
    DefenderAssignmentType type;

    // The location on the field that the defender should target
    Point target;

    // The coverage rating of the assignment, which scores the defender's ability
    // to block high-danger enemy scoring chances or passes
    unsigned int coverage_rating;
};

// This struct stores the concept of a shooting lane, which describes
// a potential passing or shooting lane an enemy robot could shoot the
// ball along
struct ShootingLane
{
    // The segment representing the lane on the field, starting from
    // the position of the enemy robot and ending at the intended point
    // where the ball will be received or enter the net
    Segment lane;

    // The threat rating of the lane, which scores the dangerousness
    // of the lane relative to other lanes (i.e. how likely a pass or shot
    // along the lane will eventually result in the enemy team scoring)
    unsigned int threat_rating;
};

// The minimum distance between two threats for them to be considered non-similar
static constexpr double MIN_DISTANCE_BETWEEN_THREATS = 0.25;

// The minimum difference between two threats in angle to the primary threat for
// them to be considered non-similar
static constexpr Angle MIN_ANGLE_BETWEEN_THREATS = Angle::fromDegrees(10);

/**
 * Determines all possible defender assignments where a defender could be placed
 * on the field and returns them in order of decreasing coverage rating
 *
 * @param threats all enemy threats to consider in determining defender assignments,
 * in order of decreasing threat
 * @param field the field being played on
 * @param ball the ball
 *
 * @return a list of all possible defender assignments in order of decreasing
 * coverage rating
 */
std::vector<DefenderAssignment> getAllDefenderAssignments(
    const std::vector<EnemyThreat> &threats, const Field &field, const Ball &ball);

/**
 * Filters out enemy threats with similar positioning/angle to the primary threat
 * (i.e. the first threat in the list). Of the similar threats, only the closest
 * threat to the primary threat is returned in the filtered list.
 *
 * Example scenario with primary threat as X and other threats numbered:
 * ┌───────────────────┐
 * │       1           │
 * │        2          │
 * │                   │
 * │            X 3    │
 * │  4                │
 * └───────────────────┘
 * In this scenario, threats 1 and 3 would be filtered out. Threat 1 has a similar
 * angle to the primary threat as threat 2 (but threat 2 is closer). Threat 3 is too
 * close in proximity to the primary threat, so it is filtered out.
 *
 * @param threats all enemy threats in order of decreasing threat
 *
 * @return a copy of the threats list with similarly positioned threats removed
 */
std::vector<EnemyThreat> filterOutSimilarThreats(const std::vector<EnemyThreat> &threats);
