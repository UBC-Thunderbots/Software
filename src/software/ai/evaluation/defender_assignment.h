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
