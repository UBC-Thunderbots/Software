#pragma once

#include "software/ai/evaluation/enemy_threat.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/world/field.h"

// This struct stores the concept of a defense lane, which describes
// a potential passing or shooting lane an enemy robot could shoot the
// ball along
struct DefenseLane
{
    // The segment representing the lane on the field, starting from
    // the position of the enemy robot and ending at the intended point
    // where the ball will be received or enter the net
    Segment lane;

    // The "expected threat" of the lane, which scores the dangerousness
    // of the lane relative to other lanes (i.e. how likely a pass or shot 
    // along the lane will eventually result in the enemy team scoring)
    unsigned int expected_threat;
};

// This struct stores the concept of a defense position, which describes
// the location on the field that a defender could move towards to block 
// a potential enemy pass or shot on net
struct DefensePosition
{
    // The location of the defense position on the field
    //
    // There is a special case where if the defense position represents
    // the location of a potential crease defender, then this member 
    // represents the origin of the enemy threat
    Point position;

    // The "effectiveness" of the position, which scores its ability to 
    // block high-danger enemy scoring chances or passes
    unsigned int effectiveness;

    // Whether the defense position represents the location of a
    // potential crease defender
    bool is_crease_defense;
};

struct CreaseDefenderInfo

static constexpr unsigned int SHOOTING_LANE_MULTIPLIER = 4;

/**
 * Determines all possible defense positions where a defender could be placed
 * on the field and returns them in order of decreasing "effectiveness" -- a
 * score which quantifies a position's ability to block high-danger enemy  
 * scoring chances or passes.
 * 
 * @param threats all enemy threats to consider in determining defense positions
 * @param field the field being played on
 * 
 * @return a list of all possible defense positions in order of decreasing
 * effectiveness
 */
std::vector<DefensePosition> getAllDefensePositions(const std::vector<EnemyThreat> &threats,
                                                    const Field &field);