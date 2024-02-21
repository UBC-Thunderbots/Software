#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/world/field.h"

// Indicates the type of defender for a DefenderAssignment
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
    double coverage_rating;

    // Equality operator for unit testing.
    bool operator==(const DefenderAssignment &other) const
    {
        return this->type == other.type && this->target == other.target &&
               this->coverage_rating == other.coverage_rating;
    }
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
    double threat_rating;

    // Equality operator for unit testing.
    bool operator==(const ShootingLane &other) const
    {
        return this->lane == other.lane && this->threat_rating == other.threat_rating;
    }
};

// A goal lane describes a shooting lane from an enemy robot to the goal
struct GoalLane : ShootingLane
{
    // The angle in radians from the enemy robot to the goal that the goal lane describes
    Angle angle_to_goal;

    // Equality operator for unit testing.
    bool operator==(const GoalLane &other) const
    {
        return this->lane == other.lane && this->threat_rating == other.threat_rating &&
               this->angle_to_goal == other.angle_to_goal;
    }
};

/**
 * Determines all possible defender assignments where a defender could be placed
 * on the field and returns them in order of decreasing coverage rating
 *
 * @param threats all enemy threats to consider in determining defender assignments,
 * in order of decreasing threat
 * @param field the field being played on
 * @param ball the ball
 * @param config the DefenderAssignmentConfig used for tuning assignments
 *
 * @return a list of all possible defender assignments in order of decreasing
 * coverage rating
 */
std::vector<DefenderAssignment> getAllDefenderAssignments(
    const std::vector<EnemyThreat> &threats, const Field &field, const Ball &ball,
    const TbotsProto::DefensePlayConfig::DefenderAssignmentConfig &config);

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
 * @param min_distance the minimum distance between two threats for them to be
 * considered non-similar
 * @param min_angle the minimum difference between two threats in angle to the
 * primary threat for them to be considered non-similar

 * @return a copy of the threats list with similarly positioned threats removed
 */
std::vector<EnemyThreat> filterOutSimilarThreats(const std::vector<EnemyThreat> &threats,
                                                 double min_distance,
                                                 const Angle &min_angle);

/**
 * Groups together goal lanes that are densely clustered (i.e. have similar angles
 * to the goal).
 *
 * @param goal_lanes the goal lanes to group
 * @param density_threshold the max percent difference between two goal lanes' angles
 * to goal for them to be considered a dense lane cluster
 *
 * @return a list of lists which represent groupings of densely clustered goal lanes
 */
std::vector<std::vector<GoalLane>> groupGoalLanesByDensity(
    std::vector<GoalLane> &goal_lanes, double density_threshold);
