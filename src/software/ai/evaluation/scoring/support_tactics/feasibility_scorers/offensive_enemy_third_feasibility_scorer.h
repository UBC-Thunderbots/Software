#pragma once

#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorer.h"

/**
 * Scores SupportTacticCandidates based on their feasibility for the
 * OffensiveEnemyThirdPlay and the current gameplay scenario.
 *
 * The most feasible tactics for OffensiveEnemyThirdPlay are ones that
 * prioritize scoring and creating offensive pressure in the enemy zone.
 */
class OffensiveEnemyThirdFeasibilityScorer : public FeasibilityScorer
{
};