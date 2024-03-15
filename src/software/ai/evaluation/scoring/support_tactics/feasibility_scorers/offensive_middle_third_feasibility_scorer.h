#pragma once

#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorer.h"

/**
 * Scores SupportTacticCandidates based on their feasibility for the
 * OffensiveMiddleThirdPlay and the current gameplay scenario.
 *
 * The most feasible tactics for OffensiveMiddleThirdPlay are ones that
 * prioritize gaining entry into the enemy third and setting up scoring opportunities.
 */
class OffensiveMiddleThirdFeasibilityScorer : public FeasibilityScorer
{
}