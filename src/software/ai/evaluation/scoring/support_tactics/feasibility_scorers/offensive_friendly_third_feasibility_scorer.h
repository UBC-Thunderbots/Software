#pragma once

#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorer.h"

/**
 * Scores SupportTacticCandidates based on their feasibility for the
 * OffensiveFriendlyThirdPlay and the current gameplay scenario.
 *
 * The most feasible tactics for OffensiveFriendlyThirdPlay are ones that
 * prioritize moving the ball up the field and creating passing opportunities.
 */
class OffensiveFriendlyThirdFeasibilityScorer : public FeasibilityScorer
{
};