#pragma once

#include <optional>

#include "software/ai/evaluation/shot.h"
#include "software/ai/passing/pass.h"

/**
 * Represents the state of a Skill.
 */
struct SkillState
{
    // Whether the Skill has committed to a pass
    bool pass_committed = false;

    // The pass that the Skill is planning to take
    std::optional<Pass> pass = std::nullopt;

    // The shot on goal that the Skill is planning to take
    std::optional<Shot> shot = std::nullopt;

    // The point on the field that the Skill is planning to chip the ball to
    std::optional<Point> chip_target = std::nullopt;
};
