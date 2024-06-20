#pragma once

#include <optional>

#include "software/ai/passing/pass.h"
#include "software/ai/evaluation/shot.h"

struct SkillState
{
    bool pass_committed              = false;
    std::optional<Pass> pass         = std::nullopt;
    std::optional<Shot> shot         = std::nullopt;
    std::optional<Point> chip_target = std::nullopt;
};
