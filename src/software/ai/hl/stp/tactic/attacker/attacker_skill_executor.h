#pragma once

#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/passing/pass.h"
#include "software/util/make_enum/make_enum.hpp"

// clang-format off
MAKE_ENUM(AttackerSkill,
          SHOOT,
          PASS
          )
// clang-format on

class AttackerSkillExecutor
{
   public:
    struct ControlParams
    {
        std::optional<Pass> pass = std::nullopt;
    };

    void reset(AttackerSkill skill, const TbotsProto::AiConfig &ai_config);

    void updatePrimitive(const TacticUpdate &tactic_update,
                         const ControlParams &control_params);

    bool done() const;

   private:
    std::optional<AttackerSkill> current_skill_;

    std::unique_ptr<FSM<PivotKickFSM>> pivot_kick_fsm_;
    std::unique_ptr<FSM<DribbleFSM>> dribble_fsm_;
};
