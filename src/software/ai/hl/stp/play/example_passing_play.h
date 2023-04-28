#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/passing/pass_generator.hpp"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

class ExamplePassingPlay : public Play
{
    public:
        explicit ExamplePassingPlay(TbotsProto::AiConfig config);

        void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    private:
        TbotsProto::AiConfig ai_config;
};
