#pragma once

#include <memory>
#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't, while keeping some robots
 * to protect the defense area
 */
class OffensePlay : public Play
{
   public:
    OffensePlay(const TbotsProto::AiConfig config, std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void reset() override;
    void updateTactics(const PlayUpdate &play_update) override;

   private:
    std::unique_ptr<FSM<OffensePlayFSM>> fsm;
    OffensePlayFSM::ControlParams control_params;
};
