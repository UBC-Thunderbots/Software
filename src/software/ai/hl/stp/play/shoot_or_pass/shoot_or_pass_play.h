#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"
#include "software/ai/hl/stp/strategy/strategy.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't.
 */
class ShootOrPassPlay : public Play
{
   public:
    ShootOrPassPlay(const TbotsProto::AiConfig &config,
                    std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void reset() override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    std::unique_ptr<FSM<ShootOrPassPlayFSM>> fsm;
    ShootOrPassPlayFSM::ControlParams control_params;
};
