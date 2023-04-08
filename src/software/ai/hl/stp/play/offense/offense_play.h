#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't, while keeping some robots
 * to protect the defense area
 */
class OffensePlay : public Play
{
   public:
    OffensePlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    std::shared_ptr<ShootOrPassPlay> shoot_or_pass_play;
    std::shared_ptr<DefensePlay> defense_play;
};
