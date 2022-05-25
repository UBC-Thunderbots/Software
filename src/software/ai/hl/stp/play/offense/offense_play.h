#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/crease_defense/crease_defense_play.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't, while keeping some robots
 * to protect the defense area
 */
class OffensePlay : public Play
{
   public:
    OffensePlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;

   private:
    std::shared_ptr<ShootOrPassPlay> shoot_or_pass_play;
    std::shared_ptr<CreaseDefensePlay> crease_defense_play;
};
