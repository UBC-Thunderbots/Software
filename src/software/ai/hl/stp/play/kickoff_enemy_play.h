#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the enemies kick off,
 * prioritizes defending the net and shadowing the robot
 * that is nearest to the ball. Any remaining bots will block
 * some odd angles to the net.
 */
class KickoffEnemyPlay : public Play
{
   public:
    KickoffEnemyPlay(const TbotsProto::AiConfig &config,
                     std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
