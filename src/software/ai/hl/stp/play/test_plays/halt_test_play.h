#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A test Play that halts 3 robots.
 *
 * The Play is never done
 *
 * This play is applicable when the ball's y coordinate is >= 0
 * This play's invariant holds while the ball is within the field
 */
class HaltTestPlay : public Play
{
   public:
    /**
     * Constructor for HaltTestPlay
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    HaltTestPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
};
