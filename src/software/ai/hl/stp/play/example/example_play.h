#pragma once

#include "software/ai/hl/stp/play/example/example_play_fsm.h"
#include "software/ai/hl/stp/play/play_base.h"

/**
 * An example play that moves the robots in a circle around the ball
 */
class ExamplePlay : public PlayBase<ExamplePlayFSM>
{
   public:
    /**
     * Creates an example play
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    ExamplePlay(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
