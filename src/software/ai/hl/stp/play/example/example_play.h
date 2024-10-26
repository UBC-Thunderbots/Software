#pragma once

#include "software/ai/hl/stp/play/example/example_play_fsm.h"

/**
 * An example play that moves the robots in a circle around the ball
 */
class ExamplePlay : public Play
{
public:
    /**
     * Creates an example play
     *
     * @param ai_config the play config for this play
     */
    ExamplePlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

private:
    FSM<ExamplePlayFSM> fsm;
    ExamplePlayFSM::ControlParams control_params;
};
