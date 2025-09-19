#include "software/ai/hl/stp/play/example/example_play_fsm.h"

ExamplePlayFSM::ExamplePlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<ExamplePlayFSM>(ai_config_ptr), move_tactics(DIV_A_NUM_ROBOTS)
{
    std::generate(move_tactics.begin(), move_tactics.end(), [ai_config_ptr]()
                  { return std::make_shared<MoveTactic>(ai_config_ptr); });
}

void ExamplePlayFSM::moveToPosition(const Update &event)
{
    // The angle between each robot spaced out in a circle around the ball
    Angle angle_between_robots = Angle::full() / static_cast<double>(move_tactics.size());

    for (size_t k = 0; k < move_tactics.size(); k++)
    {
        move_tactics[k]->updateControlParams(
            event.common.world_ptr->ball().position() +
                Vector::createFromAngle(angle_between_robots *
                                        static_cast<double>(k + 1)),
            (angle_between_robots * static_cast<double>(k + 1)) + Angle::half());
    }

    // Set the Tactics this Play wants to run, in order of priority.
    // If there are fewer robots in play, robots at the end of the list will not be
    // assigned
    TacticVector result = {};
    result.insert(result.end(), move_tactics.begin(), move_tactics.end());
    event.common.set_tactics({result});
}
