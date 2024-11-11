#include "software/ai/hl/stp/play/example/example_play_fsm.h"

ExamplePlayFSM::ExamplePlayFSM() : move_tactics(DIV_A_NUM_ROBOTS)
{
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(); });
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

    // set the Tactics this Play wants to run, in order of priority
    // If there are fewer robots in play, robots at the end of the list will not be
    // assigned
    TacticVector result = {};
    result.insert(result.end(), move_tactics.begin(), move_tactics.end());
    event.common.set_tactics({result});
}