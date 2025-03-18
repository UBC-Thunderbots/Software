#include "software/ai/hl/stp/play/timeout_play/timeout_play_fsm.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"

PriorityTacticVector TimeoutPlayFSM::makeTimeoutFormation(WorldPtr world)
{
    TacticVector tactic = {};

    double half_field_length_x = world->field().xLength() / 2.0;
    double formation_spacing_x = half_field_length_x / 4;

    double half_field_length_y = world->field().yLength() / 2.0;
    double formation_spacing_y = half_field_length_y / 4;

    std::vector<Point> formation_points = {
        Point(-formation_spacing_x, 0),
        Point(-formation_spacing_x * 2, 0),
        Point(-formation_spacing_x * 3, 0),
        Point(-formation_spacing_x * 2, -formation_spacing_y),
        Point(-formation_spacing_x * 2, -formation_spacing_y * 2),
        Point(-formation_spacing_x * 2, -formation_spacing_y * 3)};

    for (const Point& point : formation_points)
    {
        std::shared_ptr<MoveTactic> move_tactic = std::make_shared<MoveTactic>();
        move_tactic->updateControlParams(point, Angle::zero());
        tactic.push_back(move_tactic);
    }

    return {tactic};
}

void TimeoutPlayFSM::updateTimeout(const Update& event)
{
    event.common.set_tactics(makeTimeoutFormation(event.common.world_ptr));
}

TimeoutPlayFSM::TimeoutPlayFSM(TbotsProto::AiConfig ai_config)
{
    // timeout here
}
