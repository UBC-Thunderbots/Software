
#include "software/ai/hl/stp/play/halt_play/halt_play_fsm.h"

#include <algorithm>
#include <iterator>

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/world/game_state.h"

static PriorityTacticVector makeTimeoutFormation(WorldPtr world)
{
    TacticVector tactic = {};

    double half_field_length_x = world->field().xLength() / 2.0;
    double x_grain             = half_field_length_x / 4;

    double half_field_legnth_y = world->field().yLength() / 2.0;
    double y_grain             = half_field_legnth_y / 4;

    std::vector<Point> timeoout_formation_points = {Point(-x_grain, 0),
                                                    Point(-x_grain * 2, 0),
                                                    Point(-x_grain * 3, 0),
                                                    Point(-x_grain * 2, -y_grain),
                                                    Point(-x_grain * 2, -y_grain * 2),
                                                    Point(-x_grain * 2, -y_grain * 3)};

    // making a t like formation
    std::for_each(timeoout_formation_points.begin(), timeoout_formation_points.end(),
                  [&](Point point)
                  {
                      std::shared_ptr<MoveTactic> move_tactic =
                          std::make_shared<MoveTactic>();
                      move_tactic->updateControlParams(point, Angle::zero());
                      tactic.push_back(move_tactic);
                  });

    return {tactic};
}

HaltPlayFSM::HaltPlayFSM(TbotsProto::AiConfig ai_config) : halt_tactics({{}})
{
    std::generate_n(std::back_inserter(halt_tactics.front()), MAX_ROBOT_IDS_PER_SIDE,
                    []() { return std::make_shared<HaltTactic>(); });
}


void HaltPlayFSM::updateTimeout(const Update& event)
{
    event.common.set_tactics(makeTimeoutFormation(event.common.world_ptr));
}

void HaltPlayFSM::updateStop(const Update& event)
{
    event.common.set_tactics(halt_tactics);
}


bool HaltPlayFSM::shouldEnterHalt(const Update& update)
{
    RefereeCommand command = update.common.world_ptr->gameState().getRefereeCommand();
    return command == RefereeCommand::HALT;
}

bool HaltPlayFSM::shouldEnterTimeout(const Update& update)
{
    RefereeCommand command = update.common.world_ptr->gameState().getRefereeCommand();
    return command == RefereeCommand::TIMEOUT_US ||
           command == RefereeCommand::TIMEOUT_THEM;
}
