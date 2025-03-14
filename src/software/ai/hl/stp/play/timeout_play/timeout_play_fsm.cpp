#include "software/ai/hl/stp/play/timeout_play/timeout_play_fsm.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"

static inline PriorityTacticVector makeTimeoutFormation(WorldPtr world)
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
                  [&](Point point) {
                      std::shared_ptr<MoveTactic> move_tactic =
                          std::make_shared<MoveTactic>();
                      move_tactic->updateControlParams(point, Angle::zero());
                      tactic.push_back(move_tactic);
                  });

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
