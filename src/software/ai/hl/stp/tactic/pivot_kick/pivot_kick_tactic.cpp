#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_tactic.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/logger/logger.h"

PivotKickTactic::PivotKickTactic()
    : Tactic(false, {RobotCapability::Move, RobotCapability::Kick, RobotCapability::Chip,
                     RobotCapability::Dribble}),
      fsm(DribbleFSM()),
      control_params(PivotKickFSM::ControlParams())
{
}

double PivotKickTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    {
        // the closer the robot is to a ball, the cheaper it is to perform the kick
        double cost = (robot.position() - world.ball().position()).length() /
                      world.field().totalXLength();

        return std::clamp<double>(cost, 0, 1);
    }
}

void PivotKickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void PivotKickTactic::updateControlParams(const Point &kick_origin,
                                          const Angle &kick_direction,
                                          AutoChipOrKick auto_chip_or_kick)
{
    control_params.kick_origin       = kick_origin;
    control_params.kick_direction    = kick_direction;
    control_params.auto_chip_or_kick = auto_chip_or_kick;
}

bool PivotKickTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void PivotKickTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(PivotKickFSM::Update(control_params, tactic_update));
}
