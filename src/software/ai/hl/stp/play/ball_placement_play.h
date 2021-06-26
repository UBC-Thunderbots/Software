#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A Play that performs ball placement, i.e. placing the ball in a defined location
 * determined by the referee. This is used to obey the referee "Ball Placement Us" command
 */
class BallPlacementPlay : public Play
{
   public:
    BallPlacementPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    TacticVector circleCenter(std::vector<std::shared_ptr<MoveTactic>> move_tactics, const Point& rotating_point, Angle angle, TacticVector current);
};
