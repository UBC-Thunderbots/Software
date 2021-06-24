#include "software/ai/hl/stp/play/calibration_play.h"

#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/circle.h"
#include "software/util/design_patterns/generic_factory.h"

CalibrationPlay::CalibrationPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, false)
{
}

std::vector<CircleWithColor> CalibrationPlay::getCirclesWithColorToDraw()
{
    if (circle_to_draw.has_value())
    {
        return {
            std::make_pair<Circle, std::string>(Circle(circle_to_draw.value().origin(), circle_to_draw.value().radius()), "blue")
        };
    }
    return {};
}

bool CalibrationPlay::isApplicable(const World &world) const
{
    // This play is never applicable so it will never be chosen during gameplay
    // This play can be run for testing by using the Play override
    return false;
}

bool CalibrationPlay::invariantHolds(const World &world) const
{
    return true;
}

void CalibrationPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const World &world)
{
    LOG(INFO) << "Starting kick calibration";

    Point start_point = Point(0,0);

    Angle angle_to_face = Angle::fromDegrees(0);
    Angle angle_to_add = Angle::fromDegrees(45);

    for (int k = 0; k < 8; k++)
    {
        auto dribble_tactic = std::make_shared<DribbleTactic>();
        auto kick_tactic = std::make_shared<KickTactic>(false);

        dribble_tactic->updateControlParams(start_point, angle_to_face + k * angle_to_add, true);

        do 
        {
            yield({{dribble_tactic}});
        } while(!dribble_tactic->done());

        kick_tactic->updateControlParams(
                start_point, angle_to_face + k * angle_to_add, 2.0);
        do 
        {
            yield({{kick_tactic}});
        } while(!kick_tactic->done());
    }
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CalibrationPlay, PlayConfig> factory;
