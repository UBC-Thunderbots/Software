#include "software/ai/hl/stp/play/example_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"

CalibrationPlay::CalibrationPlay(std::shared_ptr<const PlayConfig> config) : Play(config, false)
{
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

void CalibrationPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig(), stop_mode);

    /**
     * Chip Distance Calibration
     *
     * We need to be able to support multiple virtual and physical robots that
     * have different chipping properties and also different environments 
     * (virtual and physical) that have different physics.
     *
     * This calibration sequence is as follows:
     *
     * 1. move the ball to the center of the field
     * 2. chip to a certain distance
     * 3. check expected distance vs actual distance
     * 4. log the difference between expeced vs actual
     * 4. repeat 1 - 4 with another chip distance
     *
     * After this test runs ~10 times, we output an averge off all the differences,
     * and leave it up the user of the calibration play to update the respective
     * constants.
     */
    auto dribble_tactic = std::make_shared<DribbleTactic>();

    for (double chip_distance = 0.5; chip_distance < 5.0; chip_distance += 0.5)
    {
        dribble_tactic->updateControlParams(Point(0,0), Angle::fromDegrees(0), true);

        while (!dribble_tactic->done())
        {
            yield({{goalie_tactic, dribble_tactic}});
        }
    }
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CalibrationPlay, PlayConfig> factory;
