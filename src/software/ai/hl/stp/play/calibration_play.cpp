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
    /**
     * Chip Distance Calibration
     *
     * We need to be able to support multiple virtual and physical robots that
     * have different chipping properties. We also need to run in environments
     * (virtual and physical) with varying phsyics
     *
     * This calibration sequence is as follows:
     *
     * 1. move the ball to the -2, 0
     * 2. chip to a certain distance
     * 3. check expected distance vs actual distance
     * 4. log the difference between expected vs actual
     * 5. repeat 1 - 4 with another chip distance
     *
     * After this test runs ~5 times, we output expected and the average chip distance
     * and leave it up the user of the calibration play to update the respective
     * constants.
     */
    LOG(INFO) << "Starting movement calibration";

    Point start_point = Point(-2,0);
    auto move_tactic = std::make_shared<MoveTactic>(false);

    for (int k = 0; k < 5; k++)
    {
        move_tactic->updateControlParams(start_point, Angle::fromDegrees(0), 0.0);
        this->circle_to_draw = Circle(start_point, 0.10);
        start_point += Vector(1.0, 0);
        do 
        {
            yield({{move_tactic}});
        } while(!move_tactic->done());
    }

    LOG(INFO) << "Movement calibration complete, starting chip calibration";

    const Point chip_start         = Point(-4, 3);
    const Angle angle_to_face      = Angle::fromDegrees(-45);
    const double max_chip_distance = 10.0;

    for (double chip_distance = ROBOT_MIN_CHIP_CLEAR_DISTANCE_METERS * 2; chip_distance < max_chip_distance;
         chip_distance += 0.5)
    {
        auto dribble_tactic = std::make_shared<DribbleTactic>();
        auto chip_tactic    = std::make_shared<KickTactic>(false);

        dribble_tactic->updateControlParams(chip_start, angle_to_face, false);

        do 
        {
            yield({{dribble_tactic}});
        } while(!dribble_tactic->done());

        chip_tactic->updateControlParams(chip_start, angle_to_face, chip_distance);

        do
        {
            yield({{chip_tactic}});
        } while (!chip_tactic->done());

        // NOTE: This isn't entirely accurate because it doesn't account for acceleration
        LOG(DEBUG) << "Ball chipped with initial velocity: "
                   << world.ball().velocity().length();

        LOG(DEBUG) << "Expected : "
                   << Vector::createFromAngle(Angle::fromDegrees(45))
                          .normalize(chip_distance)
                          .length();

        // wait for ball to stop rolling
        do
        {
            yield({{chip_tactic}});
        } while(world.ball().velocity().length() <= 0.01);
    }
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CalibrationPlay, PlayConfig> factory;
