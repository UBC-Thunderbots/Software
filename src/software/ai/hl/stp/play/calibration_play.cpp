#include "software/ai/hl/stp/play/calibration_play.h"

#include "software/ai/passing/pass_generator.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
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
        auto pass = Pass(
                world.ball().position(),
                Point(play_config->getPassingConfig()->getReceiverXPosition()->value(),
                    play_config->getPassingConfig()->getReceiverYPosition()->value()),
                play_config->getPassingConfig()->getMaxPassSpeedMPerS()->value());
    auto attacker_tactic = std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());
    auto receiver_tactic = std::make_shared<ReceiverTactic>(pass);

    auto stop_tactic1 = std::make_shared<MoveTactic>(true);
    auto stop_tactic2 = std::make_shared<MoveTactic>(true);
    auto stop_tactic3 = std::make_shared<MoveTactic>(true);
    auto stop_tactic4 = std::make_shared<MoveTactic>(true);
    auto stop_tactic5 = std::make_shared<MoveTactic>(true);
    auto stop_tactic6 = std::make_shared<MoveTactic>(true);
    auto stop_tactic7 = std::make_shared<MoveTactic>(true);
    auto stop_tactic8 = std::make_shared<MoveTactic>(true);
    auto stop_tactic9 = std::make_shared<MoveTactic>(true);


    stop_tactic1->updateControlParams(Point(0, -1), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic2->updateControlParams(Point(0, -2), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic3->updateControlParams(Point(0, -3), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic4->updateControlParams(Point(0, 1), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic5->updateControlParams(Point(0, 2), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic6->updateControlParams(Point(0, 3), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic7->updateControlParams(Point(0, 4), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic8->updateControlParams(Point(0, -4), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    stop_tactic9->updateControlParams(Point(0, 0), Angle::fromDegrees(0),
            0.0,
            MaxAllowedSpeedMode::TIPTOE);
    do
    {
        pass = Pass(
                world.ball().position(),
                Point(play_config->getPassingConfig()->getReceiverXPosition()->value(),
                    play_config->getPassingConfig()->getReceiverYPosition()->value()),
                play_config->getPassingConfig()->getMaxPassSpeedMPerS()->value());

        attacker_tactic->updateControlParams(
                pass, play_config->getPassingConfig()->getExecutePass()->value());
        receiver_tactic->updateControlParams(pass);

        yield({{attacker_tactic, receiver_tactic}});

    } while (!attacker_tactic->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CalibrationPlay, PlayConfig> factory;
