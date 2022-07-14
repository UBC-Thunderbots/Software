#include "software/ai/hl/stp/play/move_to_dest_and_orient_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

MoveToDestAndOrientPlay::MoveToDestAndOrientPlay(TbotsProto::AiConfig config)
    : Play(config, false)
{
}

void MoveToDestAndOrientPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                             const World &world)
{
    Angle target_orientation = Angle::quarter() * (std::rand() % 4);
    LOG(INFO) << "Selected target orientation is: " << target_orientation.toDegrees()
              << " degrees";
    std::vector<std::shared_ptr<MoveTactic>> move_tactics(DIV_A_NUM_ROBOTS);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(); });

    // Continue to loop to demonstrate the example play indefinitely
    do
    {
        for (size_t k = 0; k < move_tactics.size(); k++)
        {
            move_tactics[k]->updateControlParams(
                world.ball().position() +
                    Vector(1 + (static_cast<double>(k) / 10) * 2, 0),
                target_orientation, 0, TbotsProto::DribblerMode::OFF,

                TbotsProto::BallCollisionType::ALLOW);
        }

        // yield the Tactics this Play wants to run, in order of priority
        // If there are fewer robots in play, robots at the end of the list will not be
        // assigned
        TacticVector result = {};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});

    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, MoveToDestAndOrientPlay, TbotsProto::AiConfig>
    factory;
