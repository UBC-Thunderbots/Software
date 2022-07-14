#include "software/ai/hl/stp/play/example_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

ExamplePlay::ExamplePlay(TbotsProto::AiConfig config) : Play(config, false) {}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    std::vector<std::shared_ptr<MoveTactic>> move_tactics(DIV_A_NUM_ROBOTS);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(); });

    // Continue to loop to demonstrate the example play indefinitely
    do
    {
        for (size_t k = 0; k < move_tactics.size(); k++)
        {
            move_tactics[k]->updateControlParams(world.ball().position() + Vector(1,0), Angle::zero(),
                                                 0,TbotsProto::DribblerMode::OFF,

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
static TGenericFactory<std::string, Play, ExamplePlay, TbotsProto::AiConfig> factory;
