#include "ai/hl/stp/play/example_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"

const std::string ExamplePlay::name = "Example Play";

std::string ExamplePlay::getName() const
{
    return ExamplePlay::name;
}

bool ExamplePlay::isApplicable(const World &world) const
{
    return true;
}

bool ExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    Timestamp pass_start_time = world.ball().lastUpdateTimestamp() + Duration::fromSeconds(5);

    AI::Passing::Pass pass(world.ball().position(), {0.5, -0.6}, 4.5, pass_start_time);
    auto passer = std::make_shared<PasserTactic>(pass, world.ball(), false);
    auto receiver = std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(), false);

//    AI::Passing::PassGenerator pass_generator(0.001, world);
//
//    do {
//        pass_generator.setWorld(world);
//        pass_generator.setPasserPoint({2.7,-1.8});
//        auto pass_opt = pass_generator.getBestPassSoFar();
////        if (pass_opt){
////            std::cout << *pass_opt << std::endl;
////        }
//    } while(true);
//    pass = *pass_generator.getBestPassSoFar();
//
//    do {
//        pass_generator.setWorld(world);
//        pass_generator.setPasserPoint(world.ball().position());
//    } while(!pass_generator.getBestPassSoFar());
//    pass = *pass_generator.getBestPassSoFar();

//    std::optional<AI::Passing::Pass> best_pass;
//    while(!best_pass){
//        pass_generator.setWorld(world);
//        pass_generator.setPasserPoint(world.ball().position());
//        best_pass = pass_generator.getBestPassSoFar();
//        std::cout << "NO BEST PASS" << std::endl;
//    }

    do
    {

            passer->updateParams(pass, world.ball());
            receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());
            yield({passer, receiver});

//        pass = AI::Passing::Pass(world.ball().position(), {0.5, -0.2}, 5, pass_start_time);
//        passer->updateParams(pass, world.ball().lastUpdateTimestamp());
//        receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());
//        pass_generator.setWorld(world);
//        pass_generator.setPasserPoint(world.ball().position());
//        auto best_pass = pass_generator.getBestPassSoFar();
//        if (best_pass){
//            passer->updateParams(*best_pass, world.ball());
//            receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), *best_pass, world.ball());
//            yield({passer, receiver});
//        } else {
//            yield({});
//        }
//        pass_generator.setWorld(world);
//        pass_generator.setPasserPoint(world.ball().position());
//        auto best_pass_opt  = pass_generator.getBestPassSoFar();
//        if (best_pass_opt){
////            pass = AI::Passing::Pass(world.ball().position(), {1, -0.0}, 4, pass_start_time);
//            pass = *best_pass_opt;
//            passer->updateParams(pass, world.ball().lastUpdateTimestamp());
//            receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());
//            yield({passer, receiver});
//        } else {
//            yield({});
//        }
//        yield({passer, receiver});
//        yield({});

    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<ExamplePlay> factory;
