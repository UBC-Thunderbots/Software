#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <algorithm>

ShootOrPassPlayFSM::ShootOrPassPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      attacker_tactic(std::make_shared<AttackerTactic>(ai_config)),
      receiver_tactic(std::make_shared<ReceiverTactic>()),
      offensive_positioning_tactics(std::vector<std::shared_ptr<MoveTactic>>()),
      pass_generator(
          PassGenerator<EighteenZoneId>(std::make_shared<const EighteenZonePitchDivision>(
                                            Field::createSSLDivisionBField()),
                                        ai_config.passing_config())),
      pass_optimization_start_time(Timestamp::fromSeconds(0)),
      best_pass_and_score_so_far(
          PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0}),
      time_since_commit_stage_start(Duration::fromSeconds(0)),
      min_pass_score_threshold(0)
{
}

void ShootOrPassPlayFSM::updateOffensivePositioningTactics(
    std::vector<EighteenZoneId> ranked_zones, PassEvaluation<EighteenZoneId> pass_eval,
    unsigned int num_tactics)
{
    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    if (num_tactics != offensive_positioning_tactics.size())
    {
        offensive_positioning_tactics =
            std::vector<std::shared_ptr<MoveTactic>>(num_tactics);
        std::generate(offensive_positioning_tactics.begin(),
                      offensive_positioning_tactics.end(),
                      []() { return std::make_shared<MoveTactic>(); });
    }

    for (unsigned int i = 0; i < offensive_positioning_tactics.size(); i++)
    {
        auto pass1 = pass_eval.getBestPassInZones({ranked_zones[i]}).pass;

        offensive_positioning_tactics[i]->updateControlParams(
            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    }
}

void ShootOrPassPlayFSM::lookForPass(const Update& event)
{
    PriorityTacticVector ret_tactics = {{attacker_tactic}, {}};

    // only look for pass if there are more than 1 robots
    if (event.common.num_tactics > 1)
    {
        auto pitch_division =
            std::make_shared<const EighteenZonePitchDivision>(event.common.world.field());

        auto pass_eval    = pass_generator.generatePassEvaluation(event.common.world);
        auto ranked_zones = pass_eval.rankZonesForReceiving(
            event.common.world, event.common.world.ball().position());

        best_pass_and_score_so_far = pass_eval.getBestPassOnField();


        // Wait for a good pass by starting out only looking for "perfect" passes
        // (with a score of 1) and decreasing this threshold over time
        // This boolean indicates if we're ready to perform a pass
        double abs_min_pass_score =
            ai_config.shoot_or_pass_play_config().abs_min_pass_score();
        double pass_score_ramp_down_duration =
            ai_config.shoot_or_pass_play_config().pass_score_ramp_down_duration();
        pass_eval = pass_generator.generatePassEvaluation(event.common.world);
        best_pass_and_score_so_far = pass_eval.getBestPassOnField();

        // update the best pass in the attacker tactic
        attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, false);

        // If we've assigned a robot as the passer in the PassGenerator, we
        // lower our threshold based on how long the PassGenerator has been
        // running since we set it
        time_since_commit_stage_start =
            event.common.world.getMostRecentTimestamp() - pass_optimization_start_time;
        min_pass_score_threshold =
            1 - std::min(time_since_commit_stage_start.toSeconds() /
                             pass_score_ramp_down_duration,
                         1.0 - abs_min_pass_score);
        updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                          event.common.num_tactics - 1);

        ret_tactics[1].insert(ret_tactics[1].end(), offensive_positioning_tactics.begin(),
                              offensive_positioning_tactics.end());
    }
    event.common.set_tactics(ret_tactics);
}

void ShootOrPassPlayFSM::startLookingForPass(const Update& event)
{
    attacker_tactic              = std::make_shared<AttackerTactic>(ai_config);
    receiver_tactic              = std::make_shared<ReceiverTactic>();
    pass_optimization_start_time = event.common.world.getMostRecentTimestamp();
    lookForPass(event);
}

void ShootOrPassPlayFSM::takePass(const Update& event)
{
    auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);

    auto ranked_zones = pass_eval.rankZonesForReceiving(
        event.common.world, best_pass_and_score_so_far.pass.receiverPoint());

    // if we make it here then we have committed to the pass
    attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, true);
    receiver_tactic->updateControlParams(best_pass_and_score_so_far.pass);
    event.common.set_inter_play_communication_fun(
        InterPlayCommunication{.last_committed_pass = best_pass_and_score_so_far});

    if (!attacker_tactic->done())
    {
        PriorityTacticVector ret_tactics = {{attacker_tactic, receiver_tactic}, {}};

        if (event.common.num_tactics > 2)
        {
            updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                              event.common.num_tactics - 2);
            ret_tactics[1].insert(ret_tactics[1].end(),
                                  offensive_positioning_tactics.begin(),
                                  offensive_positioning_tactics.end());
        }

        event.common.set_tactics(ret_tactics);
    }
    else
    {
        PriorityTacticVector ret_tactics = {{receiver_tactic}, {}};
        if (event.common.num_tactics > 1)
        {
            updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                              event.common.num_tactics - 1);
            ret_tactics[1].insert(ret_tactics[1].end(),
                                  offensive_positioning_tactics.begin(),
                                  offensive_positioning_tactics.end());
        }

        event.common.set_tactics(ret_tactics);
    }
}

bool ShootOrPassPlayFSM::passFound(const Update& event)
{
    return best_pass_and_score_so_far.rating > min_pass_score_threshold;
}

bool ShootOrPassPlayFSM::shouldAbortPass(const Update& event)
{
    // TODO (#2384): implement this
    return false;
}

bool ShootOrPassPlayFSM::passCompleted(const Update& event)
{
    return attacker_tactic->done() && receiver_tactic->done();
}

bool ShootOrPassPlayFSM::tookShot(const Update& event)
{
    // TODO (#2384): implement this
    return false;
}
