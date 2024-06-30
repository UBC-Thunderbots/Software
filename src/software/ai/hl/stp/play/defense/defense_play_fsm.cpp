#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include "software/ai/evaluation/defender_assignment.h"
#include "software/ai/evaluation/enemy_threat.h"

DefensePlayFSM::DefensePlayFSM(TbotsProto::AiConfig ai_config)
    : DefensePlayFSMBase::DefensePlayFSMBase(ai_config)
{
}

void DefensePlayFSM::defendAgainstThreats(const Update& event)
{
    auto enemy_threats = getAllEnemyThreats(
        event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
        event.common.world_ptr->enemyTeam(), event.common.world_ptr->ball(), false);

    auto assignments = getAllDefenderAssignments(
        enemy_threats, event.common.world_ptr->field(), event.common.world_ptr->ball(),
        ai_config.defense_play_config().defender_assignment_config());

    if (assignments.empty())
    {
        return;
    }

    // Choose which defender assignments to assign defenders to based on number
    // of tactics available to set
    std::vector<DefenderAssignment> crease_defender_assignments;
    std::vector<DefenderAssignment> pass_defender_assignments;
    for (unsigned int i = 0; i < event.common.num_tactics; i++)
    {
        DefenderAssignment defender_assignment;
        if (i < assignments.size())
        {
            defender_assignment = assignments.at(i);
        }
        else
        {
            // If we have more tactics to set than determined defender assignments,
            // assign remaining defenders to the defender assignment with the
            // highest coverage rating
            defender_assignment = assignments.front();
        }

        if (defender_assignment.type == CREASE_DEFENDER)
        {
            crease_defender_assignments.emplace_back(defender_assignment);

            // If we have at least two available defenders, two defenders should
            // be assigned to the highest scoring crease defender assignment to better
            // block the shot cone of the most threatening enemy
            if (i == 0 && event.common.num_tactics >= 2)
            {
                crease_defender_assignments.emplace_back(defender_assignment);
                i++;
            }
        }
        else
        {
            pass_defender_assignments.emplace_back(defender_assignment);
        }
    }
    // TODO - BIG TODO:
    // - calculate block threat position for num of crease defenders ->
    // - loop through all crease defenders and update control params to include the
    // position
    // - Get position of all robots from the CURRENT defenders ->
    // - Get nearest defender to the ball and check if it is viable for a steal
    // - If viable, pop a defender and assign DribbleTactic, else nothing

    setUpCreaseDefenders(static_cast<unsigned int>(crease_defender_assignments.size()));
    setUpPassDefenders(static_cast<unsigned int>(pass_defender_assignments.size()));
    updateCreaseDefenderControlParams(event, crease_defender_assignments);
    updatePassDefenderControlParams(pass_defender_assignments);

    PriorityTacticVector tactics_to_return = {{}, {}};
    tactics_to_return[0].insert(tactics_to_return[0].end(), crease_defenders.begin(),
                                crease_defenders.end());
    tactics_to_return[1].insert(tactics_to_return[1].end(), pass_defenders.begin(),
                                pass_defenders.end());
    event.common.set_tactics(tactics_to_return);
}
//
// bool DefensePlayFSM::ballNearbyWithoutThreat(const Update& event)
//{
//    Point robot_position = event.common.robot.position();
//    std::optional<Robot> nearest_enemy =
//            event.common.world_ptr->enemyTeam().getNearestRobot(robot_position);
//    if (nearest_enemy)
//    {
//        // Get the ball if ball is closer to robot than enemy threat by threshold ratio
//        double ball_distance =
//                distance(robot_position, event.common.world_ptr->ball().position());
//        double nearest_enemy_distance =
//                distance(robot_position, nearest_enemy->position());
//
//        return ball_distance < nearest_enemy_distance * MAX_GET_BALL_RATIO_THRESHOLD &&
//               ball_distance <= MAX_GET_BALL_RADIUS_M &&
//               event.common.world_ptr->ball().velocity().length() <=
//               MAX_BALL_SPEED_TO_GET_MS;
//    }
//    else
//    {
//        return true;
//    }
//}
//
// void DefensePlayFSM::prepareGetPossession(
//        const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent)
//{
//    Point ball_position       = event.common.world_ptr->ball().position();
//    Point enemy_goal_center   = event.common.world_ptr->field().enemyGoal().centre();
//    Vector ball_to_net_vector = Vector(enemy_goal_center.x() - ball_position.x(),
//                                       enemy_goal_center.y() - ball_position.y());
//    DribbleFSM::ControlParams control_params{
//            .dribble_destination       = ball_position,
//            .final_dribble_orientation = ball_to_net_vector.orientation(),
//            .allow_excessive_dribbling = false};
//    processEvent(DribbleFSM::Update(control_params, event.common));
//}
//
