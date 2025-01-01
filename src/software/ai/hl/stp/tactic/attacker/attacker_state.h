#pragma once

#include "software/ai/rl/torch.h"
#include "software/world/team.h"
#include "software/world/world.h"

/**
 * Tensor state representation of the World and other information pertinent
 * to the Attacker agent.
 */
class AttackerState
{
   public:
    /**
     * Constructs an AttackerState from a World.
     *
     * @param world the World to construct the AttackerState from
     * @param last_attacker_robot the ID of the last robot that executed
     * the AttackerTactic
     */
    explicit AttackerState(const World& world, std::optional<RobotId> last_attacker_robot)
        : tensor_(torch::cat(
              {convertTeamToTensor(world.friendlyTeam()),
               convertTeamToTensor(world.enemyTeam()), convertBallToTensor(world.ball()),
               convertGameStateToTensor(world.gameState()),
               torch::tensor({static_cast<float>(last_attacker_robot.value_or(0))})}))
    {
    }

    AttackerState() = default;

    /**
     * Returns the 1-D tensor representation of the AttackerState.
     *
     * @return the tensor representation of the AttackerState
     */
    torch::Tensor getTensor() const
    {
        return tensor_;
    }

    /**
     * Returns the size of the AttackerState tensor.
     *
     * @return the size of the AttackerState tensor
     */
    static size_t size()
    {
        // Static variable will be initialized only once with the result of the lambda
        // when size() is first called
        static const size_t size = []() {
            const World blank_world(Field::createSSLDivisionBField(),
                                    Ball(Point(), Vector(), Timestamp()), Team(), Team());

            return AttackerState(blank_world, std::nullopt).getTensor().size(0);
        }();
        return size;
    }

   private:
    /**
     * Converts a Team into a 1-D tensor representation of the coordinates,
     * velocities, and orientations of the team's robots
     *
     * @param team the Team to convert into a tensor
     *
     * @return a 1-D tensor representation of the Team
     */
    static torch::Tensor convertTeamToTensor(const Team& team)
    {
        std::vector<double> observation;

        for (RobotId robot_id = 0; robot_id < MAX_ROBOT_IDS_PER_SIDE; ++robot_id)
        {
            std::optional<Robot> robot = team.getRobotById(robot_id);
            const Robot& robot_value   = robot.value_or(
                Robot(robot_id, Point(), Vector(), Angle(), Angle(), Timestamp()));

            observation.push_back(robot.has_value());
            observation.push_back(robot_value.position().x());
            observation.push_back(robot_value.position().y());
            observation.push_back(robot_value.velocity().x());
            observation.push_back(robot_value.velocity().y());
            observation.push_back(robot_value.angularVelocity().toRadians());
            observation.push_back(robot_value.orientation().toRadians());
        }

        return torch::tensor(observation, {torch::kFloat32});
    }

    /**
     * Converts a Ball into a 1-D tensor representation of its coordinates,
     * velocity, and acceleration.
     *
     * @param ball the Ball to convert into a tensor
     *
     * @return a 1-D tensor representation of the Ball
     */
    static torch::Tensor convertBallToTensor(const Ball& ball)
    {
        return torch::tensor(
            {ball.position().x(), ball.position().y(), ball.velocity().x(),
             ball.velocity().y(), ball.acceleration().y(), ball.acceleration().y()},
            {torch::kFloat32});
    }

    /**
     * Converts a GameState into a 1-D tensor representation of the GameState's
     * TeamInfos (scores, red/yellow cards, fouls, etc.)
     *
     * @param game_state the GameState to convert into a tensor
     *
     * @return a 1-D tensor representation of the GameState
     */
    static torch::Tensor convertGameStateToTensor(const GameState& game_state)
    {
        const TeamInfo& friendly_team_info = game_state.getFriendlyTeamInfo();
        const TeamInfo& enemy_team_info    = game_state.getEnemyTeamInfo();

        return torch::tensor({static_cast<float>(friendly_team_info.getScore()),
                              static_cast<float>(enemy_team_info.getScore()),
                              static_cast<float>(friendly_team_info.getRedCards()),
                              static_cast<float>(enemy_team_info.getRedCards()),
                              static_cast<float>(friendly_team_info.getYellowCards()),
                              static_cast<float>(enemy_team_info.getYellowCards()),
                              static_cast<float>(friendly_team_info.getFoulsCount()),
                              static_cast<float>(enemy_team_info.getFoulsCount())},
                             {torch::kFloat32});
    }

    // The 1-D tensor representation of the AttackerState
    torch::Tensor tensor_;
};
