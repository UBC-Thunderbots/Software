#pragma once

#include "software/ai/rl/torch.h"
#include "software/world/team.h"
#include "software/world/world.h"

struct AttackerState
{
    torch::Tensor tensor;

    explicit AttackerState(const World& world)
        : tensor(torch::cat({convertTeamToTensor(world.friendlyTeam()),
                             convertTeamToTensor(world.enemyTeam()),
                             convertBallToTensor(world.ball())}))
    {
    }

    static size_t size()
    {
        const World blank_world(Field::createSSLDivisionBField(),
                                Ball(Point(), Vector(), Timestamp()), Team(), Team());

        return AttackerState(blank_world).tensor.size(0);
    }

   private:
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

    static torch::Tensor convertBallToTensor(const Ball& ball)
    {
        return torch::tensor(
            {ball.position().x(), ball.position().y(), ball.velocity().x(),
             ball.velocity().y(), ball.acceleration().y(), ball.acceleration().y()},
            {torch::kFloat32});
    }
};