#include "network_input/filter/robot_team_filter.h"

#include <algorithm>
#include <cmath>
#include <vector>


RobotTeamFilter::RobotTeamFilter() {}

Team RobotTeamFilter::getFilteredData(
    const Team &current_team_state,
    const std::vector<SSLRobotDetection> &new_robot_detections)
{
    Team new_team_state = current_team_state;

    for (auto robot_detection : new_robot_detections)
    {
        if (new_team_state.getRobotById(robot_detection.id))
        {
            Robot previous_robot_state = *new_team_state.getRobotById(robot_detection.id);

            // Discard any data with an older timestamp. It's likely from a frame that
            // hasn't been updated yet
            if (previous_robot_state.lastUpdateTimestamp() >= robot_detection.timestamp)
            {
                continue;
            }

            // TODO: Removing the expired robots should be moved to the Backend
            // once https://github.com/UBC-Thunderbots/Software/issues/424 is completed
            new_team_state.removeExpiredRobots(robot_detection.timestamp);

            Duration time_diff =
                robot_detection.timestamp - previous_robot_state.lastUpdateTimestamp();

            Vector new_robot_velocity =
                robot_detection.position - previous_robot_state.position();
            new_robot_velocity = new_robot_velocity.norm(new_robot_velocity.len() /
                                                         time_diff.getSeconds());

            AngularVelocity new_robot_angular_velocity =
                robot_detection.orientation - previous_robot_state.orientation();
            new_robot_angular_velocity /= time_diff.getSeconds();

            Robot new_robot_state =
                Robot(robot_detection.id, robot_detection.position, new_robot_velocity,
                      robot_detection.orientation, new_robot_angular_velocity,
                      robot_detection.timestamp);

            new_team_state.updateRobots({new_robot_state});
        }
        else
        {
            Robot new_robot_state =
                Robot(robot_detection.id, robot_detection.position, Vector(69, 69),
                      robot_detection.orientation, AngularVelocity::zero(),
                      robot_detection.timestamp);

            new_team_state.updateRobots({new_robot_state});
        }
    }

    return new_team_state;
}
