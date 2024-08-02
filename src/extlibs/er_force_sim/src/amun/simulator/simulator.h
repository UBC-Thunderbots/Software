/***************************************************************************
 *   Copyright 2015 Michael Eischer, Philipp Nordhus                       *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <memory>
#include <random>
#include <utility>

#include "extlibs/er_force_sim/src/core/rng.h"
#include "proto/ssl_simulation_robot_control.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"
#include "simball.h"
#include "simfield.h"
#include "simrobot.h"

// higher values break the rolling friction of the ball
const float SIMULATOR_SCALE  = 10.0f;
const float SUB_TIMESTEP     = 1 / 200.f;
const float COLLISION_MARGIN = 0.04f;
const float FOCAL_LENGTH     = 390.f;

namespace camun
{
    namespace simulator
    {
        class Simulator;
        struct SimulatorData;
    }  // namespace simulator
}  // namespace camun

class camun::simulator::Simulator
{
   public:
    typedef std::map<unsigned int, std::unique_ptr<SimRobot>> RobotMap;

    /**
     * Creates a simulator with the given set up
     *
     * @param setup the simulator set up
     */
    explicit Simulator(const amun::SimulatorSetup &setup);

   public:
    /**
     * Accepts and executes a blue or yellow robot control command
     *
     * @param control the robot control command
     *
     * @return the radio response feedback from the simulator
     */
    std::vector<robot::RadioResponse> acceptBlueRobotControlCommand(
        const SSLSimulationProto::RobotControl &control);
    std::vector<robot::RadioResponse> acceptYellowRobotControlCommand(
        const SSLSimulationProto::RobotControl &control);

    /**
     * Steps the simulation a given amount of time
     *
     * @param time_s time in seconds
     */
    void stepSimulation(double time_s);

    /**
     * Handles a tick of the simulator
     * Note: this function is required for bullet simulator callback
     *
     * @param time_s time in seconds
     */
    void handleSimulatorTick(double timeStep);

    /**
     * Generates wrapper packets from the current state of the simulator
     *
     * @return list of wrapper packets
     */
    std::vector<SSLProto::SSL_WrapperPacket> getWrapperPackets();

    /**
     * Gets the current simulator state of the simulator
     *
     * @return simulator state
     */
    world::SimulatorState getSimulatorState();

    /**
     * Handles a simulator set up command and configure the simulator accordingly
     *
     * @param command the simulator set up command
     */
    void handleSimulatorSetupCommand(const std::unique_ptr<amun::Command> &command);

   private:
    /**
     * Accepts and executes a blue or yellow robot control command
     *
     * @param control the robot control command
     * @param isBlue whether it's blue robots or not
     *
     * @return the radio response feedback from the simulator
     */
    std::vector<robot::RadioResponse> acceptRobotControlCommand(
        const SSLSimulationProto::RobotControl &control, bool isBlue);
    void resetFlipped(RobotMap &robots, float side);
    void setTeam(RobotMap &list, float side, const robot::Team &team,
                 std::map<uint32_t, robot::Specs> &specs);
    void moveBall(const sslsim::TeleportBall &ball);
    void moveRobot(const sslsim::TeleportRobot &robot);
    void initializeDetection(SSLProto::SSL_DetectionFrame &detection, size_t cameraId);

   private:
    std::unique_ptr<SimulatorData> m_data;

    bool m_enabled;
    bool m_charge;

    int64_t m_time;
    int64_t m_visionDelay;  // systemDelay + visionProcessingTime = visionDelay
    int64_t m_visionProcessingTime;
    int64_t m_minRobotDetectionTime = 0;
    int64_t m_minBallDetectionTime  = 0;
    int64_t m_lastBallSendTime      = 0;

    std::map<size_t, unsigned int> m_lastFrameNumber;

    std::mt19937 rand_shuffle_src = std::mt19937(std::random_device()());
};


/* Friction and restitution between robots, ball and field: (empirical
 * measurments) Ball vs. Robot: Restitution: about 0.60 Friction: trial and
 * error in simulator 0.18 (similar results as in reality)
 *
 * Ball vs. Floor:
 * Restitution: sqrt(h'/h) = sqrt(0.314) = 0.56
 * Friction: \mu_k = -a / g (while slipping) = 0.35
 *
 * Robot vs. Floor:
 * Restitution and Friction should be as low as possible
 *
 * Calculations:
 * Variables: r: restitution, f: friction
 * Indices: b: ball; f: floor; r: robot
 *
 * r_b * r_f = 0.56
 * r_b * r_r = 0.60
 * r_f * r_r = small
 * => r_b = 1; r_f = 0.56; r_r = 0.60
 *
 * f_b * f_f = 0.35
 * f_b * f_r = 0.22
 * f_f * f_r = very small
 * => f_b = 1; f_f = 0.35; f_r = 0.22
 */

struct camun::simulator::SimulatorData
{
    RNG rng;
    std::unique_ptr<btDefaultCollisionConfiguration> collision;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btBroadphaseInterface> overlappingPairCache;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
    std::shared_ptr<btDiscreteDynamicsWorld> dynamicsWorld;
    world::Geometry geometry;
    std::vector<SSLProto::SSL_GeometryCameraCalibration> reportedCameraSetup;
    std::vector<btVector3> cameraPositions;
    std::unique_ptr<SimField> field;
    std::shared_ptr<SimBall> ball;
    Simulator::RobotMap robotsBlue;
    Simulator::RobotMap robotsYellow;
    std::map<uint32_t, robot::Specs> specsBlue;
    std::map<uint32_t, robot::Specs> specsYellow;
    bool flip;
    float stddevBall;
    float stddevBallArea;
    float stddevRobot;
    float stddevRobotPhi;
    float ballDetectionsAtDribbler;  // per robot per second
    bool enableInvisibleBall;
    float ballVisibilityThreshold;
    float cameraOverlap;
    float cameraPositionError;
    float objectPositionOffset;
    float robotCommandPacketLoss;
    float robotReplyPacketLoss;
    float missingBallDetections;
    bool dribblePerfect;
};

#endif  // SIMULATOR_H
