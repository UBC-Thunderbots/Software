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

#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtCore/QPair>
#include <QtCore/QQueue>
#include <random>
#include <tuple>

#include "extlibs/er_force_sim/src/protobuf/command.h"
#include "extlibs/er_force_sim/src/protobuf/sslsim.h"
#include "proto/ssl_simulation_robot_control.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"


// higher values break the rolling friction of the ball
const float SIMULATOR_SCALE  = 10.0f;
const float SUB_TIMESTEP     = 1 / 200.f;
const float COLLISION_MARGIN = 0.04f;
const unsigned FOCAL_LENGTH  = 390;


class QByteArray;
class QTimer;
class Timer;
class SSL_GeometryFieldSize;

namespace camun
{
    namespace simulator
    {
        class SimRobot;
        class Simulator;
        class ErrorAggregator;
        struct SimulatorData;

        enum class ErrorSource
        {
            BLUE,
            YELLOW,
            CONFIG
        };
    }  // namespace simulator
}  // namespace camun

class camun::simulator::Simulator : public QObject
{
    Q_OBJECT

   public:
    typedef QMap<unsigned int, QPair<SimRobot *, unsigned int>>
        RobotMap; /*First int: ID, Second int: Generation*/

    /**
     * Creates a simulator with the given set up
     *
     * @param setup the simulator set up
     */
    explicit Simulator(const amun::SimulatorSetup &setup);
    ~Simulator() override;
    Simulator(const Simulator &) = delete;
    Simulator &operator=(const Simulator &) = delete;

    /**
     * Seeds the pseudorandom generator
     *
     * @param seed
     */
    void seedPRGN(uint32_t seed);

   signals:
    void gotPacket(const QByteArray &data, qint64 time, QString sender);
    void sendRadioResponses(const QList<robot::RadioResponse> &responses);
    void sendRealData(const QByteArray &data);  // sends amun::SimulatorState
    void sendSSLSimError(const QList<SSLSimError> &errors, ErrorSource source);

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

   public slots:
    void handleRadioCommands(const SSLSimRobotControl &control, bool is_blue,
                             qint64 processing_start);
    void setFlipped(bool flipped);

   private:
    /**
     * Accepts and executes a blue or yellow robot control command
     *
     * @param control the robot control command
     * @param is_blue whether it's blue robots or not
     *
     * @return the radio response feedback from the simulator
     */
    std::vector<robot::RadioResponse> acceptRobotControlCommand(
        const SSLSimulationProto::RobotControl &control, bool is_blue);
    void sendSSLSimErrorInternal(ErrorSource source);
    void resetFlipped(RobotMap &robots, float side);
    std::tuple<QList<QByteArray>, QByteArray, qint64> createVisionPacket();
    void resetVisionPackets();
    void setTeam(RobotMap &list, float side, const robot::Team &team,
                 QMap<uint32_t, robot::Specs> &team_specs);
    void moveBall(const sslsim::TeleportBall &ball);
    void moveRobot(const sslsim::TeleportRobot &robot);
    void teleportRobotToFreePosition(SimRobot *robot);
    void initializeDetection(SSLProto::SSL_DetectionFrame *detection,
                             std::size_t camera_id);

   private:
    typedef std::tuple<SSLSimRobotControl, qint64, bool> RadioCommand;
    SimulatorData *m_data;
    QQueue<RadioCommand> m_radio_commands;
    QQueue<std::tuple<QList<QByteArray>, QByteArray, qint64>> m_vision_packets;
    QQueue<QTimer *> m_vision_timers;
    QTimer *m_trigger;
    qint64 m_time;
    qint64 m_last_sent_status_time;
    bool m_enabled;
    bool m_charge;
    // systemDelay + visionProcessingTime = visionDelay
    qint64 m_vision_delay;
    qint64 m_vision_processing_time;

    qint64 m_min_robot_detection_time = 0;
    qint64 m_min_ball_detection_time  = 0;
    qint64 m_last_ball_send_time      = 0;
    std::map<qint64, unsigned> m_last_frame_number;
    ErrorAggregator *m_aggregator;



    std::mt19937 rand_shuffle_src = std::mt19937(std::random_device()());
};

#endif  // SIMULATOR_H
