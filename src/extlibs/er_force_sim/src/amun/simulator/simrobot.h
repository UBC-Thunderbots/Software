/***************************************************************************
 *   Copyright 2015 Michael Eischer, Jan Kallwies, Philipp Nordhus         *
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

#ifndef SIMROBOT_H
#define SIMROBOT_H

#include <btBulletDynamicsCommon.h>

#include <QtCore/QList>

#include "extlibs/er_force_sim/src/protobuf/command.pb.h"
#include "extlibs/er_force_sim/src/protobuf/robot.pb.h"
#include "extlibs/er_force_sim/src/protobuf/sslsim.h"
#include "proto/ssl_simulation_robot_control.pb.h"

class RNG;
namespace SSLProto
{
    class SSL_DetectionRobot;
}

namespace camun
{
    namespace simulator
    {
        class SimBall;
        class SimRobot;
        enum class ErrorSource;
    }  // namespace simulator
}  // namespace camun

class camun::simulator::SimRobot : public QObject
{
    Q_OBJECT
   public:
    SimRobot(RNG *rng, const robot::Specs &specs, btDiscreteDynamicsWorld *world,
             const btVector3 &pos, float dir);
    ~SimRobot();
    SimRobot(const SimRobot &) = delete;
    SimRobot &operator=(const SimRobot &) = delete;

   signals:
    void sendSSLSimError(const SSLSimError &error, ErrorSource s);

   public:
    void begin(SimBall *ball, double time);
    bool canKickBall(SimBall *ball) const;
    void tryKick(SimBall *ball, float power, double time);
    robot::RadioResponse setCommand(const SSLSimulationProto::RobotCommand &command,
                                    SimBall *ball, bool charge, float rx_loss,
                                    float tx_loss);
    void update(SSLProto::SSL_DetectionRobot *robot, float stddev_p, float stddev_phi,
                qint64 time, btVector3 position_offset);
    void update(world::SimRobot *robot, SimBall *ball) const;
    void restoreState(const world::SimRobot &robot);
    void move(const sslsim::TeleportRobot &robot);
    bool isFlipped();
    btVector3 position() const;
    btVector3 dribblerCorner(bool left) const;
    qint64 getLastSendTime() const
    {
        return m_last_send_time;
    }
    void setDribbleMode(bool perfect_dribbler);
    void stopDribbling();

    const robot::Specs &specs() const
    {
        return m_specs;
    }

    /**
     * determines whether the robot is in contact with the ball according to the bullet
     * engine
     * @param ball the ball in play
     * @return true if the ball is in contact with the robot, false otherwise
     */
    bool touchesBall(SimBall *ball) const;


   private:
    btVector3 relativeBallSpeed(SimBall *ball) const;
    float bound(float acceleration, float old_speed, float speedup_limit,
                float brake_limit) const;
    void calculateDribblerMove(const btVector3 pos, const btQuaternion rot,
                               const btVector3 lin_vel, float omega);
    void dribble(SimBall *ball, float speed);

    RNG *m_rng;
    robot::Specs m_specs;
    btDiscreteDynamicsWorld *m_world;
    btRigidBody *m_body;
    btRigidBody *m_dribbler_body;
    btHingeConstraint *m_dribbler_constraint;
    QList<btCollisionShape *> m_shapes;
    btMotionState *m_motion_state;
    btVector3 m_dribbler_center;
    std::unique_ptr<btHingeConstraint> m_hold_ball_constraint;

    struct Wheel
    {
        float angle;
        btVector3 pos;
        btVector3 dir;
    };

    sslsim::TeleportRobot m_move;
    SSLSimulationProto::RobotCommand m_ssl_command;
    bool m_charge;
    bool m_is_charged;
    bool m_in_standby;
    double m_shoot_time;
    double m_command_time;

    float error_sum_v_s;
    float error_sum_v_f;
    float error_sum_omega;

    bool m_perfect_dribbler = false;

    qint64 m_last_send_time = 0;
};

#endif  // SIMROBOT_H
