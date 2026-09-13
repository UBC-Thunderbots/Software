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

#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <btBulletDynamicsCommon.h>

#include <Eigen/Dense>
#include <Eigen/QR>

#include "extlibs/er_force_sim/src/core/rng.h"
#include "extlibs/er_force_sim/src/protobuf/command.pb.h"
#include "extlibs/er_force_sim/src/protobuf/robot.pb.h"
#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/ssl_simulation_robot_control.pb.h"
#include "proto/ssl_vision_detection.pb.h"
#include "simball.h"

namespace camun
{
namespace simulator
{
class SimRobot;
}  // namespace simulator
}  // namespace camun

class camun::simulator::SimRobot
{
   public:
    SimRobot(const robot::Specs& specs, std::shared_ptr<btDiscreteDynamicsWorld> world,
             const btVector3& pos, float dir);
    ~SimRobot();

   public:
    void begin(SimBall& ball, double time);

    bool canKickBall(const SimBall& ball) const;

    void tryKick(const SimBall& ball, float power, double time);

    robot::RadioResponse setCommand(const SSLSimulationProto::RobotCommand& command,
                                    const SimBall& ball, bool charge, float rxLoss,
                                    float txLoss);

    void update(SSLProto::SSL_DetectionRobot& robot, float stddev_p, float stddev_phi,
                int64_t time, btVector3 positionOffset);

    void update(world::SimRobot& robot, const SimBall& ball) const;

    void restoreState(const world::SimRobot& robot);

    void move(const sslsim::TeleportRobot& robot);

    bool isFlipped();

    btVector3 position() const;

    btVector3 dribblerCorner(bool left) const;

    int64_t getLastSendTime() const
    {
        return m_lastSendTime;
    }

    void setDribbleMode(bool perfectDribbler);

    void stopDribbling();

    /**
     * Sets how much the robot deviates from driving straight, as a fraction of its
     * forward acceleration that is applied as rotational acceleration
     *
     * @param error the rotation error
     */
    void setRotationError(float error)
    {
        m_rotationError = error;
    }

    const robot::Specs& specs() const
    {
        return m_specs;
    }

    /**
     * determines whether the robot is in contact with the ball according to the bullet
     * engine
     * @param ball the ball in play
     * @return true if the ball is in contact with the robot, false otherwise
     */
    bool touchesBall(const SimBall& ball) const;


   private:
    btVector3 relativeBallSpeed(const SimBall& ball) const;
    float bound(float acceleration, float oldSpeed, float speedupLimit,
                float brakeLimit) const;
    void calculateDribblerMove(const btVector3 pos, const btQuaternion rot,
                               const btVector3 linVel, float omega);
    void dribble(const SimBall& ball, float speed);

    /**
     * Builds the matrix that maps the robot's local velocity to the speed of each of
     * its wheels, and its inverse, from the robot's simulation limits
     */
    void generateVelocityCoupling();

    /**
     * Limits the given acceleration so that no single wheel accelerates faster than
     * the robot's simulation limits allow
     *
     * @param a_f the forward acceleration
     * @param a_s the sideways acceleration
     * @param a_phi the rotational acceleration
     * @param v_f the current forward velocity
     * @param v_s the current sideways velocity
     * @param omega the current rotational velocity
     *
     * @return the bounded {a_s, a_f, a_phi}
     */
    Eigen::Vector3f limitAcceleration(float a_f, float a_s, float a_phi, float v_f,
                                      float v_s, float omega) const;

    RNG m_rng;
    robot::Specs m_specs;
    std::shared_ptr<btDiscreteDynamicsWorld> m_world;
    std::unique_ptr<btRigidBody> m_body;
    std::unique_ptr<btRigidBody> m_dribblerBody;
    std::unique_ptr<btHingeConstraint> m_dribblerConstraint;
    std::vector<std::unique_ptr<btCollisionShape>> m_shapes;
    std::unique_ptr<btMotionState> m_motionState;
    std::unique_ptr<btPoint2PointConstraint> m_holdBallConstraint;
    // Keeps a robot that is holding the ball from tipping over, see dribble()
    std::unique_ptr<btGeneric6DofSpring2Constraint> m_notTipOverConstraint;
    btVector3 m_dribblerCenter;

    sslsim::TeleportRobot m_move;
    SSLSimulationProto::RobotCommand m_sslCommand;
    bool m_charge;
    bool m_isCharged;
    bool m_inStandby;
    double m_shootTime;
    double m_commandTime;

    float m_error_sum_v_s;
    float m_error_sum_v_f;
    float m_error_sum_omega;

    bool m_perfectDribbler = false;
    float m_rotationError  = 0.0f;

    // Whether this robot limits the acceleration of each of its wheels individually,
    // rather than just its acceleration as a whole
    bool m_limitWheelAcceleration = false;
    Eigen::Matrix<float, 4, 3> m_velocityCoupling;
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<float, 4, 3>> m_inverseCoupling;

    int64_t m_lastSendTime = 0;
};

#endif  // SIMROBOT_H
