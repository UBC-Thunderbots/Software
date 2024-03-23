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

#ifndef SIMBALL_H
#define SIMBALL_H

#include <btBulletDynamicsCommon.h>

#include <QtCore/QObject>

#include "extlibs/er_force_sim/src/protobuf/command.pb.h"
#include "extlibs/er_force_sim/src/protobuf/sslsim.h"
#include "shared/constants.h"
#include "simfield.h"

class RNG;
namespace SSLProto
{
    class SSL_DetectionBall;
}

namespace camun
{
    namespace simulator
    {
        class SimBall;
        enum class ErrorSource;
    }  // namespace simulator
}  // namespace camun

class camun::simulator::SimBall : public QObject
{
    Q_OBJECT
   public:
    SimBall(RNG *rng, btDiscreteDynamicsWorld *world);
    ~SimBall();
    SimBall(const SimBall &) = delete;
    SimBall &operator=(const SimBall &) = delete;

   signals:
    void sendSSLSimError(const SSLSimError &error, ErrorSource s);

   public:
    /**
     * processes velocity and forces to be applied on the ball
     * @param robot_collision whether the ball collides with a robot in this simulation
     * tick
     */
    void begin(bool robot_collision);
    bool update(SSLProto::SSL_DetectionBall *ball, float stddev, float stddevArea,
                const btVector3 &cameraPosition, bool enableInvisibleBall,
                float visibilityThreshold, btVector3 positionOffset);
    void move(const sslsim::TeleportBall &ball);
    void kick(const btVector3 &power);
    // returns the ball position projected onto the floor (z component is not included)
    btVector3 position() const;
    btVector3 speed() const;
    void writeBallState(world::SimBall *ball) const;
    void restoreState(const world::SimBall &ball);
    btRigidBody *body() const
    {
        return m_body;
    }
    bool isInvalid() const;

    // can be used to add ball mis-detections
    bool addDetection(SSLProto::SSL_DetectionBall *ball, btVector3 pos, float stddev,
                      float stddevArea, const btVector3 &cameraPosition,
                      bool enableInvisibleBall, float visibilityThreshold,
                      btVector3 positionOffset);

   private:
    RNG *m_rng;
    btDiscreteDynamicsWorld *m_world;
    btCollisionShape *m_sphere;
    btRigidBody *m_body;
    btMotionState *m_motionState;
    sslsim::TeleportBall m_move;
    double rolling_speed;
    bool set_transition_speed;

    enum BallState
    {
        STATIONARY,
        ROBOT_COLLISION,
        SLIDING,
        ROLLING
    };
    BallState current_ball_state;
};

#endif  // SIMBALL_H
