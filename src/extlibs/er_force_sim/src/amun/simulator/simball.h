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
#include "simfield.h"

static const float BALL_RADIUS       = 0.0215f;
static const float BALL_MASS         = 0.046f;

//these values are set in coordination with other objects the ball will collide with
static constexpr float BALL_SLIDING_FRICTION = 1.f;
static constexpr float BALL_RESTITUTION = 1.f;

static constexpr float BALL_ROLLING_FRICTION_DECELERATION = 0.5;
static constexpr float FRICTION_TRANSITION_FACTOR = 5.0/7.0;


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
    void begin(double time_s);
    bool update(SSLProto::SSL_DetectionBall *ball, float stddev, float stddevArea,
                const btVector3 &cameraPosition, bool enableInvisibleBall,
                float visibilityThreshold, btVector3 positionOffset);
    void move(const sslsim::TeleportBall &ball);
    void kick(const btVector3 &power, double velocity);
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
    double rolling_speed = -1;
    bool rollWhenPossible = false;

};

#endif  // SIMBALL_H
