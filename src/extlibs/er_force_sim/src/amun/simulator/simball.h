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

#include "extlibs/er_force_sim/src/core/rng.h"
#include "extlibs/er_force_sim/src/protobuf/command.pb.h"
#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/ssl_vision_detection.pb.h"
#include "shared/constants.h"

namespace camun
{
    namespace simulator
    {
        class SimBall;
    }  // namespace simulator
}  // namespace camun

class camun::simulator::SimBall
{
   public:
    SimBall(std::shared_ptr<btDiscreteDynamicsWorld> world);
    ~SimBall();

   public:
    /**
     * processes velocity and forces to be applied on the ball
     * @param robot_collision whether the ball collides with a robot in this simulation
     * tick
     */
    void begin(bool robot_collision);
    bool update(SSLProto::SSL_DetectionBall& ball, float stddev, float stddevArea,
                const btVector3 &cameraPosition, bool enableInvisibleBall,
                float visibilityThreshold, btVector3 positionOffset);
    void move(const sslsim::TeleportBall &ball);
    void kick(const btVector3 &power);
    // returns the ball position projected onto the floor (z component is not included)
    btVector3 position() const;
    btVector3 speed() const;
    void writeBallState(world::SimBall &ball) const;
    void restoreState(const world::SimBall &ball);
    btRigidBody *body() const
    {
        return m_body.get();
    }
    bool isInvalid() const;

    // can be used to add ball mis-detections
    bool addDetection(SSLProto::SSL_DetectionBall& ball, btVector3 pos, float stddev,
                      float stddevArea, const btVector3 &cameraPosition,
                      bool enableInvisibleBall, float visibilityThreshold,
                      btVector3 positionOffset);

   private:
    RNG m_rng;
    std::shared_ptr<btDiscreteDynamicsWorld> m_world;
    std::unique_ptr<btCollisionShape> m_sphere;
    std::unique_ptr<btRigidBody> m_body;
    std::unique_ptr<btMotionState> m_motionState;
    sslsim::TeleportBall m_move;
    double m_rolling_speed;
    bool m_set_transition_speed;

    enum BallState
    {
        STATIONARY,
        ROBOT_COLLISION,
        SLIDING,
        ROLLING
    };

    BallState m_current_ball_state;
};

#endif  // SIMBALL_H
