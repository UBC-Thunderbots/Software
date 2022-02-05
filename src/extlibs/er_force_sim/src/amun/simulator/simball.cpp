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

#include "simball.h"

#include <QtCore/QDebug>
#include <cmath>

#include "extlibs/er_force_sim/src/core/coordinates.h"
#include "extlibs/er_force_sim/src/core/rng.h"
#include "extlibs/er_force_sim/src/core/vector.h"
#include "proto/messages_robocup_ssl_detection.pb.h"
#include "simulator.h"

using namespace camun::simulator;

SimBall::SimBall(RNG *rng, btDiscreteDynamicsWorld *world) : m_rng(rng), m_world(world)
{
    // see http://robocup.mi.fu-berlin.de/buch/rolling.pdf for correct modelling
    m_sphere = new btSphereShape(BALL_RADIUS * SIMULATOR_SCALE);

    btVector3 localInertia(0, 0, 0);
    // FIXME measure inertia coefficient
    m_sphere->calculateLocalInertia(BALL_MASS, localInertia);

    btTransform startWorldTransform;
    startWorldTransform.setIdentity();
    startWorldTransform.setOrigin(btVector3(0, 0, BALL_RADIUS) * SIMULATOR_SCALE);
    m_motionState = new btDefaultMotionState(startWorldTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(BALL_MASS, m_motionState, m_sphere,
                                                    localInertia);

    // parameters seem to be ignored...
    m_body = new btRigidBody(rbInfo);
    // see simulator.cpp
    m_body->setRestitution(1.f);
    m_body->setFriction(1.f);

    // \mu_r = -a / g = 0.0357 (while rolling)
    // rollingFriction in bullet is too unstable to be useful
    // use custom implementation in begin()
    m_world->addRigidBody(m_body);
}

SimBall::~SimBall()
{
    m_world->removeRigidBody(m_body);
    delete m_body;
    delete m_sphere;
    delete m_motionState;
}

void SimBall::begin()
{
    // custom implementation of rolling friction
    const btVector3 p = m_body->getWorldTransform().getOrigin();
    if (p.z() < BALL_RADIUS * 1.1 * SIMULATOR_SCALE)
    {  // ball is on the ground
        const btVector3 velocity = m_body->getLinearVelocity();
        if (velocity.length() < 0.01 * SIMULATOR_SCALE)
        {
            // stop the ball if it is really slow
            // -> the real ball snaps to a dimple
            m_body->setLinearVelocity(btVector3(0, 0, 0));
        }
        else
        {
            // just apply rolling friction, normal friction is somehow handled by
            // bullet this is quite a hack as it's always applied but as the strong
            // deceleration is more or less magic, some additional deceleration
            // doesn't matter
            const btScalar hackFactor          = 1.4;
            const btScalar rollingDeceleration = hackFactor * 0.35;
            btVector3 force(velocity.x(), velocity.y(), 0.0f);
            force.safeNormalize();
            m_body->applyCentralImpulse(-force * rollingDeceleration * SIMULATOR_SCALE *
                                        BALL_MASS * SUB_TIMESTEP);
        }
    }

    bool moveCommand           = false;
    auto sendPartialCoordError = [this](const char *msg) {
        SSLSimError error{new sslsim::SimulatorError};
        error->set_code("PARTIAL_COORD");
        std::string message = "Partial coordinates are not implemented yet";
        error->set_message(message + msg);
        emit this->sendSSLSimError(error, ErrorSource::CONFIG);
    };
    if (m_move.has_x())
    {
        if (!m_move.has_y())
        {
            sendPartialCoordError(": position ball");
            return;
        }
        moveCommand = true;
    }
    else if (m_move.has_y() || m_move.has_z())
    {
        sendPartialCoordError(": position ball (not x)");
        return;
    }

    if (m_move.has_vx())
    {
        if (!m_move.has_vy())
        {
            sendPartialCoordError(": velocity ball");
            return;
        }
        if (m_move.by_force() && (m_move.vx() != 0 || m_move.vy() != 0 ||
                                  (m_move.has_vz() && m_move.vz() != 0)))
        {
            SSLSimError error{new sslsim::SimulatorError};
            error->set_code("VELOCITY_FORCE");
            error->set_message("Velocities != 0 and by_force are incompatible");
            emit sendSSLSimError(error, ErrorSource::CONFIG);
            return;
        }
        moveCommand = true;
    }

    if (moveCommand)
    {
        if (m_move.by_force())
        {
            Vector pos;
            coordinates::fromVision(m_move, pos);
            // move ball by hand
            btVector3 force(pos.x, pos.y, m_move.z() + BALL_RADIUS);
            force = force - m_body->getWorldTransform().getOrigin() / SIMULATOR_SCALE;
            m_body->activate();
            m_body->applyCentralImpulse(force * BALL_MASS * 0.1 * SIMULATOR_SCALE);
            m_body->setDamping(0.99, 0.99);
        }
        else
        {
            if (m_move.has_x())
            {
                // set position
                Vector cPos;
                coordinates::fromVision(m_move, cPos);
                float height = BALL_RADIUS;
                if (m_move.has_z())
                {
                    height += m_move.z();
                }
                const btVector3 pos(cPos.x, cPos.y, height);
                btTransform transform = m_body->getWorldTransform();
                transform.setOrigin(pos * SIMULATOR_SCALE);
                m_body->setWorldTransform(transform);
            }
            if (m_move.has_vx())
            {
                Vector vel;
                coordinates::fromVisionVelocity(m_move, vel);
                float vz = 0;
                if (m_move.has_vz())
                {
                    vz = m_move.vz() * 1e-3;
                }
                const btVector3 linVel(vel.x, vel.y, vz);
                m_body->setLinearVelocity(linVel * SIMULATOR_SCALE);
                m_body->setAngularVelocity(btVector3(0, 0, 0));
            }
            m_body->activate();
            m_body->setDamping(0.0, 0.0);
            m_move.Clear();  // clear move command
                             // reset is neccessary, as the command is only sent once
                             // without one canceling it
        }
    }
    else
    {
        // ball is slowed down by rolling friction, not damping!
        m_body->setDamping(0.0, 0.0);
    }
}

// samples a plane rotated towards the camera, sets p to the average position of
// the visible points and returns the relative amount of visible pixels
static float positionOfVisiblePixels(btVector3 &p, const btVector3 &simulatorBallPosition,
                                     const btVector3 &simulatorCameraPosition,
                                     const btCollisionWorld *const m_world)
{
    const float simulatorBallRadius = BALL_RADIUS * SIMULATOR_SCALE;

    // axis and angle for rotating the plane towards the camera
    btVector3 cameraDirection =
        (simulatorCameraPosition - simulatorBallPosition).normalize();

    const btVector3 up = btVector3(0, 0, 1);

    btVector3 axis = up.cross(cameraDirection).normalize();
    btScalar angle = up.angle(cameraDirection);

    const int sampleRadius       = 7;
    std::size_t maxHits          = pow((2 * sampleRadius + 1), 2);
    std::size_t cameraHitCounter = 0;

    btVector3 newPos = btVector3(0, 0, 0);

    for (int x = -sampleRadius; x <= sampleRadius; ++x)
    {
        for (int y = -sampleRadius; y <= sampleRadius; ++y)
        {
            // create offset to the midpoint of the plane
            btVector3 offset(x, y, 0);
            offset *= simulatorBallRadius / sampleRadius;

            // ignore samples outside the ball
            if (offset.length() >= simulatorBallRadius)
            {
                --maxHits;
                continue;
            }

            // rotate the plane/offset
            offset.rotate(axis, angle);
            btVector3 samplePoint = simulatorBallPosition + offset;

            btCollisionWorld::ClosestRayResultCallback sampleResult(
                samplePoint, simulatorCameraPosition);
            m_world->rayTest(samplePoint, simulatorCameraPosition, sampleResult);

            if (!sampleResult.hasHit())
            {
                // transform point back to plane with upwards normal to ensure unchanged
                // height
                samplePoint -= simulatorBallPosition;
                samplePoint.rotate(axis, -angle);
                samplePoint += simulatorBallPosition;

                newPos += samplePoint;

                ++cameraHitCounter;
            }
        }
    }

    if (cameraHitCounter > 0)
    {
        // return average position of samples transformed to real world space
        newPos /= cameraHitCounter;
        p = newPos / SIMULATOR_SCALE;
    }

    return static_cast<float>(cameraHitCounter) / static_cast<float>(maxHits);
}

bool SimBall::update(SSLProto::SSL_DetectionBall *ball, float stddev, float stddevArea,
                     const btVector3 &cameraPosition, bool enableInvisibleBall,
                     float visibilityThreshold, btVector3 positionOffset)
{
    btTransform transform;
    m_motionState->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin() / SIMULATOR_SCALE;

    return addDetection(ball, pos, stddev, stddevArea, cameraPosition,
                        enableInvisibleBall, visibilityThreshold, positionOffset);
}

bool SimBall::addDetection(SSLProto::SSL_DetectionBall *ball, btVector3 pos, float stddev,
                           float stddevArea, const btVector3 &cameraPosition,
                           bool enableInvisibleBall, float visibilityThreshold,
                           btVector3 positionOffset)
{
    // setup ssl-vision ball detection
    ball->set_confidence(1.0);
    ball->set_pixel_x(0);
    ball->set_pixel_y(0);

    btTransform transform;
    m_motionState->getWorldTransform(transform);

    const btVector3 simulatorCameraPosition =
        btVector3(cameraPosition.x(), cameraPosition.y(), cameraPosition.z()) *
        SIMULATOR_SCALE;

    float visibility = 1;
    // the camera uses the mid point of the visible pixels as the mid point of the
    // ball if some parts of the ball aren't visible the position this function
    // adjusts the position accordingly (hopefully)
    if (enableInvisibleBall)
    {
        // if the visibility is lower than the threshold the ball disappears
        visibility = positionOfVisiblePixels(pos, transform.getOrigin(),
                                             simulatorCameraPosition, m_world);
        if (visibility < visibilityThreshold)
        {
            return false;
        }
    }

    const float SCALING_LIMIT = 0.9f;
    // reflects the cameras resolution
    const unsigned PIXEL_PER_AREA = 10;  // i do not know in what unit area is,
                                         // just make it similar to a real game

    float modZ = std::min(SCALING_LIMIT * cameraPosition.z(),
                          std::max(0.f, pos.z() - BALL_RADIUS));
    float modX = (pos.x() - cameraPosition.x()) *
                     (cameraPosition.z() / (cameraPosition.z() - modZ)) +
                 cameraPosition.x();
    float modY = (pos.y() - cameraPosition.y()) *
                     (cameraPosition.z() / (cameraPosition.z() - modZ)) +
                 cameraPosition.y();

    float distBallCam =
        std::sqrt((cameraPosition.z() - modZ) * (cameraPosition.z() - modZ) +
                  (cameraPosition.x() - pos.x()) * (cameraPosition.x() - pos.x()) +
                  (cameraPosition.y() - pos.y()) * (cameraPosition.y() - pos.y()));
    float denomSqrt = (distBallCam * 1000) / FOCAL_LENGTH - 1;
    float basePixelArea =
        (BALL_RADIUS * BALL_RADIUS * 1000000 * M_PI) / (denomSqrt * denomSqrt);
    float area =
        visibility *
        std::max(0.0f, (basePixelArea +
                        static_cast<float>(m_rng->normal(stddevArea)) / PIXEL_PER_AREA));
    ball->set_area(area * PIXEL_PER_AREA);

    // if (height > 0.1f) {
    //     qDebug() << "simball" << p.x() << p.y() << height << "ttt" << ball_x <<
    //     ball_y;
    // }

    // add noise to coordinates
    // to convert from bullet coordinate system to ssl-vision rotate by 90 degree
    // ccw
    const Vector noise = m_rng->normalVector(stddev);
    coordinates::toVision(Vector(modX, modY) + noise, *ball);
    return true;
}

void SimBall::move(const sslsim::TeleportBall &ball)
{
    m_move = ball;
}

btVector3 SimBall::position() const
{
    const btTransform transform = m_body->getWorldTransform();
    return btVector3(transform.getOrigin().x(), transform.getOrigin().y(), 0);
}

btVector3 SimBall::speed() const
{
    return m_body->getLinearVelocity();
}

void SimBall::writeBallState(world::SimBall *ball) const
{
    const btVector3 ballPosition =
        m_body->getWorldTransform().getOrigin() / SIMULATOR_SCALE;
    ball->set_p_x(ballPosition.getX());
    ball->set_p_y(ballPosition.getY());
    ball->set_p_z(ballPosition.getZ());
    const btVector3 ballSpeed = speed() / SIMULATOR_SCALE;
    ball->set_v_x(ballSpeed.getX());
    ball->set_v_y(ballSpeed.getY());
    ball->set_v_z(ballSpeed.getZ());
    const btVector3 angularVelocity = m_body->getAngularVelocity();
    ball->set_angular_x(angularVelocity.x());
    ball->set_angular_y(angularVelocity.y());
    ball->set_angular_z(angularVelocity.z());
}

void SimBall::restoreState(const world::SimBall &ball)
{
    btVector3 position(ball.p_x(), ball.p_y(), ball.p_z());
    m_body->getWorldTransform().setOrigin(position * SIMULATOR_SCALE);
    btVector3 velocity(ball.v_x(), ball.v_y(), ball.v_z());
    m_body->setLinearVelocity(velocity * SIMULATOR_SCALE);
    btVector3 angular(ball.angular_x(), ball.angular_y(), ball.angular_z());
    m_body->setAngularVelocity(angular);
}

bool SimBall::isInvalid() const
{
    const btTransform transform = m_body->getWorldTransform();
    const btVector3 velocity    = m_body->getLinearVelocity();
    bool isNan =
        std::isnan(transform.getOrigin().x()) || std::isnan(transform.getOrigin().y()) ||
        std::isnan(transform.getOrigin().z()) || std::isinf(transform.getOrigin().x()) ||
        std::isinf(transform.getOrigin().y()) || std::isinf(transform.getOrigin().z()) ||
        std::isnan(velocity.x()) || std::isnan(velocity.y()) ||
        std::isnan(velocity.z()) || std::isinf(velocity.x()) ||
        std::isinf(velocity.y()) || std::isinf(velocity.z());
    bool isBelowField = (transform.getOrigin().z() <= 0);
    return isNan || isBelowField;
}

void SimBall::kick(const btVector3 &power)
{
    m_body->activate();
    m_body->applyCentralForce(power);

    // btTransform transform;
    // m_motionState->getWorldTransform(transform);
    // const btVector3 p = transform.getOrigin() / SIMULATOR_SCALE;
    // qDebug() << "kick at" << p.x() << p.y();
}
