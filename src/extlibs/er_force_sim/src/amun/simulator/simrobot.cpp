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

#include "simrobot.h"

#include <cmath>

#include "extlibs/er_force_sim/src/core/coordinates.h"
#include "extlibs/er_force_sim/src/core/rng.h"
#include "mesh.h"
#include "proto/ssl_vision_detection.pb.h"
#include "simball.h"
#include "simulator.h"

using namespace camun::simulator;

const float MAX_SPEED = 1000;

float boundSpeed(float speed)
{
    return qBound(-MAX_SPEED, speed, MAX_SPEED);
}

SimRobot::SimRobot(RNG *rng, const robot::Specs &specs, btDiscreteDynamicsWorld *world,
                   const btVector3 &pos, float dir)
    : m_rng(rng),
      m_specs(specs),
      m_world(world),
      m_charge(false),
      m_isCharged(false),
      m_inStandby(false),
      m_shootTime(0.0),
      m_commandTime(0.0),
      error_sum_v_s(0),
      error_sum_v_f(0),
      error_sum_omega(0)
{
    btCompoundShape *wholeShape = new btCompoundShape;
    btTransform robotShapeTransform;
    robotShapeTransform.setIdentity();

    // subtract collision margin from dimensions
    Mesh mesh(m_specs.radius() - COLLISION_MARGIN / SIMULATOR_SCALE,
              m_specs.height() - 2 * COLLISION_MARGIN / SIMULATOR_SCALE, m_specs.angle(),
              0.04f, m_specs.dribbler_height() + 0.02f);
    for (const QList<QVector3D> &hullPart : mesh.hull())
    {
        btConvexHullShape *hullPartShape = new btConvexHullShape;
        m_shapes.append(hullPartShape);
        for (const QVector3D &v : hullPart)
        {
            hullPartShape->addPoint(btVector3(v.x(), v.y(), v.z()) * SIMULATOR_SCALE);
        }
        wholeShape->addChildShape(robotShapeTransform, hullPartShape);
    }
    m_shapes.append(wholeShape);

    btTransform startWorldTransform;
    startWorldTransform.setIdentity();
    btVector3 robotBasePos(btVector3(pos.x(), pos.y(), m_specs.height() / 2.0f) *
                           SIMULATOR_SCALE);
    startWorldTransform.setOrigin(robotBasePos);
    startWorldTransform.setRotation(btQuaternion(btVector3(0, 0, 1), dir - M_PI_2));
    m_motionState = new btDefaultMotionState(startWorldTransform);

    // set robot dynamics and move to start position
    btVector3 localInertia(0, 0, 0);
    const float robotMassProportion = 49.0f / 50.0f;
    wholeShape->calculateLocalInertia(robotMassProportion * m_specs.mass(), localInertia);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        robotMassProportion * m_specs.mass(), m_motionState, wholeShape, localInertia);

    m_body = new btRigidBody(rbInfo);
    // see simulator.cpp
    m_body->setRestitution(0.6f);
    m_body->setFriction(0.22f);
    m_world->addRigidBody(m_body);

    btCylinderShape *dribblerShape = new btCylinderShapeX(
        btVector3(m_specs.dribbler_width() / 2.0f, 0.007f, 0.007f) * SIMULATOR_SCALE);
    m_shapes.append(dribblerShape);
    // WARNING: hack, instead of 0.02 should be the dribbler height
    // the ball seems to get instable if the dribbler is at correct height
    // possibly the ball gets 'sucked' onto the robot
    m_dribblerCenter = btVector3(
        btVector3(0, m_specs.shoot_radius() - 0.01f, -m_specs.height() / 2.0f + 0.02f) *
        SIMULATOR_SCALE);
    btTransform dribblerStartTransform;
    dribblerStartTransform.setIdentity();
    dribblerStartTransform.setOrigin(m_dribblerCenter + robotBasePos);
    dribblerStartTransform.setRotation(btQuaternion(btVector3(0, 0, 1), dir - M_PI_2));

    btVector3 dribblerInertia(0, 0, 0);
    dribblerShape->calculateLocalInertia((1 - robotMassProportion) * m_specs.mass(),
                                         dribblerInertia);
    btRigidBody::btRigidBodyConstructionInfo rbDribInfo(
        (1 - robotMassProportion) * m_specs.mass(), nullptr, dribblerShape,
        dribblerInertia);
    rbDribInfo.m_startWorldTransform = dribblerStartTransform;

    btRigidBody *dribblerBody = new btRigidBody(rbDribInfo);
    dribblerBody->setRestitution(0.2f);
    dribblerBody->setFriction(1.5f);
    m_dribblerBody = dribblerBody;
    m_world->addRigidBody(dribblerBody);

    btTransform localA, localB;
    localA.setIdentity();
    localB.setIdentity();
    localA.setOrigin(m_dribblerCenter);
    localA.setRotation(btQuaternion(btVector3(0, 1, 0), M_PI_2));
    localB.setRotation(btQuaternion(btVector3(0, 1, 0), M_PI_2));
    m_dribblerConstraint = new btHingeConstraint(*m_body, *dribblerBody, localA, localB);
    m_dribblerConstraint->enableAngularMotor(false, 0, 0);
    m_world->addConstraint(m_dribblerConstraint, true);
}

SimRobot::~SimRobot()
{
    if (m_holdBallConstraint)
    {
        m_world->removeConstraint(m_holdBallConstraint.get());
    }
    m_world->removeConstraint(m_dribblerConstraint);
    m_world->removeRigidBody(m_dribblerBody);
    m_world->removeRigidBody(m_body);
    delete m_dribblerConstraint;
    delete m_body;
    delete m_dribblerBody;
    delete m_motionState;
    qDeleteAll(m_shapes);
}

void SimRobot::calculateDribblerMove(const btVector3 pos, const btQuaternion rot,
                                     const btVector3 linVel, float omega)
{
    const btQuaternion rotated = rot *
                                 btQuaternion(m_dribblerCenter.x(), m_dribblerCenter.y(),
                                              m_dribblerCenter.z(), 0) *
                                 rot.inverse();
    const btVector3 dribblerPos =
        btVector3(rotated.x(), rotated.y(), rotated.z()) + pos * SIMULATOR_SCALE;
    const btVector3 dribblerDirection(1, 0, 0);
    const btQuaternion dribblerDirectionRot(dribblerDirection, 0);
    const btQuaternion newDribblerRot = rot * dribblerDirectionRot;
    m_dribblerBody->setWorldTransform(btTransform(newDribblerRot, dribblerPos));
    m_dribblerBody->setLinearVelocity(linVel + newDribblerRot.getAxis() * (-omega));
    m_dribblerBody->setAngularVelocity(btVector3(0, 0, 0));
}

void SimRobot::dribble(SimBall *ball, float speed)
{
    if (m_perfectDribbler)
    {
        if (canKickBall(ball) && !m_holdBallConstraint)
        {
            btTransform localA, localB;
            localA.setIdentity();
            localB.setIdentity();

            auto worldToRobot = m_body->getWorldTransform().inverse();
            localA.setOrigin(worldToRobot * ball->position());
            localA.setRotation(btQuaternion(worldToRobot * btVector3(0, 1, 0), M_PI_2));
            localB.setRotation(btQuaternion(worldToRobot * btVector3(0, 1, 0), M_PI_2));

            m_holdBallConstraint.reset(
                new btHingeConstraint(*m_body, *ball->body(), localA, localB));
            m_world->addConstraint(m_holdBallConstraint.get(), true);
        }
    }
    else
    {
        // unit for rotation is  (rad / s) in bullet, but (rpm) in sslCommand
        const float max_rotation_speed = 150.f / 2 / M_PI * 60;
        float dribbler                 = speed / max_rotation_speed;
        // rad/s is limited to 150
        float boundedDribbler = qBound(0.0f, dribbler, 1.0f);
        m_dribblerConstraint->enableAngularMotor(true, 150 * boundedDribbler,
                                                 20 * boundedDribbler);
    }
}

void SimRobot::stopDribbling()
{
    m_dribblerConstraint->enableAngularMotor(false, 0, 0);

    if (m_holdBallConstraint)
    {
        m_world->removeConstraint(m_holdBallConstraint.get());
        m_holdBallConstraint.reset();
    }
}

void SimRobot::setDribbleMode(bool perfectDribbler)
{
    if (m_perfectDribbler && !perfectDribbler)
    {
        stopDribbling();
    }
    m_perfectDribbler = perfectDribbler;
}

void SimRobot::begin(SimBall *ball, double time)
{
    m_commandTime += time;
    m_inStandby = false;
    // m_inStandby = m_command.standby();

    // after 0.1s without new command reset to stop
    if (m_commandTime > 0.1)
    {
        m_sslCommand.Clear();
        // the real robot switches to standby after a short delay
        m_inStandby = true;
    }

    // enable dribbler if necessary
    if (!m_inStandby && m_sslCommand.has_dribbler_speed() &&
        m_sslCommand.dribbler_speed() > 0)
    {
        dribble(ball, m_sslCommand.dribbler_speed());
    }
    else
    {
        stopDribbling();
    }

    auto sendPartialCoordError = [this](const std::string &msg) {
        SSLSimError error{new sslsim::SimulatorError};
        error->set_code("PARTIAL_COORD");
        std::string message = "Partial coordinates are not implemented yet";
        error->set_message(message + msg);
        emit this->sendSSLSimError(error, ErrorSource::CONFIG);
        if (!m_move.has_by_force() || !m_move.by_force())
        {
            m_move.Clear();
        }
    };
    bool moveCommand    = false;
    std::string message = " for robot (";
    message += std::to_string(m_specs.id());
    message += ')';

    if (m_move.has_x())
    {
        if (!m_move.has_y() || (!m_move.has_orientation() && !m_move.by_force()))
        {
            sendPartialCoordError(message + " position ");
            return;
        }
        else
        {
            moveCommand = true;
        }
    }
    else if (m_move.has_y())
    {
        sendPartialCoordError(message + " position (no x)");
        return;
    }

    if (m_move.has_v_x())
    {
        if (!m_move.has_v_y())
        {
            sendPartialCoordError(message + " velocity");
            return;
        }
        moveCommand = true;
    }
    else if (m_move.has_v_y())
    {
        sendPartialCoordError(message + " velocity (no x)");
        return;
    }

    if (m_move.has_v_angular())
    {
        moveCommand = true;
    }

    if (m_move.by_force())
    {
        bool sendError = false;
        if (m_move.has_v_x())
        {
            sendError = m_move.v_x() != 0 || m_move.v_y() != 0;
        }
        if (m_move.has_v_angular())
        {
            sendError |= m_move.v_angular() != 0;
        }
        if (sendError)
        {
            SSLSimError error{new sslsim::SimulatorError};
            error->set_code("VELOCITY_FORCE");
            error->set_message("Velocities != 0 and by_force are incompatible");
            emit sendSSLSimError(error, ErrorSource::CONFIG);
            return;
        }
    }  // TODO: check for force and orientation

    if (moveCommand)
    {
        if (m_move.by_force())
        {
            // move robot by hand
            btVector3 force;
            coordinates::fromVision(m_move, force);
            force.setZ(0.0f);
            force = force - m_body->getWorldTransform().getOrigin() / SIMULATOR_SCALE;
            force.setZ(0.0f);
            m_body->activate();
            m_body->applyCentralImpulse(force * m_specs.mass() * (1. / 6) *
                                        SIMULATOR_SCALE);
            m_body->setDamping(0.99, 0.99);
        }
        else
        {
            btVector3 pos;
            btVector3 linVel;
            btQuaternion rot;
            float angular;
            // set the robot to the given position and speed
            if (m_move.has_x())
            {
                coordinates::fromVision(m_move, pos);
                pos.setZ(m_specs.height() / 2.0f);
                rot = btQuaternion(
                    btVector3(0, 0, 1),
                    coordinates::fromVisionRotation(m_move.orientation()) - M_PI_2);
                m_body->setWorldTransform(btTransform(rot, pos * SIMULATOR_SCALE));
            }
            else
            {
                const auto &transform = m_body->getWorldTransform();
                rot                   = transform.getRotation();
                pos                   = transform.getOrigin();
            }
            if (m_move.has_v_x())
            {
                coordinates::fromVisionVelocity(m_move, linVel);
                linVel.setZ(0.0f);
                m_body->setLinearVelocity(linVel * SIMULATOR_SCALE);
            }
            else
            {
                linVel = m_body->getLinearVelocity();
            }
            if (m_move.has_v_angular())
            {
                const btVector3 angVel(m_move.v_angular(), 0.0f, 0.0f);
                m_body->setAngularVelocity(angVel);
                angular = angVel[0];
            }
            else
            {
                angular = m_body->getAngularVelocity()[0];
            }

            calculateDribblerMove(pos, rot, linVel, angular);

            m_body->activate();
            m_dribblerBody->activate();
            m_body->setDamping(0.0, 0.0);
            m_dribblerBody->setDamping(0.0, 0.0);
            m_move.Clear();  // clear move command
                             // reset is neccessary, as the command is only sent once
                             // without one canceling it
        }
        return;
    }

    m_body->setDamping(0.7, 0.8);

    btTransform t = m_body->getWorldTransform();
    t.setOrigin(btVector3(0, 0, 0));

    // charge kicker only if enabled
    if (!m_inStandby && m_charge)
    {
        m_shootTime += time;
        // recharge only after a short timeout, to prevent kick the ball twice
        if (!m_isCharged && m_shootTime > 0.1)
        {
            m_isCharged = true;
        }
    }
    else
    {
        m_isCharged = false;
        m_shootTime = 0.0;
    }
    // check if should kick and can do that
    if (m_isCharged && m_sslCommand.has_kick_speed() && m_sslCommand.kick_speed() > 0 &&
        canKickBall(ball))
    {
        float power          = 0.0;
        const float angle    = m_sslCommand.kick_angle() / 180 * M_PI;
        const float dirFloor = std::cos(angle);
        const float dirUp    = std::sin(angle);

        stopDribbling();

        if (m_sslCommand.kick_angle() == 0)
        {
            power = qBound(0.05f, m_sslCommand.kick_speed(), m_specs.shot_linear_max());
        }
        else
        {
            // FIXME: for now we just recalc the max distance based on the given angle
            const float maxShootSpeed =
                coordinates::chipVelFromChipDistance(m_specs.shot_chip_max());
            power = qBound(0.05f, m_sslCommand.kick_speed(), maxShootSpeed);
        }

        const auto getSpeedCompensation = [&]() -> float {
            if (m_sslCommand.kick_angle() == 0)
            {
                return 0.0f;
            }
            else
            {
                // if the ball hits the robot the chip distance actually decreases
                const btVector3 relBallSpeed = relativeBallSpeed(ball) / SIMULATOR_SCALE;
                return std::max((btScalar)0, relBallSpeed.y()) -
                       qBound((btScalar)0, (btScalar)0.5 * relBallSpeed.y(),
                              (btScalar)0.5 * dirFloor);
            }
        };
        const float speedCompensation = getSpeedCompensation();

        ball->kick(t * btVector3(0, dirFloor * power + speedCompensation, dirUp * power) *
                   (1 / time) * SIMULATOR_SCALE * BALL_MASS);
        // discharge
        m_isCharged = false;
        m_shootTime = 0.0;
    }

    if (m_inStandby || !m_sslCommand.has_move_command() ||
        !m_sslCommand.move_command().has_local_velocity())
    {
        return;
    }

    float output_v_f   = m_sslCommand.move_command().local_velocity().forward();
    float output_v_s   = -m_sslCommand.move_command().local_velocity().left();
    float output_omega = m_sslCommand.move_command().local_velocity().angular();

    btVector3 v_local(t.inverse() * m_body->getLinearVelocity());
    btVector3 v_d_local(boundSpeed(output_v_s), boundSpeed(output_v_f), 0);

    float v_f   = v_local.y() / SIMULATOR_SCALE;
    float v_s   = v_local.x() / SIMULATOR_SCALE;
    float omega = m_body->getAngularVelocity().z();

    const float error_v_s   = v_d_local.x() - v_s;
    const float error_v_f   = v_d_local.y() - v_f;
    const float error_omega = boundSpeed(output_omega) - omega;

    error_sum_v_s += error_v_s;
    error_sum_v_f += error_v_f;
    error_sum_omega += error_omega;

    const float error_sum_limit = 20.0f;
    error_sum_v_s = qBound(-error_sum_limit, error_sum_v_s, error_sum_limit);
    error_sum_v_f = qBound(-error_sum_limit, error_sum_v_f, error_sum_limit);

    // (1-(1-linear_damping)^timestep)/timestep - compensates damping
    const float V = 1.200f;             // keep current speed
    const float K = 1.f / time * 0.5f;  // correct half the error during each subtimestep
    const float K_I = /*0.1; //*/ 0.f;

    // as a certain part of the acceleration is required to compensate damping,
    // the robot will run into a speed limit! bound acceleration the speed limit
    // is acceleration * accelScale / V
    float a_f = V * v_f + K * error_v_f + K_I * error_sum_v_f;
    float a_s = V * v_s + K * error_v_s + K_I * error_sum_v_s;

    const float accelScale =
        2.f;  // let robot accelerate / brake faster than the accelerator does
    a_f = bound(a_f, v_f, accelScale * m_specs.strategy().a_speedup_f_max(),
                accelScale * m_specs.strategy().a_brake_f_max());
    a_s = bound(a_s, v_s, accelScale * m_specs.strategy().a_speedup_s_max(),
                accelScale * m_specs.strategy().a_brake_s_max());
    const btVector3 force(a_s * m_specs.mass(), a_f * m_specs.mass(), 0);

    // localInertia.z() / SIMULATOR_SCALE^2 \approx
    // 1/12*mass*(robot_width^2+robot_depth^2)
    // (1-(1-angular_damping)^timestep)/timestep *
    // localInertia.z()/SIMULATOR_SCALE^2 - compensates damping
    const float V_phi = 1.603f;  // keep current rotation
    const float K_phi =
        1.f / time * 0.5f;  // correct half the error during each subtimestep
    const float K_I_phi = /*0*0.2/1000; //*/ 0.f;

    const float a_phi = V_phi * omega + K_phi * error_omega + K_I_phi * error_sum_omega;
    const float a_phi_bound =
        bound(a_phi, omega, accelScale * m_specs.strategy().a_speedup_phi_max(),
              accelScale * m_specs.strategy().a_brake_phi_max());
    const btVector3 torque(0, 0, a_phi_bound * 0.007884f);

    if (force.length2() > 0 || torque.length2() > 0)
    {
        m_body->activate();
        m_body->applyCentralForce(t * force * SIMULATOR_SCALE);
        m_body->applyTorque(torque * SIMULATOR_SCALE * SIMULATOR_SCALE);
    }
}

// copy-paste from accelerator
float SimRobot::bound(float acceleration, float oldSpeed, float speedupLimit,
                      float brakeLimit) const
{
    // In case the robot needs to gain speed
    if ((std::signbit(acceleration) == std::signbit(oldSpeed)) || (oldSpeed == 0))
    {
        // the acceleration needs to be bounded with values for speeding up.
        return qBound(-speedupLimit, acceleration, speedupLimit);
    }
    else
    {
        // bound braking acceleration, in order to avoid fallover
        return qBound(-brakeLimit, acceleration, brakeLimit);
    }
}

btVector3 SimRobot::relativeBallSpeed(SimBall *ball) const
{
    btTransform t             = m_body->getWorldTransform();
    const btVector3 ballSpeed = ball->speed();

    const btQuaternion robotDir = t.getRotation();
    const btVector3 diff = (ballSpeed).rotate(robotDir.getAxis(), -robotDir.getAngle());

    return diff;
}

bool SimRobot::canKickBall(SimBall *ball) const
{
    const btVector3 ballPos = ball->position();
    // can't kick jumping ball
    if (ballPos.z() > 0.05f * SIMULATOR_SCALE)
    {
        return false;
    }

    if (m_holdBallConstraint)
    {
        return true;
    }

    // check for collision between ball and dribbler
    int numManifolds = m_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i)
    {
        btPersistentManifold *contactManifold =
            m_world->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject *objectA = (btCollisionObject *)(contactManifold->getBody0());
        btCollisionObject *objectB = (btCollisionObject *)(contactManifold->getBody1());
        if ((objectA == m_dribblerBody && objectB == ball->body()) ||
            (objectA == ball->body() && objectB == m_dribblerBody))
        {
            int numContacts = contactManifold->getNumContacts();
            for (int j = 0; j < numContacts; ++j)
            {
                btManifoldPoint &pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.001f * SIMULATOR_SCALE)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

robot::RadioResponse SimRobot::setCommand(const SSLSimulationProto::RobotCommand &command,
                                          SimBall *ball, bool charge, float rxLoss,
                                          float txLoss)
{
    m_sslCommand  = command;
    m_commandTime = 0.0f;
    m_charge      = charge;

    robot::RadioResponse response;
    response.set_generation(m_specs.generation());
    response.set_id(m_specs.id());
    response.set_battery(1);
    // TODO: actually compute the packet loss
    response.set_packet_loss_rx(rxLoss);
    response.set_packet_loss_tx(txLoss);
    response.set_ball_detected(canKickBall(ball));
    response.set_cap_charged(m_isCharged);

    // current velocities
    btTransform t = m_body->getWorldTransform();
    t.setOrigin(btVector3(0, 0, 0));
    btVector3 v_local(t.inverse() * m_body->getLinearVelocity());
    float v_f   = v_local.y() / SIMULATOR_SCALE;
    float v_s   = v_local.x() / SIMULATOR_SCALE;
    float omega = m_body->getAngularVelocity().z();

    robot::SpeedStatus *speedStatus = response.mutable_estimated_speed();
    speedStatus->set_v_f(v_f);
    speedStatus->set_v_s(v_s);
    speedStatus->set_omega(omega);

    return response;
}

void SimRobot::update(SSLProto::SSL_DetectionRobot *robot, float stddev_p,
                      float stddev_phi, qint64 time, btVector3 positionOffset)
{
    // setup vision packet
    robot->set_robot_id(m_specs.id());
    robot->set_confidence(1.0);
    robot->set_pixel_x(0);
    robot->set_pixel_y(0);

    // add noise
    btTransform transform;
    m_motionState->getWorldTransform(transform);
    const btVector3 p = transform.getOrigin() / SIMULATOR_SCALE + positionOffset;
    const ErForceVector p_noise = m_rng->normalVector(stddev_p);
    robot->set_x((p.y() + p_noise.x) * 1000.0f);
    robot->set_y(-(p.x() + p_noise.y) * 1000.0f);

    const btQuaternion q = transform.getRotation();
    const btVector3 dir  = btMatrix3x3(q).getColumn(0);
    robot->set_orientation(atan2(dir.y(), dir.x()) + m_rng->normal(stddev_phi));

    m_lastSendTime = time;
}

bool SimRobot::touchesBall(SimBall *ball) const
{
    // for some reason btHingeConstraints, which is used when dribbling, are not always
    // detected as contact by bullet. so if the ball is being dribbled then we assume it
    // is in contact with the robot.
    if (m_holdBallConstraint)
    {
        return true;
    }

    // iterate through contact manifold: a cache that contains all contact points between
    // pairs of collision objects
    int num_manifolds = m_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < num_manifolds; ++i)
    {
        btPersistentManifold *contact_manifold =
            m_world->getDispatcher()->getManifoldByIndexInternal(i);

        // determine if the two objects are the ball and robot body/dribbler
        btCollisionObject *objectA = (btCollisionObject *)(contact_manifold->getBody0());
        btCollisionObject *objectB = (btCollisionObject *)(contact_manifold->getBody1());
        if ((objectA == m_dribblerBody && objectB == ball->body()) ||
            (objectA == ball->body() && objectB == m_dribblerBody) ||
            (objectA == m_body && objectB == ball->body()) ||
            (objectA == ball->body() && objectB == m_body))
        {


            // check if the points are in contact now
            int num_contacts = contact_manifold->getNumContacts();
            auto contact_points = std::vector<btManifoldPoint>();
            for (int j = 0; j < num_contacts; ++j)
            {
                btManifoldPoint &pt = contact_manifold->getContactPoint(j);
                contact_points.push_back(pt);
            }

            if(std::any_of(contact_points.begin(), contact_points.end(), [=](auto &pt){
                return (pt.getDistance() < 0.001f * SIMULATOR_SCALE);
            })){
                return true;
            }
        }
    }

    return false;
}

void SimRobot::update(world::SimRobot *robot, SimBall *ball) const
{
    btTransform transform;
    m_motionState->getWorldTransform(transform);
    const btVector3 position = transform.getOrigin() / SIMULATOR_SCALE;
    robot->set_p_x(position.x());
    robot->set_p_y(position.y());
    robot->set_p_z(position.z());
    robot->set_id(m_specs.id());

    const btQuaternion q = transform.getRotation();
    auto *rotation       = robot->mutable_rotation();
    rotation->set_real(q.getX());
    rotation->set_i(q.getY());
    rotation->set_j(q.getZ());
    rotation->set_k(q.getW());
    robot->set_angle(q.getAngle());

    const btVector3 velocity = m_body->getLinearVelocity() / SIMULATOR_SCALE;
    robot->set_v_x(velocity.x());
    robot->set_v_y(velocity.y());
    robot->set_v_z(velocity.z());

    const btVector3 angular = m_body->getAngularVelocity();
    robot->set_r_x(angular.x());
    robot->set_r_y(angular.y());
    robot->set_r_z(angular.z());

    bool ballTouchesRobot = false;
    int numManifolds      = m_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i)
    {
        btPersistentManifold *contactManifold =
            m_world->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject *objectA = (btCollisionObject *)(contactManifold->getBody0());
        btCollisionObject *objectB = (btCollisionObject *)(contactManifold->getBody1());
        if ((objectA == m_dribblerBody && objectB == ball->body()) ||
            (objectA == ball->body() && objectB == m_dribblerBody) ||
            (objectA == m_body && objectB == ball->body()) ||
            (objectA == ball->body() && objectB == m_body))
        {
            ballTouchesRobot = true;
        }
    }
    robot->set_touches_ball(ballTouchesRobot);
}

void SimRobot::restoreState(const world::SimRobot &robot)
{
    btVector3 position(robot.p_x(), robot.p_y(), robot.p_z());
    m_body->getWorldTransform().setOrigin(position * SIMULATOR_SCALE);
    btQuaternion rotation(robot.rotation().real(), robot.rotation().i(),
                          robot.rotation().j(), robot.rotation().k());
    m_body->getWorldTransform().setRotation(rotation);
    btVector3 velocity(robot.v_x(), robot.v_y(), robot.v_z());
    m_body->setLinearVelocity(velocity * SIMULATOR_SCALE);
    btVector3 angular(robot.r_x(), robot.r_y(), robot.r_z());
    m_body->setAngularVelocity(angular);
}

void SimRobot::move(const sslsim::TeleportRobot &robot)
{
    m_move = robot;
}

bool SimRobot::isFlipped()
{
    btTransform t = m_body->getWorldTransform();
    bool isNan    = std::isnan(t.getOrigin().x()) || std::isnan(t.getOrigin().y()) ||
                 std::isnan(t.getOrigin().z()) || std::isinf(t.getOrigin().x()) ||
                 std::isinf(t.getOrigin().y()) || std::isinf(t.getOrigin().z());
    t.setOrigin(btVector3(0, 0, 0));
    return ((t * btVector3(0, 0, 1)).z() < 0) || isNan;
}

btVector3 SimRobot::position() const
{
    const btTransform transform = m_body->getWorldTransform();
    return btVector3(transform.getOrigin().x(), transform.getOrigin().y(), 0);
}

btVector3 SimRobot::dribblerCorner(bool left) const
{
    const btVector3 sideOffset =
        btVector3(m_specs.dribbler_width() / 2, 0, 0) * SIMULATOR_SCALE;
    const btVector3 corner = m_dribblerCenter + btVector3(0, 0.03, 0) * SIMULATOR_SCALE +
                             (left ? -sideOffset : sideOffset);
    const btTransform transform = m_body->getWorldTransform();
    return transform * corner;
}
