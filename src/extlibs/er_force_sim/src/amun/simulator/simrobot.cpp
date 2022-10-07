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
      m_is_charged(false),
      m_in_standby(false),
      m_shoot_time(0.0),
      m_command_time(0.0),
      error_sum_v_s(0),
      error_sum_v_f(0),
      error_sum_omega(0)
{
    btCompoundShape *whole_shape = new btCompoundShape;
    btTransform robot_shape_transform;
    robot_shape_transform.setIdentity();

    // subtract collision margin from dimensions
    Mesh mesh(m_specs.radius() - COLLISION_MARGIN / SIMULATOR_SCALE,
              m_specs.height() - 2 * COLLISION_MARGIN / SIMULATOR_SCALE, m_specs.angle(),
              0.04f, m_specs.dribbler_height() + 0.02f);
    for (const QList<QVector3D> &hull_part : mesh.hull())
    {
        btConvexHullShape *hull_part_shape = new btConvexHullShape;
        m_shapes.append(hull_part_shape);
        for (const QVector3D &v : hull_part)
        {
            hull_part_shape->addPoint(btVector3(v.x(), v.y(), v.z()) * SIMULATOR_SCALE);
        }
        whole_shape->addChildShape(robot_shape_transform, hull_part_shape);
    }
    m_shapes.append(whole_shape);

    btTransform start_world_transform;
    start_world_transform.setIdentity();
    btVector3 robot_base_pos(btVector3(pos.x(), pos.y(), m_specs.height() / 2.0f) *
                             SIMULATOR_SCALE);
    start_world_transform.setOrigin(robot_base_pos);
    start_world_transform.setRotation(btQuaternion(btVector3(0, 0, 1), dir - M_PI_2));
    m_motion_state = new btDefaultMotionState(start_world_transform);

    // set robot dynamics and move to start position
    btVector3 local_inertia(0, 0, 0);
    const float robot_mass_proportion = 49.0f / 50.0f;
    whole_shape->calculateLocalInertia(robot_mass_proportion * m_specs.mass(),
                                       local_inertia);
    btRigidBody::btRigidBodyConstructionInfo rb_info(
        robot_mass_proportion * m_specs.mass(), m_motion_state, whole_shape,
        local_inertia);

    m_body = new btRigidBody(rb_info);
    // see simulator.cpp
    m_body->setRestitution(0.6f);
    m_body->setFriction(0.22f);
    m_world->addRigidBody(m_body);

    btCylinderShape *dribbler_shape = new btCylinderShapeX(
        btVector3(m_specs.dribbler_width() / 2.0f, 0.007f, 0.007f) * SIMULATOR_SCALE);
    m_shapes.append(dribbler_shape);
    // WARNING: hack, instead of 0.02 should be the dribbler height
    // the ball seems to get instable if the dribbler is at correct height
    // possibly the ball gets 'sucked' onto the robot
    m_dribbler_center = btVector3(
        btVector3(0, m_specs.shoot_radius() - 0.01f, -m_specs.height() / 2.0f + 0.02f) *
        SIMULATOR_SCALE);
    btTransform dribbler_start_transform;
    dribbler_start_transform.setIdentity();
    dribbler_start_transform.setOrigin(m_dribbler_center + robot_base_pos);
    dribbler_start_transform.setRotation(btQuaternion(btVector3(0, 0, 1), dir - M_PI_2));

    btVector3 dribbler_inertia(0, 0, 0);
    dribbler_shape->calculateLocalInertia((1 - robot_mass_proportion) * m_specs.mass(),
                                          dribbler_inertia);
    btRigidBody::btRigidBodyConstructionInfo rb_drib_info(
        (1 - robot_mass_proportion) * m_specs.mass(), nullptr, dribbler_shape,
        dribbler_inertia);
    rb_drib_info.m_startWorldTransform = dribbler_start_transform;

    btRigidBody *dribbler_body = new btRigidBody(rb_drib_info);
    dribbler_body->setRestitution(0.2f);
    dribbler_body->setFriction(1.5f);
    m_dribbler_body = dribbler_body;
    m_world->addRigidBody(dribbler_body);

    btTransform local_a, local_b;
    local_a.setIdentity();
    local_b.setIdentity();
    local_a.setOrigin(m_dribbler_center);
    local_a.setRotation(btQuaternion(btVector3(0, 1, 0), M_PI_2));
    local_b.setRotation(btQuaternion(btVector3(0, 1, 0), M_PI_2));
    m_dribbler_constraint =
        new btHingeConstraint(*m_body, *dribbler_body, local_a, local_b);
    m_dribbler_constraint->enableAngularMotor(false, 0, 0);
    m_world->addConstraint(m_dribbler_constraint, true);
}

SimRobot::~SimRobot()
{
    if (m_hold_ball_constraint)
    {
        m_world->removeConstraint(m_hold_ball_constraint.get());
    }
    m_world->removeConstraint(m_dribbler_constraint);
    m_world->removeRigidBody(m_dribbler_body);
    m_world->removeRigidBody(m_body);
    delete m_dribbler_constraint;
    delete m_body;
    delete m_dribbler_body;
    delete m_motion_state;
    qDeleteAll(m_shapes);
}

void SimRobot::calculateDribblerMove(const btVector3 pos, const btQuaternion rot,
                                     const btVector3 lin_vel, float omega)
{
    const btQuaternion rotated =
        rot *
        btQuaternion(m_dribbler_center.x(), m_dribbler_center.y(), m_dribbler_center.z(),
                     0) *
        rot.inverse();
    const btVector3 dribbler_pos =
        btVector3(rotated.x(), rotated.y(), rotated.z()) + pos * SIMULATOR_SCALE;
    const btVector3 dribbler_direction(1, 0, 0);
    const btQuaternion dribbler_direction_rot(dribbler_direction, 0);
    const btQuaternion new_dribbler_rot = rot * dribbler_direction_rot;
    m_dribbler_body->setWorldTransform(btTransform(new_dribbler_rot, dribbler_pos));
    m_dribbler_body->setLinearVelocity(lin_vel + new_dribbler_rot.getAxis() * (-omega));
    m_dribbler_body->setAngularVelocity(btVector3(0, 0, 0));
}

void SimRobot::dribble(SimBall *ball, float speed)
{
    if (m_perfect_dribbler)
    {
        if (canKickBall(ball) && !m_hold_ball_constraint)
        {
            btTransform local_a, local_b;
            local_a.setIdentity();
            local_b.setIdentity();

            auto world_to_robot = m_body->getWorldTransform().inverse();
            local_a.setOrigin(world_to_robot * ball->position());
            local_a.setRotation(
                btQuaternion(world_to_robot * btVector3(0, 1, 0), M_PI_2));
            local_b.setRotation(
                btQuaternion(world_to_robot * btVector3(0, 1, 0), M_PI_2));

            m_hold_ball_constraint.reset(
                new btHingeConstraint(*m_body, *ball->body(), local_a, local_b));
            m_world->addConstraint(m_hold_ball_constraint.get(), true);
        }
    }
    else
    {
        // unit for rotation is  (rad / s) in bullet, but (rpm) in sslCommand
        const float max_rotation_speed = 150.f / 2 / M_PI * 60;
        float dribbler                 = speed / max_rotation_speed;
        // rad/s is limited to 150
        float bounded_dribbler = qBound(0.0f, dribbler, 1.0f);
        m_dribbler_constraint->enableAngularMotor(true, 150 * bounded_dribbler,
                                                  20 * bounded_dribbler);
    }
}

void SimRobot::stopDribbling()
{
    m_dribbler_constraint->enableAngularMotor(false, 0, 0);

    if (m_hold_ball_constraint)
    {
        m_world->removeConstraint(m_hold_ball_constraint.get());
        m_hold_ball_constraint.reset();
    }
}

void SimRobot::setDribbleMode(bool perfect_dribbler)
{
    if (m_perfect_dribbler && !perfect_dribbler)
    {
        stopDribbling();
    }
    m_perfect_dribbler = perfect_dribbler;
}

void SimRobot::begin(SimBall *ball, double time)
{
    m_command_time += time;
    m_in_standby = false;
    // m_in_standby = m_command.standby();

    // after 0.1s without new command reset to stop
    if (m_command_time > 0.1)
    {
        m_ssl_command.Clear();
        // the real robot switches to standby after a short delay
        m_in_standby = true;
    }

    // enable dribbler if necessary
    if (!m_in_standby && m_ssl_command.has_dribbler_speed() &&
        m_ssl_command.dribbler_speed() > 0)
    {
        dribble(ball, m_ssl_command.dribbler_speed());
    }
    else
    {
        stopDribbling();
    }

    auto send_partial_coord_error = [this](const std::string &msg) {
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
    bool move_command   = false;
    std::string message = " for robot (";
    message += std::to_string(m_specs.id());
    message += ')';

    if (m_move.has_x())
    {
        if (!m_move.has_y() || (!m_move.has_orientation() && !m_move.by_force()))
        {
            send_partial_coord_error(message + " position ");
            return;
        }
        else
        {
            move_command = true;
        }
    }
    else if (m_move.has_y())
    {
        send_partial_coord_error(message + " position (no x)");
        return;
    }

    if (m_move.has_v_x())
    {
        if (!m_move.has_v_y())
        {
            send_partial_coord_error(message + " velocity");
            return;
        }
        move_command = true;
    }
    else if (m_move.has_v_y())
    {
        send_partial_coord_error(message + " velocity (no x)");
        return;
    }

    if (m_move.has_v_angular())
    {
        move_command = true;
    }

    if (m_move.by_force())
    {
        bool send_error = false;
        if (m_move.has_v_x())
        {
            send_error = m_move.v_x() != 0 || m_move.v_y() != 0;
        }
        if (m_move.has_v_angular())
        {
            send_error |= m_move.v_angular() != 0;
        }
        if (send_error)
        {
            SSLSimError error{new sslsim::SimulatorError};
            error->set_code("VELOCITY_FORCE");
            error->set_message("Velocities != 0 and by_force are incompatible");
            emit sendSSLSimError(error, ErrorSource::CONFIG);
            return;
        }
    }  // TODO: check for force and orientation

    if (move_command)
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
            btVector3 lin_vel;
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
                coordinates::fromVisionVelocity(m_move, lin_vel);
                lin_vel.setZ(0.0f);
                m_body->setLinearVelocity(lin_vel * SIMULATOR_SCALE);
            }
            else
            {
                lin_vel = m_body->getLinearVelocity();
            }
            if (m_move.has_v_angular())
            {
                const btVector3 ang_vel(m_move.v_angular(), 0.0f, 0.0f);
                m_body->setAngularVelocity(ang_vel);
                angular = ang_vel[0];
            }
            else
            {
                angular = m_body->getAngularVelocity()[0];
            }

            calculateDribblerMove(pos, rot, lin_vel, angular);

            m_body->activate();
            m_dribbler_body->activate();
            m_body->setDamping(0.0, 0.0);
            m_dribbler_body->setDamping(0.0, 0.0);
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
    if (!m_in_standby && m_charge)
    {
        m_shoot_time += time;
        // recharge only after a short timeout, to prevent kick the ball twice
        if (!m_is_charged && m_shoot_time > 0.1)
        {
            m_is_charged = true;
        }
    }
    else
    {
        m_is_charged = false;
        m_shoot_time = 0.0;
    }
    // check if should kick and can do that
    if (m_is_charged && m_ssl_command.has_kick_speed() &&
        m_ssl_command.kick_speed() > 0 && canKickBall(ball))
    {
        float power           = 0.0;
        const float angle     = m_ssl_command.kick_angle() / 180 * M_PI;
        const float dir_floor = std::cos(angle);
        const float dir_up    = std::sin(angle);

        stopDribbling();

        if (m_ssl_command.kick_angle() == 0)
        {
            power = qBound(0.05f, m_ssl_command.kick_speed(), m_specs.shot_linear_max());
        }
        else
        {
            // FIXME: for now we just recalc the max distance based on the given angle
            const float max_shoot_speed =
                coordinates::chipVelFromChipDistance(m_specs.shot_chip_max());
            power = qBound(0.05f, m_ssl_command.kick_speed(), max_shoot_speed);
        }

        const auto get_speed_compensation = [&]() -> float {
            if (m_ssl_command.kick_angle() == 0)
            {
                return 0.0f;
            }
            else
            {
                // if the ball hits the robot the chip distance actually decreases
                const btVector3 rel_ball_speed =
                    relativeBallSpeed(ball) / SIMULATOR_SCALE;
                return std::max((btScalar)0, rel_ball_speed.y()) -
                       qBound((btScalar)0, (btScalar)0.5 * rel_ball_speed.y(),
                              (btScalar)0.5 * dir_floor);
            }
        };
        const float speed_compensation = get_speed_compensation();

        ball->kick(t *
                   btVector3(0, dir_floor * power + speed_compensation, dir_up * power) *
                   (1 / time) * SIMULATOR_SCALE * BALL_MASS);
        // discharge
        m_is_charged = false;
        m_shoot_time = 0.0;
    }

    if (m_in_standby || !m_ssl_command.has_move_command() ||
        !m_ssl_command.move_command().has_local_velocity())
    {
        return;
    }

    float output_v_f   = m_ssl_command.move_command().local_velocity().forward();
    float output_v_s   = -m_ssl_command.move_command().local_velocity().left();
    float output_omega = m_ssl_command.move_command().local_velocity().angular();

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
    // is acceleration * accel_scale / V
    float a_f = V * v_f + K * error_v_f + K_I * error_sum_v_f;
    float a_s = V * v_s + K * error_v_s + K_I * error_sum_v_s;

    const float accel_scale =
        2.f;  // let robot accelerate / brake faster than the accelerator does
    a_f = bound(a_f, v_f, accel_scale * m_specs.strategy().a_speedup_f_max(),
                accel_scale * m_specs.strategy().a_brake_f_max());
    a_s = bound(a_s, v_s, accel_scale * m_specs.strategy().a_speedup_s_max(),
                accel_scale * m_specs.strategy().a_brake_s_max());
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
        bound(a_phi, omega, accel_scale * m_specs.strategy().a_speedup_phi_max(),
              accel_scale * m_specs.strategy().a_brake_phi_max());
    const btVector3 torque(0, 0, a_phi_bound * 0.007884f);

    if (force.length2() > 0 || torque.length2() > 0)
    {
        m_body->activate();
        m_body->applyCentralForce(t * force * SIMULATOR_SCALE);
        m_body->applyTorque(torque * SIMULATOR_SCALE * SIMULATOR_SCALE);
    }
}

// copy-paste from accelerator
float SimRobot::bound(float acceleration, float old_speed, float speedup_limit,
                      float brake_limit) const
{
    // In case the robot needs to gain speed
    if ((std::signbit(acceleration) == std::signbit(old_speed)) || (old_speed == 0))
    {
        // the acceleration needs to be bounded with values for speeding up.
        return qBound(-speedup_limit, acceleration, speedup_limit);
    }
    else
    {
        // bound braking acceleration, in order to avoid fallover
        return qBound(-brake_limit, acceleration, brake_limit);
    }
}

btVector3 SimRobot::relativeBallSpeed(SimBall *ball) const
{
    btTransform t              = m_body->getWorldTransform();
    const btVector3 ball_speed = ball->speed();

    const btQuaternion robot_dir = t.getRotation();
    const btVector3 diff =
        (ball_speed).rotate(robot_dir.getAxis(), -robot_dir.getAngle());

    return diff;
}

bool SimRobot::canKickBall(SimBall *ball) const
{
    const btVector3 ball_pos = ball->position();
    // can't kick jumping ball
    if (ball_pos.z() > 0.05f * SIMULATOR_SCALE)
    {
        return false;
    }

    if (m_hold_ball_constraint)
    {
        return true;
    }

    // check for collision between ball and dribbler
    int num_manifolds = m_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < num_manifolds; ++i)
    {
        btPersistentManifold *contact_manifold =
            m_world->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject *object_a = (btCollisionObject *)(contact_manifold->getBody0());
        btCollisionObject *object_b = (btCollisionObject *)(contact_manifold->getBody1());
        if ((object_a == m_dribbler_body && object_b == ball->body()) ||
            (object_a == ball->body() && object_b == m_dribbler_body))
        {
            int num_contacts = contact_manifold->getNumContacts();
            for (int j = 0; j < num_contacts; ++j)
            {
                btManifoldPoint &pt = contact_manifold->getContactPoint(j);
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
                                          SimBall *ball, bool charge, float rx_loss,
                                          float tx_loss)
{
    m_ssl_command  = command;
    m_command_time = 0.0f;
    m_charge       = charge;

    robot::RadioResponse response;
    response.set_generation(m_specs.generation());
    response.set_id(m_specs.id());
    response.set_battery(1);
    // TODO: actually compute the packet loss
    response.set_packet_loss_rx(rx_loss);
    response.set_packet_loss_tx(tx_loss);
    response.set_ball_detected(canKickBall(ball));
    response.set_cap_charged(m_is_charged);

    // current velocities
    btTransform t = m_body->getWorldTransform();
    t.setOrigin(btVector3(0, 0, 0));
    btVector3 v_local(t.inverse() * m_body->getLinearVelocity());
    float v_f   = v_local.y() / SIMULATOR_SCALE;
    float v_s   = v_local.x() / SIMULATOR_SCALE;
    float omega = m_body->getAngularVelocity().z();

    robot::SpeedStatus *speed_status = response.mutable_estimated_speed();
    speed_status->set_v_f(v_f);
    speed_status->set_v_s(v_s);
    speed_status->set_omega(omega);

    return response;
}

void SimRobot::update(SSLProto::SSL_DetectionRobot *robot, float stddev_p,
                      float stddev_phi, qint64 time, btVector3 position_offset)
{
    // setup vision packet
    robot->set_robot_id(m_specs.id());
    robot->set_confidence(1.0);
    robot->set_pixel_x(0);
    robot->set_pixel_y(0);

    // add noise
    btTransform transform;
    m_motion_state->getWorldTransform(transform);
    const btVector3 p = transform.getOrigin() / SIMULATOR_SCALE + position_offset;
    const ErForceVector p_noise = m_rng->normalVector(stddev_p);
    robot->set_x((p.y() + p_noise.x) * 1000.0f);
    robot->set_y(-(p.x() + p_noise.y) * 1000.0f);

    const btQuaternion q = transform.getRotation();
    const btVector3 dir  = btMatrix3x3(q).getColumn(0);
    robot->set_orientation(atan2(dir.y(), dir.x()) + m_rng->normal(stddev_phi));

    m_last_send_time = time;
}

bool SimRobot::touchesBall(SimBall *ball) const
{
    // for some reason btHingeConstraints, which is used when dribbling, are not always
    // detected as contact by bullet. so if the ball is being dribbled then we assume it
    // is in contact with the robot.
    if (m_hold_ball_constraint)
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
        btCollisionObject *object_a = (btCollisionObject *)(contact_manifold->getBody0());
        btCollisionObject *object_b = (btCollisionObject *)(contact_manifold->getBody1());
        if ((object_a == m_dribbler_body && object_b == ball->body()) ||
            (object_a == ball->body() && object_b == m_dribbler_body) ||
            (object_a == m_body && object_b == ball->body()) ||
            (object_a == ball->body() && object_b == m_body))
        {
            // check if the points are in contact now
            int num_contacts = contact_manifold->getNumContacts();
            for (int j = 0; j < num_contacts; ++j)
            {
                btManifoldPoint &pt = contact_manifold->getContactPoint(j);
                if (pt.getDistance() < 0.001f * SIMULATOR_SCALE)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

void SimRobot::update(world::SimRobot *robot, SimBall *ball) const
{
    btTransform transform;
    m_motion_state->getWorldTransform(transform);
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

    bool ball_touches_robot = false;
    int num_manifolds       = m_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < num_manifolds; ++i)
    {
        btPersistentManifold *contact_manifold =
            m_world->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject *object_a = (btCollisionObject *)(contact_manifold->getBody0());
        btCollisionObject *object_b = (btCollisionObject *)(contact_manifold->getBody1());
        if ((object_a == m_dribbler_body && object_b == ball->body()) ||
            (object_a == ball->body() && object_b == m_dribbler_body) ||
            (object_a == m_body && object_b == ball->body()) ||
            (object_a == ball->body() && object_b == m_body))
        {
            ball_touches_robot = true;
        }
    }
    robot->set_touches_ball(ball_touches_robot);
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
    bool is_nan   = std::isnan(t.getOrigin().x()) || std::isnan(t.getOrigin().y()) ||
                  std::isnan(t.getOrigin().z()) || std::isinf(t.getOrigin().x()) ||
                  std::isinf(t.getOrigin().y()) || std::isinf(t.getOrigin().z());
    t.setOrigin(btVector3(0, 0, 0));
    return ((t * btVector3(0, 0, 1)).z() < 0) || is_nan;
}

btVector3 SimRobot::position() const
{
    const btTransform transform = m_body->getWorldTransform();
    return btVector3(transform.getOrigin().x(), transform.getOrigin().y(), 0);
}

btVector3 SimRobot::dribblerCorner(bool left) const
{
    const btVector3 side_offset =
        btVector3(m_specs.dribbler_width() / 2, 0, 0) * SIMULATOR_SCALE;
    const btVector3 corner = m_dribbler_center + btVector3(0, 0.03, 0) * SIMULATOR_SCALE +
                             (left ? -side_offset : side_offset);
    const btTransform transform = m_body->getWorldTransform();
    return transform * corner;
}
