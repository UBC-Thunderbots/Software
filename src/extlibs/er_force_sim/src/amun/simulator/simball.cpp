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
#include "proto/ssl_vision_detection.pb.h"
#include "simulator.h"

using namespace camun::simulator;

SimBall::SimBall(RNG *rng, btDiscreteDynamicsWorld *world)
    : m_rng(rng),
      m_world(world),
      current_ball_state(STATIONARY),
      set_transition_speed(true),
      rolling_speed(-1.0)
{
    // see http://robocup.mi.fu-berlin.de/buch/rolling.pdf for correct modelling
    m_sphere = new btSphereShape(BALL_RADIUS * SIMULATOR_SCALE);

    btVector3 local_inertia(0, 0, 0);
    // FIXME measure inertia coefficient
    m_sphere->calculateLocalInertia(BALL_MASS, local_inertia);

    btTransform start_world_transform;
    start_world_transform.setIdentity();
    start_world_transform.setOrigin(btVector3(0, 0, BALL_RADIUS) * SIMULATOR_SCALE);
    m_motion_state = new btDefaultMotionState(start_world_transform);

    btRigidBody::btRigidBodyConstructionInfo rb_info(BALL_MASS, m_motion_state, m_sphere,
                                                     local_inertia);

    // parameters seem to be ignored...
    m_body = new btRigidBody(rb_info);
    // see simulator.cpp
    // TODO (#2512): Check these values with real life
    m_body->setRestitution(BALL_RESTITUTION);
    m_body->setFriction(BALL_SLIDING_FRICTION);

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
    delete m_motion_state;
}

void SimBall::begin(bool robot_collision)
{
    // custom implementation of rolling friction
    const btVector3 p        = m_body->getWorldTransform().getOrigin();
    const btVector3 velocity = m_body->getLinearVelocity();
    if (p.z() < BALL_RADIUS * 1.1 * SIMULATOR_SCALE)
    {  // ball is on the ground
        bool is_stationary = velocity.length() < STATIONARY_BALL_SPEED * SIMULATOR_SCALE;
        bool should_roll   = velocity.length() < rolling_speed * SIMULATOR_SCALE;

        if (robot_collision)
        {
            current_ball_state = ROBOT_COLLISION;
        }
        else if (is_stationary)
        {
            current_ball_state = STATIONARY;
        }
        else if ((current_ball_state == SLIDING && should_roll) ||
                 current_ball_state == ROLLING)
        {
            current_ball_state = ROLLING;
        }
        else
        {
            current_ball_state = SLIDING;
        }

        switch (current_ball_state)
        {
            case STATIONARY:
                m_body->setLinearVelocity(btVector3(0, 0, 0));
                m_body->setFriction(BALL_SLIDING_FRICTION);
                set_transition_speed = true;
                break;
            case ROBOT_COLLISION:
                m_body->setFriction(BALL_SLIDING_FRICTION);
                set_transition_speed = true;
                break;
            case SLIDING:
                m_body->setFriction(BALL_SLIDING_FRICTION);
                if (set_transition_speed)
                {
                    rolling_speed =
                        FRICTION_TRANSITION_FACTOR * velocity.length() / SIMULATOR_SCALE;
                }
                set_transition_speed = false;
                break;
            case ROLLING:

                // just apply rolling friction, normal friction is somehow handled by
                // bullet
                const btScalar rolling_deceleration = BALL_ROLLING_FRICTION_DECELERATION;
                btVector3 force(velocity.x(), velocity.y(), 0.0f);
                force.safeNormalize();
                m_body->applyCentralImpulse(-force * rolling_deceleration *
                                            SIMULATOR_SCALE * BALL_MASS * SUB_TIMESTEP);

                m_body->setFriction(0.0);
                set_transition_speed = false;
                break;
        }
    }

    bool move_command             = false;
    auto send_partial_coord_error = [this](const char *msg) {
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
            send_partial_coord_error(": position ball");
            return;
        }
        move_command = true;
    }
    else if (m_move.has_y() || m_move.has_z())
    {
        send_partial_coord_error(": position ball (not x)");
        return;
    }

    if (m_move.has_vx())
    {
        if (!m_move.has_vy())
        {
            send_partial_coord_error(": velocity ball");
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
        move_command = true;
    }

    if (move_command)
    {
        if (m_move.by_force())
        {
            ErForceVector pos;
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
                ErForceVector c_pos;
                coordinates::fromVision(m_move, c_pos);
                float height = BALL_RADIUS;
                if (m_move.has_z())
                {
                    height += m_move.z();
                }
                const btVector3 pos(c_pos.x, c_pos.y, height);
                btTransform transform = m_body->getWorldTransform();
                transform.setOrigin(pos * SIMULATOR_SCALE);
                m_body->setWorldTransform(transform);
            }
            if (m_move.has_vx())
            {
                ErForceVector vel;
                coordinates::fromVisionVelocity(m_move, vel);
                float vz = 0;
                if (m_move.has_vz())
                {
                    vz = m_move.vz() * 1e-3;
                }
                const btVector3 lin_vel(vel.x, vel.y, vz);

                m_body->setLinearVelocity(lin_vel * SIMULATOR_SCALE);

                // override ballState
                current_ball_state   = SLIDING;
                rolling_speed        = lin_vel.length() * FRICTION_TRANSITION_FACTOR;
                set_transition_speed = false;

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
static float positionOfVisiblePixels(btVector3 &p, const btVector3 &simulator_ball_position,
                                     const btVector3 &simulator_camera_position,
                                     const btCollisionWorld *const m_world)
{
    const float simulator_ball_radius = BALL_RADIUS * SIMULATOR_SCALE;

    // axis and angle for rotating the plane towards the camera
    btVector3 camera_direction =
        (simulator_camera_position - simulator_ball_position).normalize();

    const btVector3 up = btVector3(0, 0, 1);

    btVector3 axis = up.cross(camera_direction);
    if (!axis.fuzzyZero())
    {
        axis = axis.normalize();
    }
    btScalar angle = up.angle(camera_direction);

    const int sample_radius        = 7;
    std::size_t max_hits           = pow((2 * sample_radius + 1), 2);
    std::size_t camera_hit_counter = 0;

    btVector3 new_pos = btVector3(0, 0, 0);

    for (int x = -sample_radius; x <= sample_radius; ++x)
    {
        for (int y = -sample_radius; y <= sample_radius; ++y)
        {
            // create offset to the midpoint of the plane
            btVector3 offset(x, y, 0);
            offset *= simulator_ball_radius / sample_radius;

            // ignore samples outside the ball
            if (offset.length() >= simulator_ball_radius)
            {
                --max_hits;
                continue;
            }

            // rotate the plane/offset
            offset.rotate(axis, angle);
            btVector3 sample_point = simulator_ball_position + offset;

            btCollisionWorld::ClosestRayResultCallback sample_result(
                    sample_point, simulator_camera_position);
            m_world->rayTest(sample_point, simulator_camera_position, sample_result);

            if (!sample_result.hasHit())
            {
                // transform point back to plane with upwards normal to ensure unchanged
                // height
                sample_point -= simulator_ball_position;
                sample_point.rotate(axis, -angle);
                sample_point += simulator_ball_position;

                new_pos += sample_point;

                ++camera_hit_counter;
            }
        }
    }

    if (camera_hit_counter > 0)
    {
        // return average position of samples transformed to real world space
        new_pos /= camera_hit_counter;
        p = new_pos / SIMULATOR_SCALE;
    }

    return static_cast<float>(camera_hit_counter) / static_cast<float>(max_hits);
}

bool SimBall::update(SSLProto::SSL_DetectionBall *ball, float stddev, float stddev_area,
                     const btVector3 &camera_position, bool enable_invisible_ball,
                     float visibility_threshold, btVector3 position_offset)
{
    btTransform transform;
    m_motion_state->getWorldTransform(transform);
    btVector3 pos = transform.getOrigin() / SIMULATOR_SCALE;

    return addDetection(ball, pos, stddev, stddev_area, camera_position,
                        enable_invisible_ball, visibility_threshold, position_offset);
}

bool SimBall::addDetection(SSLProto::SSL_DetectionBall *ball, btVector3 pos, float stddev,
                           float stddev_area, const btVector3 &camera_position,
                           bool enable_invisible_ball, float visibility_threshold,
                           btVector3 position_offset)
{
    // setup ssl-vision ball detection
    ball->set_confidence(1.0);
    ball->set_pixel_x(0);
    ball->set_pixel_y(0);

    btTransform transform;
    m_motion_state->getWorldTransform(transform);

    const btVector3 simulator_camera_position =
            btVector3(camera_position.x(), camera_position.y(), camera_position.z()) *
            SIMULATOR_SCALE;

    float visibility = 1;
    // the camera uses the mid point of the visible pixels as the mid point of the
    // ball if some parts of the ball aren't visible the position this function
    // adjusts the position accordingly (hopefully)
    if (enable_invisible_ball)
    {
        // if the visibility is lower than the threshold the ball disappears
        visibility = positionOfVisiblePixels(pos, transform.getOrigin(),
                                             simulator_camera_position, m_world);
        if (visibility < visibility_threshold)
        {
            return false;
        }
    }

    const float SCALING_LIMIT = 0.9f;
    // reflects the cameras resolution
    const unsigned PIXEL_PER_AREA = 10;  // i do not know in what unit area is,
                                         // just make it similar to a real game

    float mod_z = std::min(SCALING_LIMIT * camera_position.z(),
                           std::max(0.f, pos.z() - BALL_RADIUS));
    float mod_x = (pos.x() - camera_position.x()) *
                  (camera_position.z() / (camera_position.z() - mod_z)) +
                  camera_position.x();
    float mod_y = (pos.y() - camera_position.y()) *
                  (camera_position.z() / (camera_position.z() - mod_z)) +
                  camera_position.y();

    float dist_ball_cam =
        std::sqrt((camera_position.z() - mod_z) * (camera_position.z() - mod_z) +
                  (camera_position.x() - pos.x()) * (camera_position.x() - pos.x()) +
                  (camera_position.y() - pos.y()) * (camera_position.y() - pos.y()));
    float denom_sqrt = (dist_ball_cam * 1000) / FOCAL_LENGTH - 1;
    float base_pixel_area =
        (BALL_RADIUS * BALL_RADIUS * 1000000 * M_PI) / (denom_sqrt * denom_sqrt);
    float area =
        visibility *
        std::max(0.0f, (base_pixel_area +
                        static_cast<float>(m_rng->normal(stddev_area)) / PIXEL_PER_AREA));
    ball->set_area(area * PIXEL_PER_AREA);

    // if (height > 0.1f) {
    //     qDebug() << "simball" << p.x() << p.y() << height << "ttt" << ball_x <<
    //     ball_y;
    // }

    // add noise to coordinates
    // to convert from bullet coordinate system to ssl-vision rotate by 90 degree
    // ccw
    const ErForceVector noise = m_rng->normalVector(stddev);
    coordinates::toVision(ErForceVector(mod_x, mod_y) + noise, *ball);
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
    const btVector3 ball_position =
        m_body->getWorldTransform().getOrigin() / SIMULATOR_SCALE;
    ball->set_p_x(ball_position.getX());
    ball->set_p_y(ball_position.getY());
    ball->set_p_z(ball_position.getZ());
    const btVector3 ball_speed = speed() / SIMULATOR_SCALE;
    ball->set_v_x(ball_speed.getX());
    ball->set_v_y(ball_speed.getY());
    ball->set_v_z(ball_speed.getZ());
    const btVector3 angular_velocity = m_body->getAngularVelocity();
    ball->set_angular_x(angular_velocity.x());
    ball->set_angular_y(angular_velocity.y());
    ball->set_angular_z(angular_velocity.z());
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
    bool is_nan =
        std::isnan(transform.getOrigin().x()) || std::isnan(transform.getOrigin().y()) ||
        std::isnan(transform.getOrigin().z()) || std::isinf(transform.getOrigin().x()) ||
        std::isinf(transform.getOrigin().y()) || std::isinf(transform.getOrigin().z()) ||
        std::isnan(velocity.x()) || std::isnan(velocity.y()) ||
        std::isnan(velocity.z()) || std::isinf(velocity.x()) ||
        std::isinf(velocity.y()) || std::isinf(velocity.z());
    bool is_below_field = (transform.getOrigin().z() <= 0);
    return is_nan || is_below_field;
}

void SimBall::kick(const btVector3 &power)
{
    m_body->activate();
    m_body->applyCentralForce(power);
    // btTransform transform;
    // m_motion_state->getWorldTransform(transform);
    // const btVector3 p = transform.getOrigin() / SIMULATOR_SCALE;
    // qDebug() << "kick at" << p.x() << p.y();
}
