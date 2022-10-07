/***************************************************************************
 *   Copyright 2020 Michael Eischer, Philipp Nordhus, Andreas Wendler      *
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

#include "simulator.h"

#include <QtCore/QPair>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QtDebug>
#include <algorithm>
#include <functional>

#include "erroraggregator.h"
#include "extlibs/er_force_sim/src/core/coordinates.h"
#include "extlibs/er_force_sim/src/core/rng.h"
#include "extlibs/er_force_sim/src/protobuf/geometry.h"
#include "simball.h"
#include "simfield.h"
#include "simrobot.h"


using namespace camun::simulator;

/* Friction and restitution between robots, ball and field: (empirical
 * measurments) Ball vs. Robot: Restitution: about 0.60 Friction: trial and
 * error in simulator 0.18 (similar results as in reality)
 *
 * Ball vs. Floor:
 * Restitution: sqrt(h'/h) = sqrt(0.314) = 0.56
 * Friction: \mu_k = -a / g (while slipping) = 0.35
 *
 * Robot vs. Floor:
 * Restitution and Friction should be as low as possible
 *
 * Calculations:
 * Variables: r: restitution, f: friction
 * Indices: b: ball; f: floor; r: robot
 *
 * r_b * r_f = 0.56
 * r_b * r_r = 0.60
 * r_f * r_r = small
 * => r_b = 1; r_f = 0.56; r_r = 0.60
 *
 * f_b * f_f = 0.35
 * f_b * f_r = 0.22
 * f_f * f_r = very small
 * => f_b = 1; f_f = 0.35; f_r = 0.22
 */

struct camun::simulator::SimulatorData
{
    RNG rng;
    btDefaultCollisionConfiguration *collision;
    btCollisionDispatcher *dispatcher;
    btBroadphaseInterface *overlapping_pair_cache;
    btSequentialImpulseConstraintSolver *solver;
    btDiscreteDynamicsWorld *dynamics_world;
    world::Geometry geometry;
    QVector<SSLProto::SSL_GeometryCameraCalibration> reported_camera_setup;
    QVector<btVector3> camera_positions;
    SimField *field;
    SimBall *ball;
    Simulator::RobotMap robots_blue;
    Simulator::RobotMap robots_yellow;
    QMap<uint32_t, robot::Specs> specs_blue;
    QMap<uint32_t, robot::Specs> specs_yellow;
    bool flip;
    float stddev_ball;
    float stddev_ball_area;
    float stddev_robot;
    float stddev_robot_phi;
    float ball_detections_at_dribbler;  // per robot per second
    bool enable_invisible_ball;
    float ball_visibility_threshold;
    float camera_overlap;
    float camera_position_error;
    float object_position_offset;
    float robot_command_packet_loss;
    float robot_reply_packet_loss;
    float missing_ball_detections;
    bool dribble_perfect;
};

static void simulatorTickCallback(btDynamicsWorld *world, btScalar time_step)
{
    Simulator *sim = reinterpret_cast<Simulator *>(world->getWorldUserInfo());
    sim->handleSimulatorTick(time_step);
}

/*!
 * \class Simulator
 * \ingroup simulator
 * \brief %Simulator interface
 */

Simulator::Simulator(const amun::SimulatorSetup &setup)
    : m_time(0),
      m_last_sent_status_time(0),
      m_enabled(false),
      m_charge(true),
      m_vision_delay(35 * 1000 * 1000),
      m_vision_processing_time(5 * 1000 * 1000),
      m_aggregator(new ErrorAggregator(this))
{
    // setup bullet
    m_data                       = new SimulatorData;
    m_data->collision            = new btDefaultCollisionConfiguration();
    m_data->dispatcher           = new btCollisionDispatcher(m_data->collision);
    m_data->overlapping_pair_cache = new btDbvtBroadphase();
    m_data->solver               = new btSequentialImpulseConstraintSolver;
    m_data->dynamics_world =
        new btDiscreteDynamicsWorld(m_data->dispatcher, m_data->overlapping_pair_cache,
                                    m_data->solver, m_data->collision);
    m_data->dynamics_world->setGravity(btVector3(0.0f, 0.0f, -9.81f * SIMULATOR_SCALE));
    m_data->dynamics_world->setInternalTickCallback(simulatorTickCallback, this, true);

    m_data->geometry.CopyFrom(setup.geometry());
    for (const auto &camera : setup.camera_setup())
    {
        m_data->reported_camera_setup.append(camera);
        ErForceVector vision_position(camera.derived_camera_world_tx(),
                                      camera.derived_camera_world_ty());
        btVector3 true_position;
        coordinates::fromVision(vision_position, true_position);
        true_position.setZ(camera.derived_camera_world_tz() / 1000.0f);
        m_data->camera_positions.append(true_position);
    }

    // add field and ball
    m_data->field = new SimField(m_data->dynamics_world, m_data->geometry);
    m_data->ball  = new SimBall(&m_data->rng, m_data->dynamics_world);
    connect(m_data->ball, &SimBall::sendSSLSimError, m_aggregator,
            &ErrorAggregator::aggregate);
    m_data->flip                     = false;
    m_data->stddev_ball               = 0.0f;
    m_data->stddev_ball_area           = 0.0f;
    m_data->stddev_robot              = 0.0f;
    m_data->stddev_robot_phi           = 0.0f;
    m_data->ball_detections_at_dribbler = 0.0f;
    m_data->enable_invisible_ball      = true;
    m_data->ball_visibility_threshold  = 0.4;
    m_data->camera_overlap            = 0.3;
    m_data->camera_position_error      = 0;
    m_data->object_position_offset     = 0;
    m_data->robot_command_packet_loss   = 0;
    m_data->robot_reply_packet_loss     = 0;
    m_data->missing_ball_detections    = 0;
    m_data->dribble_perfect           = false;

    // no robots after initialisation
}

// does delete all Simrobots in the RobotMap, does not clear map
// (just like qDeleteAll would)
static void deleteAll(const Simulator::RobotMap &map)
{
    for (const auto &e : map)
    {
        delete e.first;
    }
}

Simulator::~Simulator()
{
    resetVisionPackets();

    deleteAll(m_data->robots_blue);
    deleteAll(m_data->robots_yellow);
    delete m_data->ball;
    delete m_data->field;
    delete m_data->dynamics_world;
    delete m_data->solver;
    delete m_data->overlapping_pair_cache;
    delete m_data->dispatcher;
    delete m_data->collision;
    delete m_data;
}

std::vector<robot::RadioResponse> Simulator::acceptBlueRobotControlCommand(
    const SSLSimulationProto::RobotControl &control)
{
    return acceptRobotControlCommand(control, true);
}

std::vector<robot::RadioResponse> Simulator::acceptYellowRobotControlCommand(
    const SSLSimulationProto::RobotControl &control)
{
    return acceptRobotControlCommand(control, false);
}

std::vector<robot::RadioResponse> Simulator::acceptRobotControlCommand(
    const SSLSimulationProto::RobotControl &control, bool is_blue)
{
    // collect responses from robots
    std::vector<robot::RadioResponse> responses;

    for (const SSLSimulationProto::RobotCommand &command : control.robot_commands())
    {
        // pass radio command to robot that matches the id
        const auto id          = command.id();
        SimulatorData *data    = m_data;
        auto time              = m_time;
        auto charge            = m_charge;
        auto fabricate_response = [data, &responses, time, charge, &id, &command](
                                     const Simulator::RobotMap &map, const bool *is_blue) {
            if (!map.contains(id))
                return;
            robot::RadioResponse response = map[id].first->setCommand(
                command, data->ball, charge, data->robot_command_packet_loss,
                data->robot_reply_packet_loss);
            response.set_time(time);

            if (is_blue != nullptr)
            {
                response.set_is_blue(*is_blue);
            }
            // only collect valid responses
            if (response.IsInitialized())
            {
                if (data->robot_reply_packet_loss == 0 ||
                    data->rng.uniformFloat(0, 1) > data->robot_reply_packet_loss)
                {
                    responses.emplace_back(response);
                }
            }
        };
        if (is_blue)
        {
            fabricate_response(m_data->robots_blue, &is_blue);
        }
        else
        {
            fabricate_response(m_data->robots_yellow, &is_blue);
        }
    }
    return responses;
}

void Simulator::sendSSLSimErrorInternal(ErrorSource source)
{
    QList<SSLSimError> errors = m_aggregator->getAggregates(source);
    if (errors.size() == 0)
        return;
    emit sendSSLSimError(errors, source);
}

static void createRobot(Simulator::RobotMap &list, float x, float y, uint32_t id,
                        const ErrorAggregator *agg, SimulatorData *data,
                        const QMap<uint32_t, robot::Specs> &teamSpecs)
{
    SimRobot *robot = new SimRobot(&data->rng, teamSpecs[id], data->dynamics_world,
                                   btVector3(x, y, 0), 0.f);
    robot->setDribbleMode(data->dribble_perfect);
    robot->connect(robot, &SimRobot::sendSSLSimError, agg, &ErrorAggregator::aggregate);
    list[id] = {robot, teamSpecs[id].generation()};
}

void Simulator::resetFlipped(Simulator::RobotMap &robots, float side)
{
    // find flipped robots and align them on a line
    const float x = m_data->geometry.field_width() / 2 - 0.2;
    float y       = m_data->geometry.field_height() / 2 - 0.2;

    for (RobotMap::iterator it = robots.begin(); it != robots.end(); ++it)
    {
        SimRobot *robot = it.value().first;
        if (robot->isFlipped())
        {
            SimRobot *new_robot =
                new SimRobot(&m_data->rng, robot->specs(), m_data->dynamics_world,
                             btVector3(x, side * y, 0), 0.0f);
            delete robot;
            connect(new_robot, &SimRobot::sendSSLSimError, m_aggregator,
                    &ErrorAggregator::aggregate);  // TODO? use createRobot instead of
                                                   // this. However, doing so naively
                                                   // will break the iteration, so I
                                                   // left it for now.
            new_robot->setDribbleMode(m_data->dribble_perfect);
            it.value().first = new_robot;
        }
        y -= 0.3;
    }
}

void Simulator::stepSimulation(double time_s)
{
    m_data->dynamics_world->stepSimulation(time_s, 10, SUB_TIMESTEP);
    m_time += time_s * 1E9;
}

void Simulator::handleSimulatorTick(double time_s)
{
    // has to be done according to bullet wiki
    m_data->dynamics_world->clearForces();

    resetFlipped(m_data->robots_blue, 1.0f);
    resetFlipped(m_data->robots_yellow, -1.0f);
    if (m_data->ball->isInvalid())
    {
        delete m_data->ball;
        m_data->ball = new SimBall(&m_data->rng, m_data->dynamics_world);
        connect(m_data->ball, &SimBall::sendSSLSimError, m_aggregator,
                &ErrorAggregator::aggregate);
    }

    // find out if ball and any robot collide
    auto robot_ball_collision = [this](QPair<SimRobot *, unsigned int> elem) {
        return elem.first->touchesBall(this->m_data->ball);
    };

    bool ball_collision = std::any_of(m_data->robots_blue.begin(),
                                      m_data->robots_blue.end(), robot_ball_collision) ||
                          std::any_of(m_data->robots_yellow.begin(),
                                      m_data->robots_yellow.end(), robot_ball_collision);

    // apply commands and forces to ball and robots
    m_data->ball->begin(ball_collision);
    for (const auto &pair : m_data->robots_blue)
    {
        pair.first->begin(m_data->ball, time_s);
    }
    for (const auto &pair : m_data->robots_yellow)
    {
        pair.first->begin(m_data->ball, time_s);
    }

    // add gravity to all ACTIVE objects
    // thus has to be done after applying commands
    m_data->dynamics_world->applyGravity();
}

static bool checkCameraID(const int camera_id, const btVector3 &p,
                          const QVector<btVector3> &camera_positions, const float overlap)
{
    float min_distance = std::numeric_limits<float>::max();
    float own_distance = 0;
    for (int i = 0; i < camera_positions.size(); i++)
    {
        // manhattan distance for rectangular camera regions (if the cameras are
        // distributed normally)
        float distance = std::abs(camera_positions[i].x() - p.x()) +
                         std::abs(camera_positions[i].y() - p.y());
        min_distance = std::min(min_distance, distance);
        if (i == camera_id)
        {
            own_distance = distance;
        }
    }
    return own_distance <= min_distance + 2 * overlap;
}

void Simulator::initializeDetection(SSLProto::SSL_DetectionFrame *detection,
                                    std::size_t camera_id)
{
    detection->set_frame_number(m_last_frame_number[camera_id]++);
    detection->set_camera_id(camera_id);
    detection->set_t_capture((m_time + m_vision_delay - m_vision_processing_time) * 1E-9);
    detection->set_t_sent((m_time + m_vision_delay) * 1E-9);
}

static btVector3 positionOffsetForCamera(float offset_strength, btVector3 camera_pos)
{
    btVector3 cam2d{camera_pos.x(), camera_pos.y(), 0};
    if (offset_strength < 1e-9)
    {
        // do not produce an offset that tiny
        return {0, 0, 0};
    }
    if (cam2d.length() < offset_strength)
    {
        // do not normalize a 0 vector
        return cam2d;
    }
    return btVector3(camera_pos.x(), camera_pos.y(), 0).normalized() * offset_strength;
}

std::vector<SSLProto::SSL_WrapperPacket> Simulator::getWrapperPackets()
{
    const std::size_t num_cameras = m_data->reported_camera_setup.size();

    std::vector<SSLProto::SSL_DetectionFrame> detections(num_cameras);
    for (std::size_t i = 0; i < num_cameras; i++)
    {
        initializeDetection(&detections[i], i);
    }

    bool missing_ball = m_data->missing_ball_detections > 0 &&
                       m_data->rng.uniformFloat(0, 1) <= m_data->missing_ball_detections;
    const btVector3 ball_position = m_data->ball->position() / SIMULATOR_SCALE;
    if (m_time - m_last_ball_send_time >= m_min_ball_detection_time && !missing_ball)
    {
        m_last_ball_send_time = m_time;

        for (std::size_t camera_id = 0; camera_id < num_cameras; ++camera_id)
        {
            // at least one id is always valid
            if (!checkCameraID(camera_id, ball_position, m_data->camera_positions,
                               m_data->camera_overlap))
            {
                continue;
            }

            // get ball position
            const btVector3 position_offset = positionOffsetForCamera(
                    m_data->object_position_offset, m_data->camera_positions[camera_id]);
            bool visible = m_data->ball->update(
                    detections[camera_id].add_balls(), m_data->stddev_ball,
                    m_data->stddev_ball_area, m_data->camera_positions[camera_id],
                    m_data->enable_invisible_ball, m_data->ball_visibility_threshold,
                    position_offset);
            if (!visible)
            {
                detections[camera_id].clear_balls();
            }
        }
    }

    // get robot positions
    for (bool team_is_blue : {true, false})
    {
        auto &team = team_is_blue ? m_data->robots_blue : m_data->robots_yellow;

        for (const auto &it : team)
        {
            SimRobot *robot = it.first;

            if (m_time - robot->getLastSendTime() >= m_min_robot_detection_time)
            {
                const float time_diff     = (m_time - robot->getLastSendTime()) * 1E-9;
                const btVector3 robot_pos = robot->position() / SIMULATOR_SCALE;

                for (std::size_t camera_id = 0; camera_id < num_cameras; ++camera_id)
                {
                    if (!checkCameraID(camera_id, robot_pos, m_data->camera_positions,
                                       m_data->camera_overlap))
                    {
                        continue;
                    }

                    const btVector3 position_offset = positionOffsetForCamera(
                            m_data->object_position_offset, m_data->camera_positions[camera_id]);
                    if (team_is_blue)
                    {
                        robot->update(detections[camera_id].add_robots_blue(),
                                      m_data->stddev_robot, m_data->stddev_robot_phi, m_time,
                                      position_offset);
                    }
                    else
                    {
                        robot->update(detections[camera_id].add_robots_yellow(),
                                      m_data->stddev_robot, m_data->stddev_robot_phi, m_time,
                                      position_offset);
                    }

                    // once in a while, add a ball mis-detection at a corner of the
                    // dribbler in real games, this happens because the ball detection
                    // light beam used by many teams is red
                    float detection_prob = time_diff * m_data->ball_detections_at_dribbler;
                    if (m_data->ball_detections_at_dribbler > 0 &&
                        m_data->rng.uniformFloat(0, 1) < detection_prob)
                    {
                        // always on the right side of the dribbler for now
                        if (!m_data->ball->addDetection(
                                detections[camera_id].add_balls(),
                                robot->dribblerCorner(false) / SIMULATOR_SCALE,
                                m_data->stddev_robot, 0, m_data->camera_positions[camera_id],
                                false, 0, position_offset))
                        {
                            detections[camera_id].mutable_balls()->DeleteSubrange(
                                    detections[camera_id].balls_size() - 1, 1);
                        }
                    }
                }
            }
        }
    }

    std::vector<SSLProto::SSL_WrapperPacket> packets;
    packets.reserve(num_cameras);

    // add a wrapper packet for all detections (also for empty ones).
    // The reason is that other teams might rely on the fact that these detections
    // are in regular intervals.
    for (auto &frame : detections)
    {
        // if multiple balls are reported, shuffle them randomly (the tracking might
        // have systematic errors depending on the ball order)
        if (frame.balls_size() > 1)
        {
            std::shuffle(frame.mutable_balls()->begin(), frame.mutable_balls()->end(),
                         rand_shuffle_src);
        }

        SSLProto::SSL_WrapperPacket packet;
        packet.mutable_detection()->CopyFrom(frame);
        packets.push_back(packet);
    }

    // add field geometry
    if (packets.size() == 0)
    {
        packets.push_back(SSLProto::SSL_WrapperPacket());
    }
    SSLProto::SSL_GeometryData *geometry   = packets[0].mutable_geometry();
    SSLProto::SSL_GeometryFieldSize *field = geometry->mutable_field();
    convertToSSlGeometry(m_data->geometry, field);

    const btVector3 position_error_sim_scale =
        btVector3(0.3f, 0.7f, 0.05f).normalized() * m_data->camera_position_error;
    btVector3 position_error_vision_scale{0, 0, position_error_sim_scale.z() * 1000};
    coordinates::toVision(position_error_sim_scale, position_error_vision_scale);
    for (const auto &calibration : m_data->reported_camera_setup)
    {
        auto calib = geometry->add_calib();
        calib->CopyFrom(calibration);
        calib->set_derived_camera_world_tx(calib->derived_camera_world_tx() +
                                           position_error_vision_scale.x());
        calib->set_derived_camera_world_ty(calib->derived_camera_world_ty() +
                                           position_error_vision_scale.y());
        calib->set_derived_camera_world_tz(calib->derived_camera_world_tz() +
                                           position_error_vision_scale.z());
    }

    // add ball model to geometry data
    geometry->mutable_models()->mutable_straight_two_phase()->set_acc_roll(-0.35);
    geometry->mutable_models()->mutable_straight_two_phase()->set_acc_slide(-4.5);
    geometry->mutable_models()->mutable_straight_two_phase()->set_k_switch(0.69);
    geometry->mutable_models()->mutable_chip_fixed_loss()->set_damping_z(0.566);
    geometry->mutable_models()->mutable_chip_fixed_loss()->set_damping_xy_first_hop(
        0.715);
    geometry->mutable_models()->mutable_chip_fixed_loss()->set_damping_xy_other_hops(1);

    return packets;
}

world::SimulatorState Simulator::getSimulatorState()
{
    const std::size_t num_cameras = m_data->reported_camera_setup.size();
    world::SimulatorState sim_state;

    auto *ball = sim_state.mutable_ball();
    m_data->ball->writeBallState(ball);

    // get robot positions
    for (bool team_is_blue : {true, false})
    {
        auto &team = team_is_blue ? m_data->robots_blue : m_data->robots_yellow;

        for (const auto &it : team)
        {
            SimRobot *robot = it.first;

            // convert coordinates from ER Force
            btVector3 robot_pos = robot->position() / SIMULATOR_SCALE;
            btVector3 new_robot_pos;

            coordinates::toVision(robot_pos, new_robot_pos);

            auto *robot_proto =
                    team_is_blue ? sim_state.add_blue_robots() : sim_state.add_yellow_robots();

            robot->update(robot_proto, m_data->ball);

            // Convert mm to m
            robot_proto->set_p_x(new_robot_pos.x() / 1000);
            robot_proto->set_p_y(new_robot_pos.y() / 1000);

            // Convert velocity
            coordinates::toVisionVelocity(*robot_proto, *robot_proto);
            robot_proto->set_v_x(robot_proto->v_x() / 1000);
            robot_proto->set_v_y(robot_proto->v_y() / 1000);
        }
    }

    return sim_state;
}

void Simulator::resetVisionPackets()
{
    qDeleteAll(m_vision_timers);
    m_vision_timers.clear();
    m_vision_packets.clear();
}

void Simulator::handleRadioCommands(const SSLSimRobotControl &commands, bool is_blue,
                                    qint64 processing_start)
{
    m_radio_commands.enqueue(std::make_tuple(commands, processing_start, is_blue));
}

void Simulator::setTeam(Simulator::RobotMap &list, float side, const robot::Team &team,
                        QMap<uint32_t, robot::Specs> &team_specs)
{
    // remove old team
    deleteAll(list);
    list.clear();

    // changing a team is also triggering a tracking reset
    // thus the old robots will disappear immediatelly
    // however if the delayed vision packets arrive the old robots will be tracked
    // again thus after removing a robot from a team it can take 1 simulated
    // second for the robot to disappear to prevent this remove outdated vision
    // packets
    resetVisionPackets();

    // align robots on a line
    const float x = m_data->geometry.field_width() / 2 - 0.2;
    float y       = m_data->geometry.field_height() / 2 - 0.2;

    for (int i = 0; i < team.robot_size(); i++)
    {
        const robot::Specs &specs = team.robot(i);
        const auto id             = specs.id();

        // (color, robot id) must be unique
        if (list.contains(id))
        {
            std::cerr << "Error: Two ids for the same color, aborting!" << std::endl;
            continue;
        }
        team_specs[id].CopyFrom(specs);

        createRobot(list, x, side * y, id, m_aggregator, m_data, team_specs);
        y -= 0.3;
    }
}

#define FLIP(X, ATTR)                                                                    \
    do                                                                                   \
    {                                                                                    \
        if (X.has_##ATTR())                                                              \
        {                                                                                \
            X.set_##ATTR(-X.ATTR());                                                     \
        }                                                                                \
    } while (0)

void Simulator::moveBall(const sslsim::TeleportBall &ball)
{
    // remove the dribbling constraint
    if (!ball.has_by_force() || !ball.by_force())
    {
        for (const auto &robot_list : {m_data->robots_blue, m_data->robots_yellow})
        {
            for (const auto &it : robot_list)
            {
                it.first->stopDribbling();
            }
        }
    }

    sslsim::TeleportBall b = ball;
    if (m_data->flip)
    {
        FLIP(b, x);
        FLIP(b, y);
        FLIP(b, vx);
        FLIP(b, vy);
    }

    if (b.teleport_safely())
    {
        // Not handled
    }

    m_data->ball->move(b);
}

void Simulator::moveRobot(const sslsim::TeleportRobot &robot)
{
    if (!robot.id().has_team())
        return;

    if (!robot.id().has_id())
        return;

    bool is_blue = robot.id().team() == gameController::Team::BLUE;

    RobotMap &list = is_blue ? m_data->robots_blue : m_data->robots_yellow;
    bool is_present = list.contains(robot.id().id());
    QMap<uint32_t, robot::Specs> &team_specs =
            is_blue ? m_data->specs_blue : m_data->specs_yellow;

    if (robot.has_present())
    {
        if (robot.present() && !is_present)
        {
            // add the requested robot
            if (!team_specs.contains(robot.id().id()))
            {
                SSLSimError error{new sslsim::SimulatorError};
                error->set_code("CREATE_UNSPEC_ROBOT");
                std::string message =
                    "trying to create robot " + std::to_string(robot.id().id());
                message += ", but no spec for this robot was found";
                error->set_message(std::move(message));
                m_aggregator->aggregate(error, ErrorSource::CONFIG);
            }
            else if (!robot.has_x() || !robot.has_y())
            {
                SSLSimError error{new sslsim::SimulatorError};
                error->set_code("CREATE_NOPOS_ROBOT");
                std::string message =
                    "trying to create robot " + std::to_string(robot.id().id());
                message += " without giving a position";
                error->set_message(std::move(message));
                m_aggregator->aggregate(error, ErrorSource::CONFIG);
            }
            else
            {
                ErForceVector target_pos;
                coordinates::fromVision(robot, target_pos);
                // TODO: check if the given position is fine
                createRobot(list, target_pos.x, target_pos.y, robot.id().id(), m_aggregator,
                            m_data, team_specs);
            }
        }
        else if (!robot.present() && is_present)
        {
            // remove the robot
            auto val = list.take(robot.id().id());
            val.first->stopDribbling();
            delete val.first;
            return;
        }
        else if (!robot.present() && !is_present)
        {
            return;
        }
        // Fall though: If the robot is already on the field and needs to be on the
        // field, we just use that robot.
    }
    else
    {
        if (!is_present)
            return;
    }

    if (!list.contains(robot.id().id()))
        return;  // Recheck the list in case the has_present paragraph did change it.

    sslsim::TeleportRobot r = robot;

    if (m_data->flip)
    {
        FLIP(r, x);
        FLIP(r, y);
        FLIP(r, v_x);
        FLIP(r, v_y);
    }

    SimRobot *sim_robot = list[robot.id().id()].first;
    if (!r.has_by_force() || !r.by_force())
    {
        sim_robot->stopDribbling();
    }
    sim_robot->move(r);
}

void Simulator::setFlipped(bool flipped)
{
    m_data->flip = flipped;
}

void Simulator::handleSimulatorSetupCommand(const std::unique_ptr<amun::Command> &command)
{
    bool team_or_perfect_dribble_changed = false;

    if (command->has_simulator())
    {
        const amun::CommandSimulator &sim = command->simulator();
        if (sim.has_enable())
        {
            m_enabled = sim.enable();
        }

        if (sim.has_realism_config())
        {
            auto realism = sim.realism_config();
            if (realism.has_stddev_ball_p())
            {
                m_data->stddev_ball = realism.stddev_ball_p();
            }

            if (realism.has_stddev_robot_p())
            {
                m_data->stddev_robot = realism.stddev_robot_p();
            }

            if (realism.has_stddev_robot_phi())
            {
                m_data->stddev_robot_phi = realism.stddev_robot_phi();
            }

            if (realism.has_stddev_ball_area())
            {
                m_data->stddev_ball_area = realism.stddev_ball_area();
            }

            if (realism.has_dribbler_ball_detections())
            {
                m_data->ball_detections_at_dribbler = realism.dribbler_ball_detections();
            }

            if (realism.has_enable_invisible_ball())
            {
                m_data->enable_invisible_ball = realism.enable_invisible_ball();
            }

            if (realism.has_ball_visibility_threshold())
            {
                m_data->ball_visibility_threshold = realism.ball_visibility_threshold();
            }

            if (realism.has_camera_overlap())
            {
                m_data->camera_overlap = realism.camera_overlap();
            }

            if (realism.has_camera_position_error())
            {
                m_data->camera_position_error = realism.camera_position_error();
            }

            if (realism.has_object_position_offset())
            {
                m_data->object_position_offset = realism.object_position_offset();
            }

            if (realism.has_robot_command_loss())
            {
                m_data->robot_command_packet_loss = realism.robot_command_loss();
            }

            if (realism.has_robot_response_loss())
            {
                m_data->robot_reply_packet_loss = realism.robot_response_loss();
            }

            if (realism.has_missing_ball_detections())
            {
                m_data->missing_ball_detections = realism.missing_ball_detections();
            }

            if (realism.has_vision_delay())
            {
                m_vision_delay = std::max((qint64)0, (qint64)realism.vision_delay());
            }

            if (realism.has_vision_processing_time())
            {
                m_vision_processing_time =
                    std::max((qint64)0, (qint64)realism.vision_processing_time());
            }

            if (realism.has_simulate_dribbling())
            {
                m_data->dribble_perfect      = !realism.simulate_dribbling();
                team_or_perfect_dribble_changed = true;
            }
        }

        if (sim.has_ssl_control())
        {
            const auto &ssl_control = sim.ssl_control();
            if (ssl_control.has_teleport_ball())
            {
                moveBall(ssl_control.teleport_ball());
            }
            for (const auto &moveR : ssl_control.teleport_robot())
            {
                moveRobot(moveR);
            }
        }

        if (sim.has_vision_worst_case())
        {
            if (sim.vision_worst_case().has_min_ball_detection_time())
            {
                m_min_ball_detection_time =
                    sim.vision_worst_case().min_ball_detection_time() * 1E9;
            }
            if (sim.vision_worst_case().has_min_robot_detection_time())
            {
                m_min_robot_detection_time =
                    sim.vision_worst_case().min_robot_detection_time() * 1E9;
            }
        }

        if (sim.has_set_simulator_state())
        {
            if (sim.set_simulator_state().has_ball())
            {
                m_data->ball->restoreState(sim.set_simulator_state().ball());
            }
            const auto restore_robots = [](RobotMap &map, auto robots) {
                for (const auto &robot : robots)
                {
                    if (map.contains(robot.id()))
                    {
                        map[robot.id()].first->restoreState(robot);
                    }
                }
            };
            restore_robots(m_data->robots_yellow,
                           sim.set_simulator_state().yellow_robots());
            restore_robots(m_data->robots_blue, sim.set_simulator_state().blue_robots());
        }
    }

    if (command->has_transceiver())
    {
        const amun::CommandTransceiver &t = command->transceiver();
        if (t.has_charge())
        {
            m_charge = t.charge();
        }
    }

    if (command->has_set_team_blue())
    {
        team_or_perfect_dribble_changed = true;
        setTeam(m_data->robots_blue, 1.0f, command->set_team_blue(), m_data->specs_blue);
    }

    if (command->has_set_team_yellow())
    {
        team_or_perfect_dribble_changed = true;
        setTeam(m_data->robots_yellow, -1.0f, command->set_team_yellow(),
                m_data->specs_yellow);
    }

    if (team_or_perfect_dribble_changed)
    {
        for (const auto &robot_list : {m_data->robots_blue, m_data->robots_yellow})
        {
            for (const auto &it : robot_list)
            {
                SimRobot *robot = it.first;
                robot->setDribbleMode(m_data->dribble_perfect);
            }
        }
    }
}

void Simulator::seedPRGN(uint32_t seed)
{
    m_data->rng.seed(seed);
}

static bool overlapCheck(const btVector3 &p0, const float &r0, const btVector3 &p1,
                         const float &r1)
{
    const float distance = (p1 - p0).length();
    return distance <= r0 + r1;
}

// uses the real world scale
void Simulator::teleportRobotToFreePosition(SimRobot *robot)
{
    btVector3 robot_pos = robot->position() / SIMULATOR_SCALE;
    btVector3 direction =
        (robot_pos - m_data->ball->position() / SIMULATOR_SCALE).normalize();
    float distance = 2 * (BALL_RADIUS + robot->specs().radius());
    bool valid     = true;
    do
    {
        valid    = true;
        robot_pos = robot_pos + 2 * direction * distance;

        for (const auto &robot_list : {m_data->robots_blue, m_data->robots_yellow})
        {
            for (const auto &it : robot_list)
            {
                SimRobot *robot2 = it.first;
                if (robot == robot2)
                {
                    continue;
                }

                btVector3 tmp = robot2->position() / SIMULATOR_SCALE;
                if (overlapCheck(robot_pos, robot->specs().radius(), tmp,
                                 robot2->specs().radius()))
                {
                    valid = false;
                    break;
                }
            }
            if (!valid)
            {
                break;
            }
        }
    } while (!valid);

    sslsim::TeleportRobot robot_command;
    robot_command.mutable_id()->set_id(robot->specs().id());
    coordinates::toVision(robot_pos, robot_command);

    robot_command.set_v_x(0);
    robot_command.set_v_y(0);
    robot->move(robot_command);
}
