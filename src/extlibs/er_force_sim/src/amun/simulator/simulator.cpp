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

#include <algorithm>
#include <array>
#include <functional>

#include "extlibs/er_force_sim/src/core/coordinates.h"
#include "extlibs/er_force_sim/src/protobuf/geometry.h"

using namespace camun::simulator;

Simulator::Simulator(const amun::SimulatorSetup& setup)
    : m_data(std::make_unique<SimulatorData>()),
      m_enabled(false),
      m_charge(true),
      m_time(0),
      m_visionDelay(35 * 1000 * 1000),
      m_visionProcessingTime(5 * 1000 * 1000)
{
    m_data->collision  = std::make_unique<btDefaultCollisionConfiguration>();
    m_data->dispatcher = std::make_unique<btCollisionDispatcher>(m_data->collision.get());
    m_data->overlappingPairCache = std::make_unique<btDbvtBroadphase>();
    m_data->solver        = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_data->dynamicsWorld = std::make_shared<btDiscreteDynamicsWorld>(
        m_data->dispatcher.get(), m_data->overlappingPairCache.get(),
        m_data->solver.get(), m_data->collision.get());
    m_data->dynamicsWorld->setGravity(btVector3(0.0f, 0.0f, -9.81f * SIMULATOR_SCALE));
    m_data->dynamicsWorld->setInternalTickCallback(
        [](btDynamicsWorld* world, btScalar timeStep)
        {
            Simulator* sim = reinterpret_cast<Simulator*>(world->getWorldUserInfo());
            sim->handleSimulatorTick(timeStep);
        },
        this, true);

    m_data->geometry.CopyFrom(setup.geometry());
    for (const auto& camera : setup.camera_setup())
    {
        m_data->reportedCameraSetup.push_back(camera);
        ErForceVector visionPosition(camera.derived_camera_world_tx(),
                                     camera.derived_camera_world_ty());
        btVector3 truePosition;
        coordinates::fromVision(visionPosition, truePosition);
        truePosition.setZ(camera.derived_camera_world_tz() / 1000.0f);
        m_data->cameraPositions.push_back(truePosition);
    }

    m_data->field = std::make_unique<SimField>(m_data->dynamicsWorld, m_data->geometry);
    m_data->ball  = std::make_shared<SimBall>(m_data->dynamicsWorld);
    m_data->flip  = false;
    m_data->stddevBall                  = 0.0f;
    m_data->stddevBallArea              = 0.0f;
    m_data->stddevRobot                 = 0.0f;
    m_data->stddevRobotPhi              = 0.0f;
    m_data->ballDetectionsAtDribbler    = 0.0f;
    m_data->enableInvisibleBall         = true;
    m_data->ballVisibilityThreshold     = 0.4;
    m_data->cameraOverlap               = 0.3;
    m_data->cameraPositionError         = 0;
    m_data->objectPositionOffset        = 0;
    m_data->robotCommandPacketLoss      = 0;
    m_data->robotReplyPacketLoss        = 0;
    m_data->missingBallDetections       = 0;
    m_data->dribblePerfect              = false;
    m_data->missingRobotDetections      = 0;
    m_data->commandDelay                = 0;
    m_data->robotRotationError          = 0;
    m_data->rotatedRobotDetectionsStart = 0;
    m_data->rotatedRobotDetectionsStop  = 0;

    // no robots after initialisation
}

std::vector<robot::RadioResponse> Simulator::acceptBlueRobotControlCommand(
    const SSLSimulationProto::RobotControl& control)
{
    return acceptRobotControlCommand(control, true);
}

std::vector<robot::RadioResponse> Simulator::acceptYellowRobotControlCommand(
    const SSLSimulationProto::RobotControl& control)
{
    return acceptRobotControlCommand(control, false);
}

std::vector<robot::RadioResponse> Simulator::acceptRobotControlCommand(
    const SSLSimulationProto::RobotControl& control, bool isBlue)
{
    auto& pendingCommands = isBlue ? m_blueRadioCommands : m_yellowRadioCommands;

    if (m_data->commandDelay <= 0 && pendingCommands.empty())
    {
        return applyRobotControlCommand(control, isBlue);
    }

    // The robots only receive the command after the command delay has passed. Note
    // that the radio responses of a command are reported when it is applied, so they
    // are delayed by the same amount.
    pendingCommands.emplace_back(control, m_time);

    std::vector<robot::RadioResponse> responses;
    while (!pendingCommands.empty() &&
           pendingCommands.front().second + m_data->commandDelay <= m_time)
    {
        const SSLSimulationProto::RobotControl pendingControl =
            pendingCommands.front().first;
        pendingCommands.pop_front();

        const std::vector<robot::RadioResponse> pendingResponses =
            applyRobotControlCommand(pendingControl, isBlue);
        responses.insert(responses.end(), pendingResponses.begin(),
                         pendingResponses.end());
    }

    return responses;
}

std::vector<robot::RadioResponse> Simulator::applyRobotControlCommand(
    const SSLSimulationProto::RobotControl& control, bool isBlue)
{
    // collect responses from robots
    std::vector<robot::RadioResponse> responses;

    Simulator::RobotMap& robotMap = isBlue ? m_data->robotsBlue : m_data->robotsYellow;

    for (const SSLSimulationProto::RobotCommand& command : control.robot_commands())
    {
        if (!robotMap.contains(command.id()))
        {
            return responses;
        }

        // pass radio command to robot that matches the id
        robot::RadioResponse response = robotMap.at(command.id())
                                            ->setCommand(command, *m_data->ball, m_charge,
                                                         m_data->robotCommandPacketLoss,
                                                         m_data->robotReplyPacketLoss);
        response.set_time(m_time);
        response.set_is_blue(isBlue);

        // only collect valid responses
        if (response.IsInitialized())
        {
            if (m_data->robotReplyPacketLoss == 0 ||
                m_data->rng.uniformFloat(0, 1) > m_data->robotReplyPacketLoss)
            {
                responses.emplace_back(response);
            }
        }
    }

    return responses;
}

void Simulator::resetFlipped(Simulator::RobotMap& robots, float side)
{
    // find flipped robots and align them on a line
    const float x = m_data->geometry.field_width() / 2 - 0.2;
    float y       = m_data->geometry.field_height() / 2 - 0.2;

    for (auto& [robotId, robot] : robots)
    {
        if (robot->isFlipped())
        {
            robot = std::make_unique<SimRobot>(robot->specs(), m_data->dynamicsWorld,
                                               btVector3(x, side * y, 0), 0.0f);
            applyRobotRealism(*robot);
        }
        y -= 0.3;
    }
}

void Simulator::stepSimulation(double time_s)
{
    m_data->dynamicsWorld->stepSimulation(time_s, 10, SUB_TIMESTEP);
    m_time += time_s * 1E9;
}

void Simulator::handleSimulatorTick(double time_s)
{
    // has to be done according to bullet wiki
    m_data->dynamicsWorld->clearForces();

    resetFlipped(m_data->robotsBlue, 1.0f);
    resetFlipped(m_data->robotsYellow, -1.0f);
    if (m_data->ball->isInvalid())
    {
        m_data->ball = std::make_shared<SimBall>(m_data->dynamicsWorld);
    }

    // find out if ball and any robot collide
    auto robot_ball_collision = [&](const auto& kv_pair)
    {
        auto& [robotId, robot] = kv_pair;
        return robot->touchesBall(*m_data->ball);
    };
    bool ball_collision = std::any_of(m_data->robotsBlue.begin(),
                                      m_data->robotsBlue.end(), robot_ball_collision) ||
                          std::any_of(m_data->robotsYellow.begin(),
                                      m_data->robotsYellow.end(), robot_ball_collision);

    // apply commands and forces to ball and robots
    m_data->ball->begin(ball_collision);
    for (auto& [robotId, robot] : m_data->robotsBlue)
    {
        robot->begin(*m_data->ball, time_s);
    }
    for (auto& [robotId, robot] : m_data->robotsYellow)
    {
        robot->begin(*m_data->ball, time_s);
    }

    // add gravity to all ACTIVE objects
    // thus has to be done after applying commands
    m_data->dynamicsWorld->applyGravity();
}

static bool checkCameraID(const int cameraId, const btVector3& p,
                          const std::vector<btVector3>& cameraPositions,
                          const float overlap)
{
    float minDistance = std::numeric_limits<float>::max();
    float ownDistance = 0;
    for (int i = 0; i < cameraPositions.size(); i++)
    {
        // manhattan distance for rectangular camera regions (if the cameras are
        // distributed normally)
        float distance = std::abs(cameraPositions[i].x() - p.x()) +
                         std::abs(cameraPositions[i].y() - p.y());
        minDistance = std::min(minDistance, distance);
        if (i == cameraId)
        {
            ownDistance = distance;
        }
    }
    return ownDistance <= minDistance + 2 * overlap;
}

void Simulator::initializeDetection(SSLProto::SSL_DetectionFrame& detection,
                                    size_t cameraId)
{
    detection.set_frame_number(m_lastFrameNumber[cameraId]++);
    detection.set_camera_id(cameraId);
    detection.set_t_capture((m_time + m_visionDelay - m_visionProcessingTime) * 1E-9);
    detection.set_t_sent((m_time + m_visionDelay) * 1E-9);
}

static btVector3 positionOffsetForCamera(float offsetStrength, btVector3 cameraPos)
{
    btVector3 cam2d{cameraPos.x(), cameraPos.y(), 0};
    if (offsetStrength < 1e-9)
    {
        // do not produce an offset that tiny
        return {0, 0, 0};
    }
    if (cam2d.length() < offsetStrength)
    {
        // do not normalize a 0 vector
        return cam2d;
    }
    return btVector3(cameraPos.x(), cameraPos.y(), 0).normalized() * offsetStrength;
}

uint32_t Simulator::getRotatedRobotId(uint32_t id)
{
    // pink is 0, green is 1, starting from the top left side (dribbler is up) as the
    // most significant bit, going clockwise to the least significant bit
    static constexpr std::array<uint32_t, 16> PATTERNS = {
        0b0001,  // 0
        0b1001,  // 1
        0b1101,  // 2
        0b0101,  // 3
        0b0010,  // 4
        0b1010,  // 5
        0b1110,  // 6
        0b0110,  // 7
        0b1111,  // 8
        0b0000,  // 9
        0b0011,  // 10
        0b1100,  // 11
        0b1011,  // 12
        0b1000,  // 13
        0b0111,  // 14
        0b0100,  // 15
    };

    if (id >= PATTERNS.size())
    {
        return id;
    }

    const uint32_t pattern        = PATTERNS[id];
    const uint32_t rotatedPattern = (pattern >> 1) | ((pattern << 3) & 0b1000);
    const auto it = std::find(PATTERNS.begin(), PATTERNS.end(), rotatedPattern);
    return static_cast<uint32_t>(std::distance(PATTERNS.begin(), it));
}

void Simulator::createRobotDetection(
    SimRobot& robot, bool teamIsBlue,
    std::vector<SSLProto::SSL_DetectionFrame>& detections)
{
    if (m_time - robot.getLastSendTime() < m_minRobotDetectionTime)
    {
        return;
    }

    const float timeDiff     = (m_time - robot.getLastSendTime()) * 1E-9;
    const btVector3 robotPos = robot.position() / SIMULATOR_SCALE;

    const std::size_t numCameras = m_data->reportedCameraSetup.size();
    for (std::size_t cameraId = 0; cameraId < numCameras; ++cameraId)
    {
        if (!checkCameraID(cameraId, robotPos, m_data->cameraPositions,
                           m_data->cameraOverlap))
        {
            continue;
        }

        const btVector3& cameraPos = m_data->cameraPositions[cameraId];
        const btVector3 positionOffset =
            positionOffsetForCamera(m_data->objectPositionOffset, cameraPos);
        auto& cameraDetections = detections[cameraId];

        const bool missingRobot =
            m_data->missingRobotDetections > 0 &&
            m_data->rng.uniformFloat(0, 1) <= m_data->missingRobotDetections;
        if (!missingRobot)
        {
            auto& robotDetection = teamIsBlue ? *cameraDetections.add_robots_blue()
                                              : *cameraDetections.add_robots_yellow();
            robot.update(robotDetection, m_data->stddevRobot, m_data->stddevRobotPhi,
                         m_time, positionOffset);
        }

        // Once in a while the vision misreads a robot's pattern as the one that it
        // turns into when rotated by 90 degrees, and reports that second robot on top
        // of the real one. Such a misdetection tends to persist for a few frames, so
        // whether it starts and whether it stops are rolled separately.
        const auto rotatedDetectionKey =
            std::make_tuple(cameraId, robot.specs().id(), teamIsBlue);
        const bool hadRotated        = m_hasRotatedDetection[rotatedDetectionKey];
        const float rotatedThreshold = hadRotated
                                           ? (1 - m_data->rotatedRobotDetectionsStop)
                                           : m_data->rotatedRobotDetectionsStart;
        const bool hasRotated        = m_data->rng.uniformFloat(0, 1) < rotatedThreshold;
        m_hasRotatedDetection[rotatedDetectionKey] = hasRotated;

        if (hasRotated)
        {
            auto& rotatedDetection = teamIsBlue ? *cameraDetections.add_robots_blue()
                                                : *cameraDetections.add_robots_yellow();
            robot.update(rotatedDetection, m_data->stddevRobot, m_data->stddevRobotPhi,
                         m_time, positionOffset);
            rotatedDetection.set_robot_id(getRotatedRobotId(rotatedDetection.robot_id()));
            rotatedDetection.set_orientation(rotatedDetection.orientation() + M_PI_2);
        }

        // once in a while, add a ball mis-detection at a corner of the dribbler
        // in real games, this happens because the ball detection light beam used by
        // many teams is red
        const float detectionProb = timeDiff * m_data->ballDetectionsAtDribbler;
        if (m_data->ballDetectionsAtDribbler > 0 &&
            m_data->rng.uniformFloat(0, 1) < detectionProb)
        {
            // always on the right side of the dribbler for now
            if (!m_data->ball->addDetection(*cameraDetections.add_balls(),
                                            robot.dribblerCorner(false) / SIMULATOR_SCALE,
                                            m_data->stddevRobot, 0, cameraPos, false, 0,
                                            positionOffset))
            {
                cameraDetections.mutable_balls()->DeleteSubrange(
                    cameraDetections.balls_size() - 1, 1);
            }
        }
    }
}

void Simulator::applyRobotRealism(SimRobot& robot) const
{
    robot.setDribbleMode(m_data->dribblePerfect);
    robot.setRotationError(m_data->robotRotationError);
}

std::vector<SSLProto::SSL_WrapperPacket> Simulator::getWrapperPackets()
{
    const std::size_t numCameras = m_data->reportedCameraSetup.size();

    std::vector<SSLProto::SSL_DetectionFrame> detections(numCameras);
    for (std::size_t i = 0; i < numCameras; i++)
    {
        initializeDetection(detections[i], i);
    }

    const btVector3 ballPosition = m_data->ball->position() / SIMULATOR_SCALE;
    if (m_time - m_lastBallSendTime >= m_minBallDetectionTime)
    {
        m_lastBallSendTime = m_time;

        for (std::size_t cameraId = 0; cameraId < numCameras; ++cameraId)
        {
            // at least one id is always valid
            if (!checkCameraID(cameraId, ballPosition, m_data->cameraPositions,
                               m_data->cameraOverlap))
            {
                continue;
            }

            // the ball is missed by each camera individually, rather than by all of
            // them at once, which is what the real vision does
            const bool missingBall =
                m_data->missingBallDetections > 0 &&
                m_data->rng.uniformFloat(0, 1) <= m_data->missingBallDetections;
            if (missingBall)
            {
                continue;
            }

            // get ball position
            const btVector3 positionOffset = positionOffsetForCamera(
                m_data->objectPositionOffset, m_data->cameraPositions[cameraId]);
            bool visible = m_data->ball->update(
                *detections[cameraId].add_balls(), m_data->stddevBall,
                m_data->stddevBallArea, m_data->cameraPositions[cameraId],
                m_data->enableInvisibleBall, m_data->ballVisibilityThreshold,
                positionOffset);
            if (!visible)
            {
                detections[cameraId].clear_balls();
            }
        }
    }

    // get robot positions
    for (auto& [teamIsBlue, team] :
         {std::make_tuple(true, std::ref(m_data->robotsBlue)),
          std::make_tuple(false, std::ref(m_data->robotsYellow))})
    {
        for (auto& [robotId, robot] : team)
        {
            createRobotDetection(*robot, teamIsBlue, detections);
        }
    }

    std::vector<SSLProto::SSL_WrapperPacket> packets;
    packets.reserve(numCameras);

    // add a wrapper packet for all detections (also for empty ones).
    // The reason is that other teams might rely on the fact that these detections
    // are in regular intervals.
    for (auto& frame : detections)
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
    SSLProto::SSL_GeometryData* geometry   = packets[0].mutable_geometry();
    SSLProto::SSL_GeometryFieldSize* field = geometry->mutable_field();
    convertToSSlGeometry(m_data->geometry, field);

    const btVector3 positionErrorSimScale =
        btVector3(0.3f, 0.7f, 0.05f).normalized() * m_data->cameraPositionError;
    btVector3 positionErrorVisionScale{0, 0, positionErrorSimScale.z() * 1000};
    coordinates::toVision(positionErrorSimScale, positionErrorVisionScale);
    for (const auto& calibration : m_data->reportedCameraSetup)
    {
        auto calib = geometry->add_calib();
        calib->CopyFrom(calibration);
        calib->set_derived_camera_world_tx(calib->derived_camera_world_tx() +
                                           positionErrorVisionScale.x());
        calib->set_derived_camera_world_ty(calib->derived_camera_world_ty() +
                                           positionErrorVisionScale.y());
        calib->set_derived_camera_world_tz(calib->derived_camera_world_tz() +
                                           positionErrorVisionScale.z());
    }

    // Add the ball model to the geometry data. These values describe the ball model
    // that this simulator actually implements (see SimBall::begin), so that consumers
    // of the geometry packet can predict the ball the same way we simulate it. The
    // sliding deceleration is the effective coefficient of friction between ball and
    // floor (the product of both bodies' friction values) times gravity.
    auto* ball_model = geometry->mutable_models()->mutable_straight_two_phase();
    ball_model->set_acc_roll(
        BALL_ROLLING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED);
    ball_model->set_acc_slide(-BALL_SLIDING_FRICTION_NEWTONS * FLOOR_FRICTION *
                              ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED);
    ball_model->set_k_switch(FRICTION_TRANSITION_FACTOR);
    geometry->mutable_models()->mutable_chip_fixed_loss()->set_damping_z(0.566);
    geometry->mutable_models()->mutable_chip_fixed_loss()->set_damping_xy_first_hop(
        0.715);
    geometry->mutable_models()->mutable_chip_fixed_loss()->set_damping_xy_other_hops(1);

    return packets;
}

world::SimulatorState Simulator::getSimulatorState()
{
    const std::size_t numCameras = m_data->reportedCameraSetup.size();
    world::SimulatorState simState;

    m_data->ball->writeBallState(*simState.mutable_ball());

    // get robot positions
    for (bool teamIsBlue : {true, false})
    {
        auto& team = teamIsBlue ? m_data->robotsBlue : m_data->robotsYellow;

        for (auto& [robotId, robot] : team)
        {
            // convert coordinates from ER Force
            btVector3 robotPos = robot->position() / SIMULATOR_SCALE;
            btVector3 newRobotPos;

            coordinates::toVision(robotPos, newRobotPos);

            auto& robotProto =
                teamIsBlue ? *simState.add_blue_robots() : *simState.add_yellow_robots();

            robot->update(robotProto, *m_data->ball);

            // Convert mm to m
            robotProto.set_p_x(newRobotPos.x() / 1000);
            robotProto.set_p_y(newRobotPos.y() / 1000);

            // Convert velocity
            coordinates::toVisionVelocity(robotProto, robotProto);
            robotProto.set_v_x(robotProto.v_x() / 1000);
            robotProto.set_v_y(robotProto.v_y() / 1000);
        }
    }

    return simState;
}

void Simulator::setTeam(Simulator::RobotMap& robotMap, float side,
                        const robot::Team& team,
                        std::map<uint32_t, robot::Specs>& teamSpecs)
{
    // remove old team
    robotMap.clear();

    // align robots on a line
    const float x = m_data->geometry.field_width() / 2 - 0.2;
    float y       = m_data->geometry.field_height() / 2 - 0.2;

    for (int i = 0; i < team.robot_size(); i++)
    {
        const robot::Specs& specs = team.robot(i);
        const auto id             = specs.id();

        // (color, robot id) must be unique
        if (robotMap.contains(id))
        {
            std::cerr << "Error: Two ids for the same color, aborting!" << std::endl;
            continue;
        }
        teamSpecs[id].CopyFrom(specs);

        robotMap[id] = std::make_unique<SimRobot>(teamSpecs[id], m_data->dynamicsWorld,
                                                  btVector3(x, side * y, 0), 0.f);
        applyRobotRealism(*robotMap[id]);

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

void Simulator::moveBall(const sslsim::TeleportBall& ball)
{
    // remove the dribbling constraint
    if (!ball.has_by_force() || !ball.by_force())
    {
        for (auto& [robotId, robot] : m_data->robotsBlue)
        {
            robot->stopDribbling();
        }
        for (auto& [robotId, robot] : m_data->robotsYellow)
        {
            robot->stopDribbling();
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

void Simulator::moveRobot(const sslsim::TeleportRobot& robot)
{
    if (!robot.id().has_team())
        return;

    if (!robot.id().has_id())
        return;

    bool isBlue        = robot.id().team() == gameController::Team::BLUE;
    RobotMap& robotMap = isBlue ? m_data->robotsBlue : m_data->robotsYellow;
    bool isPresent     = robotMap.contains(robot.id().id());
    std::map<uint32_t, robot::Specs>& teamSpecs =
        isBlue ? m_data->specsBlue : m_data->specsYellow;

    if (robot.has_present())
    {
        if (robot.present() && !isPresent)
        {
            // add the requested robot
            if (!teamSpecs.contains(robot.id().id()))
            {
                std::cerr << "Trying to create robot " << std::to_string(robot.id().id())
                          << ", but no spec for this robot was found" << std::endl;
            }
            else if (!robot.has_x() || !robot.has_y())
            {
                std::cerr << "Trying to create robot " << std::to_string(robot.id().id())
                          << ", without giving a position" << std::endl;
            }
            else
            {
                ErForceVector targetPos;
                coordinates::fromVision(robot, targetPos);
                // TODO: check if the given position is fine

                robotMap[robot.id().id()] = std::make_unique<SimRobot>(
                    teamSpecs[robot.id().id()], m_data->dynamicsWorld,
                    btVector3(targetPos.x, targetPos.y, 0), 0.f);
                applyRobotRealism(*robotMap[robot.id().id()]);
            }
        }
        else if (!robot.present() && isPresent)
        {
            // remove the robot
            robotMap.at(robot.id().id())->stopDribbling();
            robotMap.erase(robot.id().id());
            return;
        }
        else if (!robot.present() && !isPresent)
        {
            return;
        }
        // Fall though: If the robot is already on the field and needs to be on the
        // field, we just use that robot.
    }
    else
    {
        if (!isPresent)
            return;
    }

    if (!robotMap.contains(robot.id().id()))
        return;  // Recheck the robotMap in case the has_present paragraph did change it.

    sslsim::TeleportRobot r = robot;

    if (m_data->flip)
    {
        FLIP(r, x);
        FLIP(r, y);
        FLIP(r, v_x);
        FLIP(r, v_y);
    }

    if (!r.has_by_force() || !r.by_force())
    {
        robotMap[robot.id().id()]->stopDribbling();
    }
    robotMap[robot.id().id()]->move(r);
}

void Simulator::handleSimulatorSetupCommand(const std::unique_ptr<amun::Command>& command)
{
    bool robotRealismChanged = false;

    if (command->has_simulator())
    {
        const amun::CommandSimulator& sim = command->simulator();

        if (sim.has_enable())
        {
            m_enabled = sim.enable();
        }

        if (sim.has_realism_config())
        {
            auto realism = sim.realism_config();

            if (realism.has_stddev_ball_p())
            {
                m_data->stddevBall = realism.stddev_ball_p();
            }

            if (realism.has_stddev_robot_p())
            {
                m_data->stddevRobot = realism.stddev_robot_p();
            }

            if (realism.has_stddev_robot_phi())
            {
                m_data->stddevRobotPhi = realism.stddev_robot_phi();
            }

            if (realism.has_stddev_ball_area())
            {
                m_data->stddevBallArea = realism.stddev_ball_area();
            }

            if (realism.has_dribbler_ball_detections())
            {
                m_data->ballDetectionsAtDribbler = realism.dribbler_ball_detections();
            }

            if (realism.has_enable_invisible_ball())
            {
                m_data->enableInvisibleBall = realism.enable_invisible_ball();
            }

            if (realism.has_ball_visibility_threshold())
            {
                m_data->ballVisibilityThreshold = realism.ball_visibility_threshold();
            }

            if (realism.has_camera_overlap())
            {
                m_data->cameraOverlap = realism.camera_overlap();
            }

            if (realism.has_camera_position_error())
            {
                m_data->cameraPositionError = realism.camera_position_error();
            }

            if (realism.has_object_position_offset())
            {
                m_data->objectPositionOffset = realism.object_position_offset();
            }

            if (realism.has_robot_command_loss())
            {
                m_data->robotCommandPacketLoss = realism.robot_command_loss();
            }

            if (realism.has_robot_response_loss())
            {
                m_data->robotReplyPacketLoss = realism.robot_response_loss();
            }

            if (realism.has_missing_ball_detections())
            {
                m_data->missingBallDetections = realism.missing_ball_detections();
            }

            if (realism.has_vision_delay())
            {
                m_visionDelay = std::max<int64_t>(0l, realism.vision_delay());
            }

            if (realism.has_vision_processing_time())
            {
                m_visionProcessingTime =
                    std::max<int64_t>(0l, realism.vision_processing_time());
            }

            if (realism.has_simulate_dribbling())
            {
                m_data->dribblePerfect = !realism.simulate_dribbling();
                robotRealismChanged    = true;
            }

            if (realism.has_missing_robot_detections())
            {
                m_data->missingRobotDetections = realism.missing_robot_detections();
            }

            if (realism.has_command_delay())
            {
                m_data->commandDelay = std::max<int64_t>(0l, realism.command_delay());
            }

            if (realism.has_robot_rotation_error())
            {
                m_data->robotRotationError = realism.robot_rotation_error();
                robotRealismChanged        = true;
            }

            if (realism.has_rotated_robot_detections_start())
            {
                m_data->rotatedRobotDetectionsStart =
                    realism.rotated_robot_detections_start();
            }

            if (realism.has_rotated_robot_detections_stop())
            {
                m_data->rotatedRobotDetectionsStop =
                    realism.rotated_robot_detections_stop();
            }
        }

        if (sim.has_ssl_control())
        {
            const auto& sslControl = sim.ssl_control();
            if (sslControl.has_teleport_ball())
            {
                moveBall(sslControl.teleport_ball());
            }
            for (const auto& moveR : sslControl.teleport_robot())
            {
                moveRobot(moveR);
            }
        }

        if (sim.has_vision_worst_case())
        {
            if (sim.vision_worst_case().has_min_ball_detection_time())
            {
                m_minBallDetectionTime =
                    sim.vision_worst_case().min_ball_detection_time() * 1E9;
            }
            if (sim.vision_worst_case().has_min_robot_detection_time())
            {
                m_minRobotDetectionTime =
                    sim.vision_worst_case().min_robot_detection_time() * 1E9;
            }
        }

        if (sim.has_set_simulator_state())
        {
            if (sim.set_simulator_state().has_ball())
            {
                m_data->ball->restoreState(sim.set_simulator_state().ball());
            }
            const auto restoreRobots = [](RobotMap& map, auto robots)
            {
                for (const auto& robot : robots)
                {
                    if (map.contains(robot.id()))
                    {
                        map[robot.id()]->restoreState(robot);
                    }
                }
            };
            restoreRobots(m_data->robotsYellow,
                          sim.set_simulator_state().yellow_robots());
            restoreRobots(m_data->robotsBlue, sim.set_simulator_state().blue_robots());
        }
    }

    if (command->has_transceiver())
    {
        if (command->transceiver().has_charge())
        {
            m_charge = command->transceiver().charge();
        }
    }

    if (command->has_set_team_blue())
    {
        robotRealismChanged = true;
        setTeam(m_data->robotsBlue, 1.0f, command->set_team_blue(), m_data->specsBlue);
    }

    if (command->has_set_team_yellow())
    {
        robotRealismChanged = true;
        setTeam(m_data->robotsYellow, -1.0f, command->set_team_yellow(),
                m_data->specsYellow);
    }

    if (robotRealismChanged)
    {
        for (auto& [robotId, robot] : m_data->robotsBlue)
        {
            applyRobotRealism(*robot);
        }
        for (auto& [robotId, robot] : m_data->robotsYellow)
        {
            applyRobotRealism(*robot);
        }
    }
}
