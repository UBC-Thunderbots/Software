#pragma once

#include <cinttypes>
#include <memory>

#include "proto/robot_status_msg.pb.h"
#include "proto/ssl_simulation_robot_control.pb.h"
#include "proto/ssl_simulation_robot_feedback.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/simulation/firmware_object_deleter.h"
#include "software/simulation/simulator_robot.h"
#include "software/world/robot_state.h"

/**
 * Represents a robot in SSL simulation
 */
class ErForceSimulatorRobot
{
   public:
    /**
     * Create a new ErForceSimulatorRobot given robot id and state
     *
     * @param robot_state the robot state with id
     * @param robot_constants The robot constants
     * @param wheel_constants The wheel constants
     */
    explicit ErForceSimulatorRobot(const RobotStateWithId& robot_state_with_id,
                                   RobotConstants_t robot_constants,
                                   WheelConstants_t wheel_constants);

    /**
     * Returns the ID of this robot
     *
     * @return the ID of this robot
     */
    unsigned int getRobotId();

    /**
     * Sets the robot state
     *
     * @return the robot state
     */
    void setRobotState(const RobotState& robot_state);

    /**
     * Returns the current robot command
     *
     * @return the current robot command
     */
    std::unique_ptr<SSLSimulationProto::RobotCommand> getRobotCommand();

    /**
     * Returns the current robot status
     *
     * @return the current robot status
     */
    std::optional<TbotsProto::RobotStatus> getRobotStatus() const;

    /**
     * Sets the robot feedback
     *
     * @param robot_feedback the robot feedback
     */
    void setRobotFeedback(const SSLSimulationProto::RobotFeedback& robot_feedback);

    /**
     * Resets the values for robot command
     */
    void reset();

    /**
     * Fires the kicker, kicking the ball in the direction the robot is facing
     * at the given speed if the ball is very close to the kicker
     *
     * @param speed_m_per_s How fast to kick the ball, in meters per second
     */
    void kick(float speed_m_per_s);

    /**
     * Fires the chipper, chipping the ball in the direction the robot is facing
     * for the given distance if the ball is very close to the chipper
     *
     * @param speed_m_per_s How far to chip the ball (the distance to the first bounce)
     * in meters
     */
    void chip(float distance_m);

    /**
     * Sets the current primitive this robot is running to a new one
     *
     * @param primitive_msg The primitive to start
     */
    void startNewPrimitive(const TbotsProto::Primitive& primitive);

    /**
     * Runs the current primitive
     */
    void runCurrentPrimitive();

   private:
    // Feedback set at the end of each simulator tick after receiving feedback
    bool dribbler_ball_contact;

    // Robot states
    unsigned int id;
    RobotState robot_state;

    std::optional<float> autokick_speed_m_per_s;
    std::optional<float> autochip_distance_m;

    // Values for robot command
    std::optional<float> kick_speed;       // [m/s]
    std::optional<float> kick_angle;       // [degree]
    std::optional<double> dribbler_speed;  // [rpm]

    RobotConstants_t robot_constants;
    WheelConstants_t wheel_constants;
    PrimitiveExecutor primitive_executor;
    std::unique_ptr<TbotsProto::DirectControlPrimitive> direct_control;
};
