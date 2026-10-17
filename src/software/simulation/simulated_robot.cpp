#include "software/simulation/simulated_robot.h"

SimulatedRobot::SimulatedRobot(const RobotId robot_id,
                               const robot_constants::RobotConstants& robot_constants)
    : motor_service_(std::make_unique<SimulatedMotorService>(
          robot_constants, SimulatedMotorService::NoiseConfig(), robot_id)),
      primitive_executor_(std::make_unique<PrimitiveExecutor>(robot_constants, robot_id))
{
}

void SimulatedRobot::setPrimitive(const TbotsProto::Primitive& primitive,
                                  const RobotState& true_state)
{
    primitive_executor_->updateRobotState(true_state);
    primitive_executor_->updatePrimitive(primitive, robot_status_);
}

TbotsProto::DirectControlPrimitive SimulatedRobot::tick(const RobotState& true_state,
                                                        const Duration& time_step)
{
    const TbotsProto::DirectControlPrimitive direct_control_primitive =
        primitive_executor_->stepPrimitive(robot_status_, time_step);

    return motor_service_->poll(direct_control_primitive, true_state, robot_status_,
                                time_step);
}
