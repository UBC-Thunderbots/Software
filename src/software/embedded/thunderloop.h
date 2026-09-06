#pragma once

#include <fstream>

#include "shared/robot_constants.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/robot_localizer.h"
#include "software/embedded/services/imu.h"
#include "software/embedded/services/motor.h"
#include "software/embedded/services/network/network.h"
#include "software/embedded/services/power.h"
#include "software/embedded/toml_config/toml_config_client.h"

class Thunderloop
{
   public:
    /**
     * Thunderloop is a giant loop that runs at THUNDERLOOP_HZ.
     * It receives Primitives from Fullsystem, executes the Primitives
     * based on robot localization data, and polls the services to interact
     * with the hardware peripherals.
     *
     * High Level Diagram: Service order in loop not shown
     *
     *                   ┌─────────────────┐
     *                   │                 │
     *                   │   Thunderloop   │
     *                   │                 │
     *  Primitives───────►                 │ Target Vel ┌────────────┐
     *                   │                 ├────────────►            │
     *                   |                 │            │ MotorBoard │
     *                   │    Services     ◄────────────┤            │
     *                   │                 │ Actual Vel └────────────┘
     *                   │  Primitive Exec │
     *                   │                 │
     *                   │                 │ Kick/Chip  ┌────────────┐
     *                   │                 ├────────────►            │
     * Robot Status◄─────┤                 │            │ PowerBoard │
     *                   │                 ◄────────────┤            │
     *                   └─────────────────┘  Voltages  └────────────┘
     *
     *
     * @param robot_constants The robot constants
     * @param enable_log_merging Whether to merge repeated log message or not
     * @param loop_hz The rate to run the loop
     */
    Thunderloop(const robot_constants::RobotConstants& robot_constants,
                bool enable_log_merging, int loop_hz);

    [[noreturn]] void runLoop();

   private:
    /**
     * Updates the robot localizer with the vision-derived position and orientation
     * from a primitive's trajectory parameters.
     *
     * @param primitive The primitive to extract the vision measurement from
     */
    void updateRobotLocalizer(const TbotsProto::Primitive& primitive);

    /**
     * Updates the robot localizer with the motor and IMU measurements contained in
     * a robot status.
     *
     * @param robot_status The robot status containing motor and IMU measurements
     */
    void updateRobotLocalizer(const TbotsProto::RobotStatus& robot_status);

    std::unique_ptr<TomlConfigClient> toml_config_client_;
    std::unique_ptr<MotorService> motor_service_;
    std::unique_ptr<NetworkService> network_service_;
    std::unique_ptr<PowerService> power_service_;
    std::unique_ptr<ImuService> imu_service_;
    std::unique_ptr<RobotLocalizer> robot_localizer_;
    std::unique_ptr<PrimitiveExecutor> primitive_executor_;

    int loop_hz_;

    // This robot status message is updated by each service and then sent
    // to fullsystem over the network.
    TbotsProto::RobotStatus robot_status_;
};
