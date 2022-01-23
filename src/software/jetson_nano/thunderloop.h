#pragma once
#include <chrono>
#include <iostream>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

class Thunderloop
{
   public:
    /**
     * Thunderloop is a giant loop that runs at CONTROL_LOOP_HZ.
     * It receives Primitives and Vision from AI, executes the primitives with
     * the most recent vison data, and polls the services to interact with the hardware
     * peripherals.
     *
     * High Level Diagram: Service order in loop not shown
     *
     *                   ┌─────────────────┐
     *                   │                 │
     *                   │   ThunderLoop   │
     *                   │                 │
     *  Primitives───────►                 │ Target Vel ┌────────────┐
     *                   │                 ├────────────►            │
     *  Vision───────────►                 │            │ MotorBoard │
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
     * @param wheel_consants The wheel constants
     */
    Thunderloop(const RobotConstants_t& robot_constants,
                const WheelConstants_t& wheel_consants);

    ~Thunderloop();

    /*
     * Run the main robot loop!
     *
     * @param The rate to run the loop
     */
    void run(unsigned run_at_hz);

   private:
    // Services
    std::unique_ptr<MotorService> motor_service_;

    // Primitive Executor
    PrimitiveExecutor primitive_executor_;

    // Input Msg Buffers
    TbotsProto::Primitive primitive_;
    TbotsProto::DirectControlPrimitive direct_control_;

    // Output Msg Buffers
    TbotsProto::RobotStatus robot_status_;
    TbotsProto::NetworkStatus network_status_;
    TbotsProto::PowerStatus power_status_;
    TbotsProto::DriveUnitStatus drive_units_status_;

    // Current State
    RobotConstants_t robot_constants_;
    WheelConstants_t wheel_consants_;
    std::unique_ptr<RobotState> current_robot_state_;
    unsigned robot_id_;
};
