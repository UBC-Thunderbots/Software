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
     * It receives Primtiives and Vision from AI, executes the primitives with
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
                const WheelConstants_t& wheel_consants)
    {
        robot_id_        = 0;
        robot_constants_ = robot_constants;
        wheel_consants_  = wheel_consants;

        motor_service_ = std::make_unique<MotorService>(robot_constants, wheel_consants);

        // TODO (#2331) remove this once we receive actual vision data
        current_robot_state_ =
            std::make_unique<RobotState>(Point(), Vector(), Angle(), AngularVelocity());
    }

    ~Thunderloop()
    {
        // De-initialize Services
        motor_service_->stop();
    }

    /*
     * Run the main robot loop!
     *
     * @param The rate to run the loop
     */
    void run(unsigned run_at_hz)
    {
        using clock = std::chrono::steady_clock;

        auto next_frame = clock::now();

        for (;;)
        {
            // TODO (#2335) add loop timing introspection and use Preempt-RT (maybe)
            next_frame += std::chrono::milliseconds(
                static_cast<int>(MILLISECONDS_PER_SECOND / run_at_hz));

            // TODO (#2331) poll network service and update current_robot_state_
            // TODO (#2333) poll redis service

            // Execute latest primitive
            primitive_executor_.startPrimitive(robot_constants_, primitive_);
            direct_control_ = *primitive_executor_.stepPrimitive(*current_robot_state_);

            // Poll motor service with wheel velocities and dribbler rpm
            // TODO (#2332) properly implement, this is just a placeholder
            drive_units_status_ =
                *motor_service_->poll(direct_control_.direct_velocity_control(),
                                      direct_control_.dribbler_speed_rpm());

            // TODO (#2334) power service poll
            std::this_thread::sleep_until(next_frame);
        }
    }

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


int main(int argc, char** argv)
{
    // TODO (#2338) replace with network logger
    LoggerSingleton::initializeLogger("/tmp");

    auto thunderloop =
        Thunderloop(create2021RobotConstants(), create2021WheelConstants());
    thunderloop.run(CONTROL_LOOP_HZ);

    return 0;
}
