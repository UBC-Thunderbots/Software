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
     * @param robot_constants The constants for this robot
     */
    Thunderloop(const RobotConstants_t& robot_constants)
    {
        robot_id_        = 0;
        robot_constants_ = robot_constants;
        motor_service_   = std::make_unique<MotorService>(std::string("/dev/spidev0.1"));

        current_robot_state_ =
            std::make_unique<RobotState>(Point(), Vector(), Angle(), AngularVelocity());
        // Initialize Services here
    }

    ~Thunderloop()
    {
        // De-initialize Services here
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
            // TODO (#TODO) add loop timing instrospection and use Preempt-RT (maybe)
            next_frame += std::chrono::milliseconds(
                static_cast<int>(MILLISECONDS_PER_SECOND / run_at_hz));

            // TODO (#TODO) poll network service and update current_robot_state_
            // TODO (#TODO) poll redis service

            // Execute latest primitive
            primitive_executor_.startPrimitive(robot_constants_, primitive_);
            direct_control_ = *primitive_executor_.stepPrimitive(*current_robot_state_);

            // Poll motor service with wheel velocities and buffer drive unit output
            drive_units_status_ =
                *motor_service_->poll(direct_control_.direct_per_wheel_control());

            // TODO (#TODO) power service poll

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
    std::unique_ptr<RobotState> current_robot_state_;
    unsigned robot_id_;
};


int main(int argc, char** argv)
{
    // TODO (#TODO) replace with network logger
    LoggerSingleton::initializeLogger("/tmp");
    auto thunderloop = Thunderloop(create2021RobotConstants());
    thunderloop.run(CONTROL_LOOP_HZ);
    return 0;
}
