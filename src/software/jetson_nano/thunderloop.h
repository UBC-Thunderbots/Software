#pragma once
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/redis/redis_client.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/services/network/network.h"
#include "software/jetson_nano/services/power.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

class Thunderloop
{
   public:
    /**
     * Thunderloop is a giant loop that runs at CONTROL_LOOP_HZ.
     * It receives Primitives and World from AI, executes the primitives with
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
     *  World ───────────►                 │            │ MotorBoard │
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
    Thunderloop(const RobotConstants_t &robot_constants, bool enable_log_merging,
                const int loop_hz);

    ~Thunderloop();

    void runLoop();

    // Services
    std::unique_ptr<MotorService> motor_service_;
    std::unique_ptr<NetworkService> network_service_;
    std::unique_ptr<PowerService> power_service_;

    // Clients
    std::unique_ptr<RedisClient> redis_client_;

   private:
    /*
     * The struct timespec consists of nanoseconds and seconds. If the nanoseconds
     * are getting bigger than 1000000000 (= 1 second) the variable containing
     * seconds has to be incremented and the nanoseconds decremented by 1000000000.
     *
     * @param ts timespec to modify
     */
    void timespecNorm(struct timespec &ts);

    /**
     * Get the CPU temp thunderloop is running on
     *
     * @return The CPU temp.
     */
    double getCpuTemperature();

    /**
     * Converts the given timespec value to milliseconds
     * @return The time in milliseconds
     */
    double getMilliseconds(timespec time);

    /**
     * Converts the given timespec value to nanoseconds
     * @return The time in nanoseconds
     */
    double getNanoseconds(timespec time);

    // Input Msg Buffers
    TbotsProto::PrimitiveSet primitive_set_;
    TbotsProto::World world_;
    TbotsProto::Primitive primitive_;
    TbotsProto::DirectControlPrimitive direct_control_;

    // Output Msg Buffers
    TbotsProto::RobotStatus robot_status_;
    TbotsProto::JetsonStatus jetson_status_;
    TbotsProto::NetworkStatus network_status_;
    TbotsProto::PowerStatus power_status_;
    std::optional<TbotsProto::MotorStatus> motor_status_;
    TbotsProto::ThunderloopStatus thunderloop_status_;
    TbotsProto::ChipperKickerStatus chipper_kicker_status_;
    TbotsProto::PrimitiveExecutorStatus primitive_executor_status_;
    TbotsProto::Timestamp time_sent_;

    // Current State
    RobotConstants_t robot_constants_;
    Angle current_orientation_;
    int robot_id_;
    int channel_id_;
    std::string network_interface_;
    int loop_hz_;

    // Calibrated power service constants
    double kick_coeff_;
    int kick_constant_;
    int chip_pulse_width_;

    // Primitive Executor
    PrimitiveExecutor primitive_executor_;

    // 500 millisecond timeout on receiving primitives before we stop the robots
    const double PACKET_TIMEOUT_NS = 500.0 * NANOSECONDS_PER_MILLISECOND;

    // Path to the CPU thermal zone temperature file
    const std::string CPU_TEMP_FILE_PATH = "/sys/class/thermal/thermal_zone1/temp";
};
