#pragma once
#include <chrono>
#include <iostream>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/2022_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/redis/redis_client.h"
#include "software/jetson_nano/services/motor.h"
#include "software/jetson_nano/services/network.h"
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
     * @param interface The network interface to communicate over
     * @param loop_hz The rate to run the loop
     */
    Thunderloop(const RobotConstants_t& robot_constants, std::string interface,
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
    void timespecNorm(struct timespec& ts);

    // Primitive Executor
    PrimitiveExecutor primitive_executor_;

    // Input Msg Buffers
    TbotsProto::PrimitiveSet primitive_set_;
    TbotsProto::World world_;
    TbotsProto::RobotState robot_state_;
    TbotsProto::Primitive primitive_;
    TbotsProto::DirectControlPrimitive direct_control_;

    // Output Msg Buffers
    TbotsProto::RobotStatus robot_status_;
    TbotsProto::NetworkStatus network_status_;
    TbotsProto::PowerStatus power_status_;
    TbotsProto::MotorStatus motor_status_;
    TbotsProto::ThunderloopStatus thunderloop_status_;

    // Current State
    RobotConstants_t robot_constants_;
    unsigned robot_id_;
    unsigned channel_id_;
    int loop_hz_;
};
