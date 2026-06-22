#pragma once

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <optional>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
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
     * It receives Primitives from AI, executes the Primitives with
     * the most recent vison data, and polls the services to interact
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
                bool enable_log_merging, const int loop_hz);

    ~Thunderloop();

    [[noreturn]] void runLoop();

    // Services
    std::unique_ptr<MotorService> motor_service_;
    std::unique_ptr<NetworkService> network_service_;
    std::unique_ptr<PowerService> power_service_;
    std::unique_ptr<ImuService> imu_service_;

    // TOML config client
    std::unique_ptr<TomlConfigClient> toml_config_client_;

   private:
    // Each loop stage returns its contribution to the outgoing RobotStatus instead of
    // mutating shared member buffers. assembleRobotStatus() is the single place that
    // composes these into robot_status_, so every output proto has one obvious producer.
    struct NetworkPollResult
    {
        TbotsProto::NetworkStatus network_status;
        double poll_time_ms = 0.0;
        // Set only when a new primitive arrived and the executor was (re)started.
        std::optional<double> primitive_start_time_ms;
    };

    struct PrimitiveStepResult
    {
        TbotsProto::DirectControlPrimitive direct_control;
        TbotsProto::PrimitiveExecutorStatus executor_status;
        double step_time_ms = 0.0;
    };

    /*
     * The struct timespec consists of nanoseconds and seconds. If the nanoseconds
     * are getting bigger than 1000000000 (= 1 second) the variable containing
     * seconds has to be incremented and the nanoseconds decremented by 1000000000.
     *
     * @param ts timespec to modify
     */
    void timespecNorm(struct timespec& ts);

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

    /**
     * Updates ErrorCodes for BAT, CAP, CPU TEMP if over thresholds
     */
    void updateErrorCodes();

    /**
     * Poll the motor service, sending it the given control command and writing the
     * resulting motor status into robot_status_.
     *
     * @param direct_control The control command to send to the motors
     * @param time_since_prev_iteration The time difference since the last iteration
     *
     * @return The time taken to poll the service, in milliseconds
     */
    double pollMotorService(const TbotsProto::DirectControlPrimitive& direct_control,
                            const struct timespec& time_since_prev_iteration);

    /**
     * Poll the power service, sending it the given control command and writing the
     * resulting power status into robot_status_.
     *
     * @param direct_control The control command to send to the power board
     *
     * @return The time taken to poll the service, in milliseconds
     */
    double pollPowerService(const TbotsProto::DirectControlPrimitive& direct_control);

    /**
     * Wait for networking communication to be established. This function is blocking.
     */
    void waitForNetworkUp();

    /**
     * Polls the network service: sends the last robot_status_ and receives the newest
     * primitive. When a new primitive arrives, updates the current primitive, seeds the
     * localizer with its starting pose, and (re)starts the primitive executor.
     *
     * @return The network status and timing telemetry for this poll
     */
    inline NetworkPollResult pollNetwork();

    /**
     * Fuses sensor measurements (IMU, motors) into a robot state estimate and returns current Robot State.
     * @return The kinematic state of the robot in the world
     */
    inline RobotState updateLocalization();

    /**
     * Steps the primitive executor, substituting a stop primitive if no primitive has
     * been received within the timeout.
     *
     * @param delta_time The time passed since the last step
     *
     * @return The control command, executor status, and timing telemetry for this step
     */
    inline PrimitiveStepResult stepActivePrimitive(const Duration& delta_time);

    /**
     * Tracks chipper/kicker firing events from the given control command.
     *
     * @param direct_control The control command issued this iteration
     *
     * @return The chipper/kicker status (time since last kick/chip)
     */
    inline TbotsProto::ChipperKickerStatus trackChicker(
        const TbotsProto::DirectControlPrimitive& direct_control);

    /**
     * Composes the outgoing robot_status_ from the per-stage results. This is the single
     * place where the aggregate status is assembled.
     *
     * @param network The result of the network poll stage
     * @param primitive The result of the primitive execution stage
     * @param chicker_status The status from the chicker tracking stage
     * @param motor_poll_time_ms Motor service poll time, or nullopt if disabled
     * @param power_poll_time_ms Power service poll time, or nullopt if disabled
     */
    inline void assembleRobotStatus(const NetworkPollResult& network,
                                    const PrimitiveStepResult& primitive,
                                    const TbotsProto::ChipperKickerStatus& chicker_status,
                                    std::optional<double> motor_poll_time_ms,
                                    std::optional<double> power_poll_time_ms);


    // The current primitive being executed. Persists across iterations; only replaced
    // when a newer primitive arrives over the network.
    TbotsProto::Primitive primitive_;

    // The outgoing robot status. Persists across iterations because the network service
    // sends the copy assembled on the previous iteration. The motor and power services
    // write their status into it directly; assembleRobotStatus() fills in the rest.
    TbotsProto::RobotStatus robot_status_;

    // Sticky timing telemetry. Written only by assembleRobotStatus() (poll times) and the
    // end of runLoop() (iteration time); fields retain their last value between updates.
    TbotsProto::ThunderloopStatus thunderloop_status_;

    // Current State
    robot_constants::RobotConstants robot_constants_;
    int robot_id_;
    int channel_id_;
    std::string network_interface_;
    int loop_hz_;

    // Primitive Executor
    PrimitiveExecutor primitive_executor_;

    // Robot localization model
    RobotLocalizer robot_localizer_;

    // Loop timing state tracked across iterations. Initialized at the start of runLoop()
    // and updated by the stage helpers as the corresponding events occur.
    struct timespec last_primitive_received_time_;
    struct timespec last_chipper_fired_;
    struct timespec last_kicker_fired_;

    // 500 millisecond timeout on receiving primitives before we stop the robots
    const double PACKET_TIMEOUT_NS = 500.0 * NANOSECONDS_PER_MILLISECOND;

    // Timeout after a failed ping request
    const int PING_RETRY_DELAY_S = 1;

    const std::string PATH_TO_RINGBUFFER_LOG = "/usr/bin/dmesg";

    std::ifstream log_file = std::ifstream(PATH_TO_RINGBUFFER_LOG);

    // Path to the CPU thermal zone temperature file
    const std::string CPU_TEMP_FILE_PATH = "/sys/class/thermal/thermal_zone0/temp";
};

/*
 * reads from the kernel ring buffer, likely /var/log/dmesg file, to see if the power
 * is stable
 *
 * This is not defined in Thunderloop to allow it to be unit tested easily
 *
 * @return True if the power is stable, false otherwise
 */
bool isPowerStable(std::ifstream& log_file);
