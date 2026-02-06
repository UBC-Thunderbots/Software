#pragma once

#include <cstdint>

enum class StSpinOpcode
{
    NO_OP                = 0x00,
    SET_TARGET_SPEED     = 0x01,
    SET_TARGET_TORQUE    = 0x02,
    SET_RESPONSE_TYPE    = 0x03,
    SET_PID_TORQUE_KP_KI = 0x04,
    SET_PID_FLUX_KP_KI   = 0x05,
    SET_PID_SPEED_KP_KI  = 0x06,
};

enum class StSpinResponseType
{
    SPEED_AND_FAULTS = 0x01,
    IQ_AND_ID        = 0x02,
};

enum class StSpinFaultCode
{
    NO_FAULT     = 0x0000,
    DURATION     = 0x0001,
    OVER_VOLT    = 0x0002,
    UNDER_VOLT   = 0x0004,
    OVER_TEMP    = 0x0008,
    START_UP     = 0x0010,
    SPEED_FDBK   = 0x0020,
    OVER_CURR    = 0x0040,
    SW_ERROR     = 0x0080,
    SAMPLE_FAULT = 0x0100,
    OVERCURR_SW  = 0x0200,
    DP_FAULT     = 0x0400
};

struct SetTargetSpeedFrame
{
    bool motor_enabled;
    int16_t motor_target_speed_rpm;
};

struct SetTargetTorqueFrame
{
    bool motor_enabled;
    int16_t motor_target_torque;
};

struct SetResponseTypeFrame
{
    StSpinResponseType response_type;
};

struct SetPidTorqueKpKiFrame
{
    int16_t kp;
    int16_t ki;
};

struct SetPidFluxKpKiFrame
{
    int16_t kp;
    int16_t ki;
};

struct SetPidSpeedKpKiFrame
{
    int16_t kp;
    int16_t ki;
};
