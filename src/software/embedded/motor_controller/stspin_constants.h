#pragma once

enum class StSpinOpcode
{
    SPI_NOOP      = 0b00000000,
    MOV_AX        = 0b10000010,
    GET_AX        = 0b10000011,
    MOV_BX        = 0b10000100,
    GET_BX        = 0b10000101,
    SET_SPEEDRAMP = 0b00000010,
    GET_SPEED     = 0b00000011,
    SET_ENCODER   = 0b00000100,
    GET_ENCODER   = 0b00000101,
    START_MOTOR   = 0b00001000,
    STOP_MOTOR    = 0b11111111,
    ACK_FAULTS    = 0b00010000,
    GET_FAULT     = 0b00010001,
    SET_CURRENT   = 0b00100000,
    GET_CURRENT   = 0b00100001,
    ACK           = 0b11000000,
    NACK          = 0b11000001,
    SPI_ERROR     = 0b11100000
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
