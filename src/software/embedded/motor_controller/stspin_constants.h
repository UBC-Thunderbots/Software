#pragma once

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
