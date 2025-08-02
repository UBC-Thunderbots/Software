#ifndef __COMMON_TYPES
#define __COMMON_TYPES

#define OPCODE_VALUES                    \
    DEF_VALUE(SPI_NOOP, 0b00000000)           \
    DEF_VALUE(MOV_AX, 0b10000010)        \
    DEF_VALUE(GET_AX, 0b10000011)        \
    DEF_VALUE(MOV_BX, 0b10000100)        \
    DEF_VALUE(GET_BX, 0b10000101)        \
    DEF_VALUE(SET_SPEEDRAMP, 0b00000010) \
    DEF_VALUE(GET_SPEED, 0b00000011)     \
    DEF_VALUE(SET_ENCODER, 0b00000100)   \
    DEF_VALUE(GET_ENCODER, 0b00000101)   \
    DEF_VALUE(START_MOTOR, 0b00001000)   \
    DEF_VALUE(STOP_MOTOR, 0b11111111)    \
    DEF_VALUE(ACK_FAULTS, 0b00010000)    \
    DEF_VALUE(GET_FAULT, 0b00010001)     \
    DEF_VALUE(SET_CURRENT, 0b00100000)   \
    DEF_VALUE(GET_CURRENT, 0b00100001)   \
    DEF_VALUE(ACK, 0b11000000)           \
    DEF_VALUE(NACK, 0b11000001)          \
    DEF_VALUE(SPI_ERROR, 0b11100000)

#define DEF_VALUE(a, b) a = b,
enum OPCODES { OPCODE_VALUES };
#undef DEF_VALUE

/**
 * For documentation on fault codes, visit ST MC SDK v6.2.0 documentation page
 * /group___m_c___type.html#fault_codes
 */
enum FAULT_CODES {
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
    DP_FAULT     = 0x0400,
};

enum FRAME_PARTS {
    FRAME_SOF = 0x73,
    FRAME_EOF = 0x45,
};

#endif
