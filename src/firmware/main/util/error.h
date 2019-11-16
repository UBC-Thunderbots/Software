#ifndef ERROR_H
#define ERROR_H

#include <limits.h>
#include <stdbool.h>

/**
 * \ingroup ERROR
 *
 * \brief The level-triggered errors.
 */
typedef enum
{
    /**
     * \brief The capacitor took too long to charge and has been locked out.
     */
    ERROR_LT_CHARGE_TIMEOUT,

    /**
     * \brief Wheel 0 motor is too hot.
     *
     * The hot motor errors shall be contiguous.
     */
    ERROR_LT_MOTOR0_HOT,

    /**
     * \brief Wheel 1 motor is too hot.
     *
     * The hot motor errors shall be contiguous.
     */
    ERROR_LT_MOTOR1_HOT,

    /**
     * \brief Wheel 2 motor is too hot.
     *
     * The hot motor errors shall be contiguous.
     */
    ERROR_LT_MOTOR2_HOT,

    /**
     * \brief Wheel 3 motor is too hot.
     *
     * The hot motor errors shall be contiguous.
     */
    ERROR_LT_MOTOR3_HOT,

    /**
     * \brief Dribbler motor is too hot.
     *
     * The hot motor errors shall be contiguous.
     */
    ERROR_LT_MOTOR4_HOT,

    /**
     * \brief Wheel 0 encoder is not commutating properly.
     *
     * The encoder commutation errors shall be contiguous.
     */
    ERROR_LT_ENC0_COMMUTATION,

    /**
     * \brief Wheel 1 encoder is not commutating properly.
     *
     * The encoder commutation errors shall be contiguous.
     */
    ERROR_LT_ENC1_COMMUTATION,

    /**
     * \brief Wheel 2 encoder is not commutating properly.
     *
     * The encoder commutation errors shall be contiguous.
     */
    ERROR_LT_ENC2_COMMUTATION,

    /**
     * \brief Wheel 3 encoder is not commutating properly.
     *
     * The encoder commutation errors shall be contiguous.
     */
    ERROR_LT_ENC3_COMMUTATION,

    /**
     * \brief Wheel 0 Hall sensor stuck low.
     *
     * The Hall sensor stuck low errors shall be contiguous.
     */
    ERROR_LT_HALL0_STUCK_LOW,

    /**
     * \brief Wheel 1 Hall sensor stuck low.
     *
     * The Hall sensor stuck low errors shall be contiguous.
     */
    ERROR_LT_HALL1_STUCK_LOW,

    /**
     * \brief Wheel 2 Hall sensor stuck low.
     *
     * The Hall sensor stuck low errors shall be contiguous.
     */
    ERROR_LT_HALL2_STUCK_LOW,

    /**
     * \brief Wheel 3 Hall sensor stuck low.
     *
     * The Hall sensor stuck low errors shall be contiguous.
     */
    ERROR_LT_HALL3_STUCK_LOW,

    /**
     * \brief Dribbler Hall sensor stuck low.
     *
     * The Hall sensor stuck low errors shall be contiguous.
     */
    ERROR_LT_HALL4_STUCK_LOW,

    /**
     * \brief Wheel 0 Hall sensor stuck high.
     *
     * The Hall sensor stuck high errors shall be contiguous.
     */
    ERROR_LT_HALL0_STUCK_HIGH,

    /**
     * \brief Wheel 1 Hall sensor stuck high.
     *
     * The Hall sensor stuck high errors shall be contiguous.
     */
    ERROR_LT_HALL1_STUCK_HIGH,

    /**
     * \brief Wheel 2 Hall sensor stuck high.
     *
     * The Hall sensor stuck high errors shall be contiguous.
     */
    ERROR_LT_HALL2_STUCK_HIGH,

    /**
     * \brief Wheel 3 Hall sensor stuck high.
     *
     * The Hall sensor stuck high errors shall be contiguous.
     */
    ERROR_LT_HALL3_STUCK_HIGH,

    /**
     * \brief Dribbler Hall sensor stuck high.
     *
     * The Hall sensor stuck high errors shall be contiguous.
     */
    ERROR_LT_HALL4_STUCK_HIGH,

    /**
     * \brief The number of level-triggered errors.
     *
     * This must be the last enumerator.
     */
    ERROR_LT_COUNT,
} error_lt_t;

/**
 * \ingroup ERROR
 *
 * \brief The edge-triggered errors.
 */
typedef enum
{
    /**
     * \brief A CRC error occurred on the ICB.
     */
    ERROR_ET_ICB_CRC,

    /**
     * \brief A packet with bad frame check sequence was received from the
     * radio module.
     */
    ERROR_ET_MRF_FCS,

    /**
     * \brief The robot crashed. A core dump is available.
     */
    ERROR_ET_CRASH_CORE,

    /**
     * \brief The robot crashed. No core dump is available.
     */
    ERROR_ET_CRASH_NO_CORE,

    /**
     * \brief The number of edge-triggered errors.
     *
     * This must be the last enumerator.
     */
    ERROR_ET_COUNT,
} error_et_t;

/**
 * \ingroup ERROR
 *
 * \brief The error reporting consumers.
 */
typedef enum
{
    /**
     * \brief Sends errors to the host over the radio.
     */
    ERROR_CONSUMER_MRF,

    /**
     * \brief Records errors into SD card log records.
     */
    ERROR_CONSUMER_LOG,

    /**
     * \brief The number of error reporting consumers.
     *
     * This must be the last enumerator.
     */
    ERROR_CONSUMER_COUNT,
} error_consumer_t;

/**
 * \ingroup ERROR
 *
 * \brief The total number of errors.
 */
#define ERROR_COUNT (ERROR_LT_COUNT + ERROR_ET_COUNT)

/**
 * \ingroup ERROR
 *
 * \brief The number of bytes needed to store the errors as a bitmask.
 */
#define ERROR_BYTES ((ERROR_COUNT + CHAR_BIT - 1) / CHAR_BIT)

bool error_lt_get(error_lt_t error);
void error_lt_set(error_lt_t error, bool active);
void error_et_fire(error_et_t error);
bool error_any_latched(error_consumer_t consumer);
void error_pre_report(error_consumer_t consumer, void *buffer);
void error_post_report(error_consumer_t consumer, bool ok);

#endif
