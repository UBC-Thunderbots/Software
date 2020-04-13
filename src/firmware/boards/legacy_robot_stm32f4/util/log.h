#ifndef LOG_H
#define LOG_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "io/sdcard.h"
#include "util/error.h"

#define LOG_RECORD_SIZE 256U  // 128U
#define LOG_MAGIC_TICK UINT32_C(0xE2468845)

/**
 * \ingroup LOG
 *
 * \brief The type of payload associated with a tick-type log record.
 */
typedef struct __attribute__((packed))
{
    float breakbeam_diff;
    float battery_voltage;
    float capacitor_voltage;

    float dr_x;
    float dr_y;
    float dr_angle;
    float dr_vx;
    float dr_vy;
    float dr_avel;

    float enc_vx;
    float enc_vy;
    float enc_avel;

    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;

    float gyro_avel;

    float cam_x;
    float cam_y;
    float cam_angle;

    float cam_ball_x;
    float cam_ball_y;

    uint16_t cam_delay;
    uint8_t new_cam_data;

    uint8_t drive_serial;
    uint8_t primitive;
    float primitive_data[10U];

    int16_t wheels_encoder_counts[4U];
    int16_t wheels_drives[4U];
    uint8_t wheels_temperatures[4U];

    uint8_t dribbler_ticked;
    uint8_t dribbler_pwm;
    uint8_t dribbler_speed;
    uint8_t dribbler_temperature;

    uint32_t idle_cpu_cycles;

    uint8_t errors[ERROR_BYTES];
} log_tick_t;

/**
 * \ingroup LOG
 *
 * \brief The type of a log record.
 */
typedef struct
{
    uint32_t magic;
    uint32_t epoch;
    uint64_t time;
    union {
        log_tick_t tick;
        uint8_t padding[LOG_RECORD_SIZE - 4U - 4U - 8U];
    };
} log_record_t;

/**
 * \ingroup LOG
 *
 * \brief The possible states the logging subsystem can be in.
 */
typedef enum
{
    /**
     * \brief The logging subsystem is working properly.
     */
    LOG_STATE_OK,

    /**
     * \brief The logging subsystem has not yet been initialized or has been
     * deinitialized.
     */
    LOG_STATE_UNINITIALIZED,

    /**
     * \brief The logging subsystem has shut down permanently because the SD card module
     * reported an error.
     */
    LOG_STATE_SD_ERROR,

    /**
     * \brief The logging subsystem has shut down permanently because the SD card is full.
     */
    LOG_STATE_CARD_FULL,
} log_state_t;

log_state_t log_state(void);
sd_status_t log_last_error(void);
bool log_init(void);
void log_shutdown(void);
log_record_t *log_alloc(void);
void log_queue(log_record_t *record);


/**
 * Logs destination information.
 *
 * @param log a log object to save information into
 * @param destination a 3 length array of {x, y, rotation} destination values
 * on the global axis
 */
void log_destination(log_record_t *log, float destination[3]);

/**
 * Logs acceleration information.
 *
 * @param log a log object to save information into
 * @param accel a 3 length array of {x, y, rotation} accelerations
 */
void log_accel(log_record_t *log, float accel[3]);

/**
 * Logs the time target.
 *
 * @param log a log object to save information into
 * @param time_target the time target to log
 */
void log_time_target(log_record_t *log, float time_target);

#endif
