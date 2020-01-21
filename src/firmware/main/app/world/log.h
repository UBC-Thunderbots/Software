#pragma once

/**
 * This struct represents a log that the robot can write information to
 */
typedef struct Log Log_t;

// TODO:
Log_t* app_log_create();

void app_log_set_breakbeam_diff(Log_t* log, float breakbeam_diff);
