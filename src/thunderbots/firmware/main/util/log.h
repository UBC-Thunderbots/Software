#include "../log.h"

/**
 * If you want to log data then you should a function for it here
 * so that we can make them easily reusable.
 */ 

/**
 * Logs destination information.
 * 
 * @param log a log object to save information into
 * @param destination a 3 length array of {x, y, rotation} destination values
 * on the global axis
 * @return void
 */ 
void log_destination(log_record_t  *log, float destination[3]); 

/**
 * Logs acceleration information.
 * 
 * @param log a log object to save information into
 * @param accel a 3 length array of {x, y, rotation} accelerations 
 * @return void
 */ 
void log_accel(log_record_t *log, float accel[3]); 

/**
 * Logs the time target.
 * 
 * @param log a log object to save information into
 * @param time_target the time target to log
 * @return void
 */ 
void log_time_target(log_record_t *log, float time_target);
