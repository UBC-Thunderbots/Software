#include "log.h"

void log_destination(log_record_t  *log, float destination[3]) {
    log->tick.primitive_data[0] = destination[0];
    log->tick.primitive_data[1] = destination[1];
    log->tick.primitive_data[2] = destination[2];
}


void log_accel(log_record_t *log, float accel[3]) {
    log->tick.primitive_data[3] = accel[0];
    log->tick.primitive_data[4] = accel[1];
    log->tick.primitive_data[5] = accel[2];
}


void log_time_target(log_record_t *log, float time_target) {
    log->tick.primitive_data[6] = time_target;
}
