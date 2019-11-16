#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "util/log.h"

void receive_init(unsigned int index);
void receive_shutdown(void);
void receive_tick(log_record_t *record);
uint8_t receive_last_serial(void);
void handle_camera_packet(uint8_t *, uint8_t);
void handle_drive_packet(uint8_t *);
void handle_other_packet(uint8_t *, size_t);
#endif
