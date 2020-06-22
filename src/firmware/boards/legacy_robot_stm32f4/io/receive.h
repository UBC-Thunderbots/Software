#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "firmware/app/primitives/primitive_manager.h"
#include "firmware/app/world/firmware_world.h"
#include "util/log.h"

void receive_init(unsigned int index, PrimitiveManager_t *_primitive_manager,
                  FirmwareWorld_t *_world);
void receive_shutdown(void);
void receive_tick(log_record_t *record);
void handle_camera_packet(uint8_t *, uint8_t);
void handle_drive_packet(uint8_t *packet_data, size_t packet_size);
void handle_other_packet(uint8_t *, size_t);
#endif
