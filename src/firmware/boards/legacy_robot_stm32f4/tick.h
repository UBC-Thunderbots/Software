#ifndef TICK_H
#define TICK_H

#include "firmware/app/primitives/primitive_manager.h"
#include "firmware/app/world/firmware_world.h"

void tick_init(PrimitiveManager_t *_primitive_manager, FirmwareWorld_t *_world);
void tick_shutdown(void);
void timer6_isr(void);

#endif
