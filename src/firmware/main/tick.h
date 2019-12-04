#ifndef TICK_H
#define TICK_H

#include "world/firmware_world.h"

void tick_init(FirmwareWorld* _world);
void tick_shutdown(void);
void timer6_isr(void);

#endif
