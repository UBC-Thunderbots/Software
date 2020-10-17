#ifndef CHARGER_H
#define CHARGER_H

#include <stdbool.h>

void charger_init(void);
void charger_shutdown(void);
bool charger_full(void);
void charger_mark_fired(void);
void charger_enable(bool enabled);
void charger_tick(void);

#endif
