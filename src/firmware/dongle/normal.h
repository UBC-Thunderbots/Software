#ifndef NORMAL_H
#define NORMAL_H

#include <stdbool.h>

void normal_init(void);
bool normal_can_enter(void);
void normal_on_enter(void);
void normal_on_exit(void);
void timer6_isr(void);

#endif
