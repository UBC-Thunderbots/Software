#ifndef STM32LIB_FORMAT_H
#define STM32LIB_FORMAT_H

#include <stdint.h>

void formathex4(char *dest, uint8_t val);
void formathex8(char *dest, uint8_t val);
void formathex16(char *dest, uint16_t val);
void formathex32(char *dest, uint32_t val);

#endif
