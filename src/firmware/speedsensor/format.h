#ifndef STRING_H
#define STRING_H

#include <stddef.h>
#include <stdint.h>

void formatuint1(char *dest, uint8_t val);
void formatuint2(char *dest, uint8_t val);
void formatuint4(char *dest, uint16_t val);
void formatuint8(char *dest, uint32_t val);
void formatuint16(char *dest, uint32_t val);
void formathex4(char *dest, uint8_t val);
void formathex8(char *dest, uint8_t val);
void formathex16(char *dest, uint16_t val);
void formathex32(char *dest, uint32_t val);

#endif
