#ifndef STM32LIB_CRC32_H
#define STM32LIB_CRC32_H

#include <stddef.h>
#include <stdint.h>

#define CRC32_EMPTY 0xFFFFFFFFU

void crc32_init(void);
uint32_t crc32_be(const void *data, size_t length, uint32_t initial);

#endif
