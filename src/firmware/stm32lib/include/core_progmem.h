#ifndef STM32LIB_CORE_PROGMEM_H
#define STM32LIB_CORE_PROGMEM_H

#include <exception.h>
#include <stdint.h>

extern const exception_core_writer_t core_progmem_writer;
extern uint32_t core_progmem_dump[256 * 1024 / 4] __attribute__((section(".coredump")));

#endif
