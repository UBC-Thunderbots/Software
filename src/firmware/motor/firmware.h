#ifndef __FIRMWARE_H
#define __FIRMWARE_H

#include "firmware/motor/types.h"
#include "firmware/motor/crc.h"
#include "firmware/motor/stm32f0xx/stm32f0xx_hal.h"
#include "firmware/motor/stm32f0xx/stm32f0xx_hal_spi.h"

#include <stdint.h>

#define SPI_TIMEOUT HAL_MAX_DELAY

#define CRC_FAIL \
    (uint8_t[]) { FRAME_SOF, NACK, 0x0, 0x0, 0xA4, FRAME_EOF }

void MC_SPI_ReceiveMessage(SPI_HandleTypeDef *hspi, uint8_t *msgBuf);
/**
 * Checks if a byte value is a valid member of the @ref OPCODES enum.
 *
 * @param x The byte value to check
 * @return 1 if the value is a valid opcode, 0 otherwise
 * @see OPCODES
 */
int isOpcode(uint8_t x);

#endif
