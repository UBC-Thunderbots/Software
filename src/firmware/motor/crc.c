#include "crc.h"
#include <stdint.h>

const uint8_t crc_table[] = CRC_TABLE;

uint8_t crc_gen_checksum(enum OPCODES opcode, uint16_t data) {
    uint8_t crc = 0xFF;

    crc = crc_table[crc ^ opcode];
    crc = crc_table[crc ^ ((uint8_t) (data >> 8))];
    crc = crc_table[crc ^ ((uint8_t) data & 0xFF)];

    return crc ^ 0xFF;
}
