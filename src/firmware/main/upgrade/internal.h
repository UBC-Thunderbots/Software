#ifndef UPGRADE_INTERNAL_H
#define UPGRADE_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

#include "io/sdcard.h"
#include "upgrade/constants.h"

/**
 * \ingroup UPGRADE
 *
 * \brief The magic number for the firmware storage area.
 */
#define UPGRADE_FW_MAGIC 0x1453CABE

/**
 * \brief The first sector of the firmware data area.
 */
#define UPGRADE_FW_FIRST_SECTOR 0

/**
 * \ingroup UPGRADE
 *
 * \brief The magic number for the FPGA storage area.
 */
#define UPGRADE_FPGA_MAGIC 0x74E4BCC5

/**
 * \brief The first sector of the FPGA data area.
 */
#define UPGRADE_FPGA_FIRST_SECTOR UPGRADE_SD_AREA_SECTORS

bool upgrade_int_check_area(uint32_t sector, uint32_t magic, size_t *length,
                            uint32_t *flags, uint32_t *crc);

#endif
