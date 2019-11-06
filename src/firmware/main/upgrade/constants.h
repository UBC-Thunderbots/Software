#ifndef UPGRADE_CONSTANTS_H
#define UPGRADE_CONSTANTS_H

#include "io/sdcard.h"

/**
 * \ingroup UPGRADE
 *
 * \brief The number of sectors taken up by each of the storage blocks.
 */
#define UPGRADE_SD_AREA_SECTORS (1024 * 4096 / SD_SECTOR_SIZE)

/**
 * \ingroup UPGRADE
 *
 * \brief The number of upgrade areas.
 */
#define UPGRADE_SD_AREA_COUNT 2

#endif
