/**
 * \ingroup UPGRADE
 *
 * \{
 */
#include "common.h"

#include "io/dma.h"
#include "io/sdcard.h"

DMA_ALLOCATE(upgrade_common_sector_dma_buffer, SD_SECTOR_SIZE);

void *upgrade_common_get_sector_dma_buffer(void)
{
    return upgrade_common_sector_dma_buffer;
}
/**
 * \}
 */
