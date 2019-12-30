#include "fpga.h"

#include <minmax.h>

#include "common.h"
#include "internal.h"
#include "io/dma.h"
#include "io/icb.h"
#include "io/sdcard.h"

/**
 * \brief The length of the FPGA data, when checked.
 */
static size_t upgrade_fpga_length;

/**
 * \brief The build ID, read from the header.
 */
static uint32_t upgrade_fpga_build_id_value;

bool upgrade_fpga_check(void)
{
    uint32_t flags;

    return upgrade_int_check_area(UPGRADE_FPGA_FIRST_SECTOR, UPGRADE_FPGA_MAGIC,
                                  &upgrade_fpga_length, &flags,
                                  &upgrade_fpga_build_id_value);
}

bool upgrade_fpga_send(void)
{
    void *block_buffer   = upgrade_common_get_sector_dma_buffer();
    bool ok              = true;
    uint32_t next_sector = UPGRADE_FPGA_FIRST_SECTOR + 1;
    size_t length_left   = upgrade_fpga_length;
    while (ok && length_left)
    {
        ok = sd_read(next_sector++, block_buffer) == SD_STATUS_OK;
        if (ok)
        {
            icb_conf_block(block_buffer, MIN(SD_SECTOR_SIZE, length_left));
            length_left -= MIN(SD_SECTOR_SIZE, length_left);
        }
    }
    return ok;
}

uint32_t upgrade_fpga_build_id(void)
{
    return upgrade_fpga_build_id_value;
}
