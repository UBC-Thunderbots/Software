#include <crc32.h>
#include <minmax.h>
#include <stdlib.h>

#include "io/dma.h"
#include "upgrade/constants.h"
#include "upgrade/internal.h"

/**
 * \brief Checks whether a storage area has valid data in it.
 *
 * \param[in] sector the first sector of the storage area, where the header
 * lives
 * \param[in] magic the magic number that exists in a valid header in this area
 * \param[out] length the length of the data in the storage area, if valid
 * \param[out] flags the flags from the storage area header, if valid
 * \param[out] crc the CRC of the storage area data, if valid
 * \retval true the storage area is valid
 * \retval false the storage area is not valid
 */
bool upgrade_int_check_area(uint32_t sector, uint32_t magic, size_t *length,
                            uint32_t *flags, uint32_t *crc)
{
    bool ok = true;

    // Allocate a buffer to hold one sector.
    dma_memory_handle_t block_buffer_handle = dma_alloc(SD_SECTOR_SIZE);
    uint32_t *block_buffer                  = dma_get_buffer(block_buffer_handle);

    // Read the header.
    ok = ok && sd_read(sector++, block_buffer) == SD_STATUS_OK;

    // Check magic number.
    ok = ok && (block_buffer[0] == magic);

    // Extract flags, length, and CRC.
    *flags  = block_buffer[1];
    *length = block_buffer[2];
    *crc    = block_buffer[3];

    // Check for reasonable length.
    ok = ok && *length && (*length <= (UPGRADE_SD_AREA_SECTORS - 1) * SD_SECTOR_SIZE);

    // Check that the data on the SD card matches the CRC from the header.
    uint32_t actual_crc = CRC32_EMPTY;
    size_t length_left  = *length;
    while (ok && length_left)
    {
        ok                       = ok && sd_read(sector++, block_buffer) == SD_STATUS_OK;
        size_t bytes_this_sector = MIN(length_left, SD_SECTOR_SIZE);
        actual_crc               = crc32_be(block_buffer, bytes_this_sector, actual_crc);
        length_left -= bytes_this_sector;
    }
    ok = ok && (actual_crc == *crc);

    // Done.
    dma_free(block_buffer_handle);
    return ok;
}
