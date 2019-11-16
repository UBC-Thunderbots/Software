#include <build_id.h>
#include <crc32.h>
#include <stddef.h>

extern const char linker_ramtext_lma;
extern const char linker_ramtext_size;

static uint32_t value;

/**
 * \brief Computes a CRC32 of the text, rodata, data, and ramtext sections of the Flash
 * memory.
 *
 * \pre The CRC32 module must be initialized before invoking this function.
 */
void build_id_init(void)
{
    const char *start = (const char *)0x08000000U;
    size_t length     = (&linker_ramtext_lma - start) + ((size_t)&linker_ramtext_size);
    value             = crc32_be(start, length, CRC32_EMPTY);
}

/**
 * \brief Returns the computed build ID.
 *
 * \return the build ID
 */
uint32_t build_id_get(void)
{
    return value;
}
