#ifndef DMA_H
#define DMA_H

#include <stdbool.h>
#include <stddef.h>

/**
 * \brief Allocates a DMA buffer.
 *
 * \param[in] name The name of the variable that will be the DMA buffer.
 * \param[in] size The size of the DMA buffer, in bytes.
 */
#define DMA_ALLOCATE(name, size)                                                         \
    _Static_assert(!((size) % 16), "DMA buffer size must be a multiple of 16.");         \
    static unsigned long name[(size) / sizeof(unsigned long)]                            \
        __attribute__((aligned(16), section("dma")));

/**
 * \brief A handle to a DMA-capable, suitably aligned memory block.
 */
typedef struct dma_memory_handle *dma_memory_handle_t;

void dma_init(void);
bool dma_check(const void *pointer, size_t length);
dma_memory_handle_t dma_alloc(size_t length);
void dma_free(dma_memory_handle_t handle);
void *dma_get_buffer(dma_memory_handle_t handle);

#endif
