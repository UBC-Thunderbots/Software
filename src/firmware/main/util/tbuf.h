#ifndef TBUF_H
#define TBUF_H

/**
 * \ingroup TBUF
 *
 * \brief A control block for triple buffer management.
 */
typedef unsigned int tbuf_t;

/**
 * \ingroup TBUF
 *
 * \brief The initial value that a \ref tbuf_t instance must be initialized to.
 */
#define TBUF_INIT 0x00000102U

unsigned int tbuf_write_get(tbuf_t *tbuf);
void tbuf_write_put(tbuf_t *tbuf, unsigned int written);
unsigned int tbuf_read_get(tbuf_t *tbuf);
void tbuf_read_put(tbuf_t *tbuf, unsigned int released);

#endif
