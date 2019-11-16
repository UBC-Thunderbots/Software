#ifndef MRF_H
#define MRF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    MRF_TX_OK,
    MRF_TX_NO_ACK,
    MRF_TX_CCA_FAIL,
    MRF_TX_CANCELLED,
} mrf_tx_result_t;

typedef struct
{
    /**
     * \brief the channel to operate on, from 11 to 26 inclusive
     */
    uint8_t channel;

    /**
     * \brief \c true to run at 625 kb/s, or \c false to run at 250 kb/s
     */
    bool symbol_rate;

    /**
     * \brief the 16-bit address to use
     */
    uint16_t pan_id;

    /**
     * \brief the PAN ID to communicate on, from 0 to 0xFFFE
     */
    uint16_t short_address;

    /**
     * \brief the node’s MAC address
     */
    uint64_t mac_address;
} mrf_settings_t;

void mrf_init(const mrf_settings_t *settings);
void mrf_shutdown(void);
uint16_t mrf_pan_id(void);
uint16_t mrf_short_address(void);
uint8_t mrf_alloc_seqnum(void);
mrf_tx_result_t mrf_transmit(const void *frame);
void mrf_transmit_cancel(void);
size_t mrf_receive(void *buffer);
void mrf_receive_cancel(void);

#endif
