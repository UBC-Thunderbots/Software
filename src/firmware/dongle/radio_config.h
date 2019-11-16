#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

/**
 * \file
 *
 * \brief Stores configuration parameters that are used in multiple locations.
 */

#include <stdint.h>

/**
 * \brief The configuration parameters.
 */
extern struct radio_config
{
    /**
     * \brief The radio channel.
     */
    uint8_t channel;

    /**
     * \brief The symbol rate.
     */
    uint8_t symbol_rate;

    /**
     * \brief The PAN ID.
     */
    uint16_t pan_id;

    /**
     * \brief The station’s MAC address.
     */
    uint64_t mac_address;
} radio_config;

#endif
