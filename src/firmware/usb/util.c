/**
 * \defgroup UUTIL USB utility functions
 *
 * \brief These functions provide miscellaneous utility capabilities.
 */

#include <limits.h>
#include <usb.h>

#include "internal.h"



/**
 * \cond INTERNAL
 * \brief Finds the interface descriptor for a specific interface and alternate setting.
 *
 * \param[in] config the configuration descriptor containing the interface
 * \param[in] interface the interface number
 * \param[in] altsetting the alternate setting number
 * \return the interface descriptor, or \c null if not found
 */
const usb_interface_descriptor_t *uutil_find_interface_descriptor(
    const usb_configuration_descriptor_t *config, unsigned int interface,
    unsigned int altsetting)
{
    const uint8_t *base = (const uint8_t *)config;
    const uint8_t *cur  = base;

    while (cur - base < config->wTotalLength)
    {
        if (cur[1U] == USB_DTYPE_INTERFACE)
        {
            const usb_interface_descriptor_t *idesc =
                (const usb_interface_descriptor_t *)cur;
            if (idesc->bInterfaceNumber == interface &&
                idesc->bAlternateSetting == altsetting)
            {
                return idesc;
            }
        }
        cur += cur[0U];
    }

    return 0;
}

/**
 * \brief Finds the endpoint info block for an endpoint.
 *
 * \param[in] ep the endpoint address
 * \return the most specific applicable endpoint control block, or \c null if none
 */
const udev_endpoint_info_t *uutil_find_endpoint_info(unsigned int ep)
{
    if (!UEP_NUM(ep) || UEP_NUM(ep) > UEP_MAX_ENDPOINT || !uep0_current_configuration)
    {
        return 0;
    }

    unsigned int interface = uep_eps[UEP_IDX(ep)].interface;
    const udev_interface_info_t *iinfo =
        interface < uep0_current_configuration->descriptors->bNumInterfaces
            ? uep0_current_configuration->interfaces[interface]
            : 0;
    if (iinfo)
    {
        const udev_alternate_setting_info_t *asinfo =
            &iinfo->alternate_settings[uep0_alternate_settings[interface]];
        const udev_endpoint_info_t *einfo = asinfo->endpoints[UEP_IDX(ep)];
        if (einfo)
        {
            return einfo;
        }
        einfo = iinfo->endpoints[UEP_IDX(ep)];
        if (einfo)
        {
            return einfo;
        }
    }

    return uep0_current_configuration->endpoints[UEP_IDX(ep)];
}

/**
 * \endcond
 */
