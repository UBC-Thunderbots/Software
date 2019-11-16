#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * \brief The size of a sector on an SD card.
 */
#define SD_SECTOR_SIZE 512U

/**
 * \brief The possible results of taking an action against the card.
 */
typedef enum
{
    /**
     * \brief The action completed successfully.
     */
    SD_STATUS_OK,

    /**
     * \brief The action failed because the card has not been initialized.
     */
    SD_STATUS_UNINITIALIZED,

    /**
     * \brief The action failed because no card is plugged in.
     */
    SD_STATUS_NO_CARD,

    /**
     * \brief The action failed because the attached card is working properly
     * but is incompatible with this system.
     */
    SD_STATUS_INCOMPATIBLE_CARD,

    /**
     * \brief The action failed because the card sent an illegal response.
     */
    SD_STATUS_ILLEGAL_RESPONSE,

    /**
     * \brief The action failed because a logical error occurred (e.g. invalid
     * parameter or address value or command sequence error).
     */
    SD_STATUS_LOGICAL_ERROR,

    /**
     * \brief The action failed because a CRC failed.
     */
    SD_STATUS_CRC_ERROR,

    /**
     * \brief The action failed because a command was sent that the card did
     * not recognize.
     */
    SD_STATUS_ILLEGAL_COMMAND,

    /**
     * \brief The card was in an unexpected state after the command is run.
     */
    SD_STATUS_ILLEGAL_STATE,

    /**
     * \brief The card encountered an internal error in its controller or
     * uncorrectable corruption in its memory array.
     */
    SD_STATUS_CARD_INTERNAL_ERROR,

    /**
     * \brief Command response timeout.
     */
    SD_STATUS_COMMAND_RESPONSE_TIMEOUT,

    /**
     * \brief The command’s argument was out of the allowed range for this
     * card.
     */
    SD_STATUS_OUT_OF_RANGE,

    /**
     * \brief The command’s address argument positions the first data block
     * misaligned to the card physical blocks.
     */
    SD_STATUS_ADDRESS_MISALIGN,

    /**
     * \brief Either the argument of a SET_BLOCKLEN command exceeds the maximum
     * value allowed for the card, or the previously defined block length is
     * illegal for the current command.
     */
    SD_STATUS_BLOCK_LEN_ERROR,

    /**
     * \brief An error in the sequence of erase commands occurred.
     */
    SD_STATUS_ERASE_SEQ_ERROR,

    /**
     * \brief An invalid selection of erase groups for erase occurred.
     */
    SD_STATUS_ERASE_PARAM,

    /**
     * \brief Attempt to program a write-protected block.
     */
    SD_STATUS_WP_VIOLATION,

    /**
     * \brief When set, signals that the card is locked by the host.
     */
    SD_STATUS_CARD_IS_LOCKED,

    /**
     * \brief Set when a sequence or password error has been detected in
     * lock/unlock card command.
     */
    SD_STATUS_LOCK_UNLOCK_FAILED,

    /**
     * \brief The CRC check of the previous command failed.
     */
    SD_STATUS_COM_CRC_ERROR,

    /**
     * \brief Card internal ECC was applied but failed to correct the data.
     */
    SD_STATUS_CARD_ECC_FAILED,

    /**
     * \brief A card error occurred, which is not related to the host command.
     */
    SD_STATUS_CC_ERROR,

    /**
     * \brief Generic card error related to the (and detected during) execution
     * of the last host command.
     */
    SD_STATUS_ERROR,

    /**
     * \brief CSD write error.
     */
    SD_STATUS_CSD_OVERWRITE,

    /**
     * \brief Set when only partial address space was erased due to existing
     * write.
     */
    SD_STATUS_WP_ERASE_SKIP,

    /**
     * \brief Command has been executed without using the internal ECC.
     */
    SD_STATUS_CARD_ECC_DISABLED,

    /**
     * \brief Erase sequence was cleared before executing because an out of
     * erase sequence command was received.
     */
    SD_STATUS_ERASE_RESET,

    /**
     * \brief Error in the sequence of the authentication process.
     */
    SD_STATUS_AKE_SEQ_ERROR,

    /**
     * \brief The card failed to finish its initialization procedure within one
     * second.
     */
    SD_STATUS_INIT_TIMEOUT,

    /**
     * \brief The card failed to deliver data in the appropriate amount of
     * time.
     */
    SD_STATUS_DATA_TIMEOUT,

    /**
     * \brief Data was received with an incorrect CRC.
     */
    SD_STATUS_DATA_CRC_ERROR,

    /**
     * \brief A start bit was missing on one or more data lines.
     */
    SD_STATUS_MISSING_START_BIT,

    /**
     * \brief A FIFO overrun or underrun occurred.
     */
    SD_STATUS_FIFO_ERROR,
} sd_status_t;

sd_status_t sd_init(void);
bool sd_is_hc(void);
uint32_t sd_sector_count(void);
sd_status_t sd_read(uint32_t sector, void *buffer);
sd_status_t sd_write(uint32_t sector, const void *data);
sd_status_t sd_erase(uint32_t sector, size_t count);
void sd_isr(void);
void sd_d0_exti_isr(void);

#endif
