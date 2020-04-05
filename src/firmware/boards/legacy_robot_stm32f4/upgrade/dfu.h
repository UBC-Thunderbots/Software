#ifndef UPGRADE_DFU_H
#define UPGRADE_DFU_H

/**
 * \ingroup UPGRADE
 *
 * \brief The maximum number of bytes the host is allowed to transfer in a
 * single DFU control transfer.
 */
#define UPGRADE_DFU_MAX_TRANSFER_SIZE 4096U

void upgrade_dfu_run(void);

#endif
