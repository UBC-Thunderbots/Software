/**
 * \addtogroup UPGRADE Firmware Upgrade Functions
 *
 * \brief These functions handle firmware and FPGA bitstream images.
 *
 *
 *
 * <h1>Overview</h1>
 *
 * There are two pieces of configuration data needed to fully operate a robot:
 * a microcontroller firmware image and an FPGA configuration bitstream. There
 * are also two pieces of nonvolatile storage on board the robot: the
 * microcontroller’s on-board Flash memory and the Secure Digital card. The
 * problems to solve are twofold: where to store each piece of configuration
 * data, and how to easily upgrade each piece of data.
 *
 * The microcontroller firmware is always stored on the microcontroller’s
 * internal Flash memory. The microcontroller executes code directly from
 * Flash, and (unless forced to do otherwise by its \c BOOT pins) does so
 * immediately from powerup. The \c BOOT pins can be adjusted to convince the
 * microcontroller to instead boot from a small internal non-rewritable ROM
 * which ships from the factory with a bootloader that understands a number of
 * protocols including USB Device Firmware Upgrade STMicroelectronics Extension
 * (DfuSe).
 *
 * The FPGA does not contain any nonvolatile storage, so its configuration
 * bitstream needs to be loaded in at each powerup. This is done by the
 * microcontroller sending the bitstream over the ICB, which shares pins with
 * the FPGA’s configuration interface.
 *
 *
 *
 * <h1>The Normal Boot Process</h1>
 *
 * When starting up normally, the microcontroller begins by executing its
 * firmware that exists in Flash. The SD card, meanwhile, does not contain a
 * microcontroller firmware image at all. Early in the boot process, the
 * firmware reads an FPGA bitstream from the SD card and delivers it to the
 * FPGA.
 *
 *
 *
 * <h1>Installing or Upgrading Microcontroller Code</h1>
 *
 * There are a number of paths to install or upgrade microcontroller code. As
 * explained in the Normal Boot Process section, normally, the microcontroller
 * starts up running firmware from its internal Flash memory. The objective,
 * then, is to get new code into that Flash memory. The following mechanisms
 * allow that to happen:
 *
 * <dl>
 * <dt>Serial Wire Debug</dt>
 * <dd>Serial Wire Debug is an interface for debugging ARM CPUs which can also
 * be used for loading code. The robot main boards bring out the Serial Wire
 * Debug pins from the microcontroller to a header, which can be attached to a
 * Serial Wire Debug adapter (for example, an STM32F4Discovery with its ST-Link
 * jumpers removed). Via OpenOCD, GDB’s \c load command can be used to install
 * new code on the microcontroller.
 *
 * This option is good for installing firmware on blank or bricked
 * microcontrollers, and is ideal if using an SWD cable for in-circuit
 * debugging anyway. However, it is cumbersome as it may (depending on the
 * state of the system) require holding the power switch in the Start position
 * to prevent the robot from powering down, and it requires starting up more
 * tools (OpenOCD and GDB).</dd>
 *
 * <dt>USB Device Firmware Upgrade STMicroelectronics Extension Bootloader</dt>
 * <dd>The STM32F4 ships with a small, non-rewritable ROM which is programmed
 * at the factory with a simple bootloader program. By raising the \c BOOT0 pin
 * to a high level at boot (which can be done with a switch on the main board),
 * the microcontroller starts running this bootloader instead of the usual
 * application code. A sufficiently recent version of the \c dfu-util program
 * (version 0.7 is adequate) can then be used to install new firmware into
 * Flash; the <code>make dfuse</code> command does this.
 *
 * This option is ideal for installing firmware on blank or bricked
 * microcontrollers as it requires only a USB cable and flipping the Boot
 * switch. However, it is cumbersome as it requires holding the power switch in
 * the Start position to prevent the robot from powering down.</dd>
 *
 * <dt>SD Card Ephemeral Image From Computer</dt>
 * <dd>As part of the boot process, the robot firmware checks whether there is
 * a firmware image on the SD card. If there is a valid, intact image (and it
 * is not identical to the firmware already in the microcontroller’s Flash), a
 * stub function is called to copy the new firmware from the SD card into
 * Flash, after which the microcontroller reboots in order to run the new
 * firmware.
 *
 * The robot’s usual SD card can thus be used to deploy new firmware. The \c
 * sdutil software tool’s \c image command can be used to install a firmware
 * image (in this case \c main.bin) onto the SD card, assuming the SD card is
 * attached to the computer. The \c ephemeral option should be used; this
 * option sets a flag in the firmware image that results in the robot deleting
 * the firmware from the SD card after the upgrade completes; this speeds up
 * future boots by avoiding the need to compare the firmware in Flash to the
 * firmware on the SD card.
 *
 * This option is ideal for bulk upgrading many already-working robots: simply
 * collect all the SD cards, install the new image on them, and return them to
 * their robots. Especially if robots are scattered, this is much easier than
 * bringing all the robots to a laptop or a laptop to all the robots, and
 * there’s no need to hold the power switch in Start.</dd>
 *
 * <dt>Gold Master SD Card Image From Computer</dt>
 * <dd>A slight variant on the SD Card Ephemeral Image option, rather than
 * copying the new firmware onto each robot’s SD card, a separate card could be
 * used just for firmware upgrade purposes. The new firmware could be placed on
 * the card using the \c sdutil tool’s \c image command’s \c permanent option
 * (instead of the \c ephemeral option), preventing robots from deleting the
 * firmware after install. The gold master SD card can then be passed from
 * robot to robot until all robots are upgraded, replacing the robot’s usual SD
 * card afterwards.
 *
 * When a robot is upgraded using a permanent image from SD card instead of an
 * ephemeral image, it powers down after the upgrade rather than rebooting,
 * since the original SD card needs to be reinserted before the robot is
 * used.
 *
 * This option is ideal for bulk upgrading many already-working robots one at a
 * time, as one might do during a competition: prepare the gold master SD card,
 * then, as each robot is taken off the field, quickly upgrade it and then
 * replace its original card. The advantage of this is that the person
 * preparing SD cards isn’t tied up waiting for all the robots to be available
 * to upgrade. The disadvantage is that only one robot can upgrade itself at a
 * time, whereas with the former option, all robots can be installing the new
 * image simultaneously.</dd>
 *
 * <dt>USB Device Firmware Upgrade</dt>
 * <dd>Robot firmware includes an implementation of the USB Device Firmware
 * Upgrade specification, a USB standard for delivering new firmware to
 * devices. The \c dfu-util program can be used to install firmware via this
 * method; the <code>make dfu</code> command run in the
 * <code>firmware/main</code> directory does this. Internally, the received
 * firmware is first written to the SD card as an ephemeral image, then copied
 * to Flash in the same manner as the SD Card Ephemeral Image method.
 *
 * This option is ideal for interactively upgrading a single robot, such as
 * during development and debugging of new firmware. It doesn’t require
 * removing the SD card for each upgrade, but it does require a USB connection
 * to the computer.</dd>
 * </dl>
 *
 *
 *
 * <h1>Installing or Upgrading FPGA Bitstream</h1>
 *
 * There are a number of paths to install or upgrade FPGA bitstream, though
 * fewer than for microcontroller firmware because the requirements are simpler
 * (no option is needed that works with a blank or bricked microcontroller,
 * since one of the microcontroller options documented in the previous section
 * can be used first). As explained in the Normal Boot Process section, the
 * FPGA bitstream is always read from SD card at startup. The objective, then,
 * is to get new bitstream onto the SD card. The following mechanisms allow
 * that to happen; note that all of them require microcontroller firmware to be
 * up and running in order to work, so a blank or bricked microcontroller must
 * first be recovered using one of the methods in the previous section:
 *
 * <dl>
 * <dt>SD Card Image From Computer</dt>
 * <dd>The \c sdutil tool’s \c image command can write a new FPGA bitstream
 * onto an SD card attached to the computer. The \c permanent option must be
 * used; ephemeral FPGA bitstreams are not possible since the bitstream
 * is always loaded from SD card on every boot.
 *
 * This option is ideal for bulk upgrading many robots: simply collect all the
 * SD cards, install the new bitstream on them, and return them to their
 * robots. Especially if robots are scattered, this is much easier than
 * bringing all the robots to a laptop or a laptop to all the robots.</dd>
 *
 * <dt>USB Device Firmware Upgrade</dt>
 * <dd>The Device Firmware Upgrade implementation mentioned in the previous
 * section allows choosing a second alternate setting for the interface, which
 * allows uploading an FPGA bitstream instead of microcontroller firmware. The
 * \c dfu-util program is again suitable; the <code>make dfu</code> command run
 * in the <code>vhdl</code> directory does this. The received bitstream is
 * written to the SD card as a permanent image, after which the robot reboots
 * and will use the new bitstream.
 *
 * This option is ideal for interactively upgrading a single robot, such as
 * during development and debugging of new VHDL. It doesn’t require removing
 * the SD card for each upgrade, but it does require a USB connection to the
 * computer.</dd>
 * </dl>
 *
 *
 *
 * <h1>Detailed Organization of the Secure Digital Card</h1>
 *
 * The first eight megabytes of the SD card are reserved for configuration
 * data, split into two four-megabyte blocks. The first block is used for
 * microcontroller firmware, while the second block is used for FPGA bitstream.
 * Each block is either blank or else comprises a header, which is padded to
 * fill one 512-byte sector, followed by the configuration data.
 *
 * The header is laid out as follows, with all values being stored in
 * little-endian byte order:
 *
 * <table>
 * <tr><th>Length (bytes)</th><th>Description</th></tr>
 * <tr><td>4</td><td>Magic Number</td></tr>
 * <tr><td>4</td><td>Flags</td></tr>
 * <tr><td>4</td><td>Length of firmware image (bytes)</td></tr>
 * <tr><td>4</td><td>CRC32 of firmware image</td></tr>
 * </table>
 *
 * The magic number is 0x1453CABE for the first (microcontroller firmware)
 * block, and 0x74E4BCC5 for the second (FPGA bitstream) block.
 *
 * The flags field is a bit field. Bits are allocated as follows:
 *
 * <table>
 * <tr><th>Bit Index</th><th>Applicable To</th><th>Description</th></tr>
 * <tr><td>0</td><td>Firmware Only</td><td>\em Ephemeral: The image on the SD card is
 * erased at system startup if identical firmware is present in the microcontroller’s
 * on-board Flash.</td></tr>
 * </table>
 *
 * Any bits that are not specified above are reserved and must be zero. Any
 * bits not applicable to the relevant block must also be zero.
 *
 * All space after the first eight megabytes is used by the logging module to
 * store logs.
 *
 * \{
 */
#include "dfu.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <crc32.h>
#include <gpio.h>
#include <minmax.h>
#include <registers/iwdg.h>
#include <semphr.h>
#include <stack.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unused.h>
#include <usb.h>
#include <usb_dfu.h>

#include "common.h"
#include "internal.h"
#include "io/dma.h"
#include "io/pins.h"
#include "io/sdcard.h"
#include "main.h"
#include "util/constants.h"

/**
 * \brief The possible types of jobs the writeout task performs.
 */
typedef enum
{
    /**
     * \brief The writeout task prepare to write data into a storage area.
     *
     * The \ref upgrade_dfu_writeout_job_t::integer "integer" parameter is the
     * index (0 or 1) of the storage area to write to.
     *
     * The writeout task will initialize its pointers in preparation for
     * writing to the specified storage area.
     *
     * This job does not send a response.
     */
    UPGRADE_WRITEOUT_JOB_TYPE_PREPARE,

    /**
     * \brief Provides data to the writeout task to write to the storage area.
     *
     * The \ref upgrade_dfu_writeout_job_t::pointer "pointer" parameter is a
     * pointer to the data to write (which will be freed when no longer
     * needed), while the \ref upgrade_dfu_writeout_job_t::integer "integer"
     * parameter is the size of the data.
     *
     * The writeout task will collect the data until it has enough to make a
     * full SD sector (or until it receives an UPGRADE_WRITEOUT_JOB_TYPE_COMMIT
     * message) and then write the data to the SD card.
     *
     * This job sends a response when as many sectors as possible have been
     * written and the remaining data has been collected. The response’s \ref
     * upgrade_dfu_writeout_job_t::integer "integer" parameter is nonzero if
     * all writes so far have succeeded, or zero if a write has failed. In the
     * latter case, the requester’s attention is directed to \ref sd_status for
     * more details.
     */
    UPGRADE_WRITEOUT_JOB_TYPE_DATA,

    /**
     * \brief Instructs the writeout task to finish the data it has been
     * provided and write the header.
     *
     * The writeout task will write any data it has collected that is less than
     * a full SD sector, then write a header at the start of the storage area.
     *
     * This job sends a response when all data and the header have been
     * written. The response’s \ref upgrade_dfu_writeout_job_t::integer
     * "integer" parameter is nonzero if all writes have succeeded, or zero if
     * a write has failed. In the latter case, the requester’s attention is
     * directed to \ref sd_status for more details.
     */
    UPGRADE_WRITEOUT_JOB_TYPE_COMMIT,

    /**
     * \brief Instructs the writeout task to terminate.
     *
     * This job sends a response when all prior jobs are complete and the
     * writeout task is about to terminate.
     */
    UPGRADE_WRITEOUT_JOB_TYPE_EXIT,
} upgrade_dfu_writeout_job_type_t;

/**
 * \brief The type of a queue element sent to or from the writeout task.
 */
typedef struct
{
    /**
     * \brief The type of job.
     */
    upgrade_dfu_writeout_job_type_t type;

    /**
     * \brief A pointer parameter associated with the job.
     */
    void *pointer;

    /**
     * \brief An integer parameter associated with the job.
     */
    size_t integer;
} upgrade_dfu_writeout_job_t;

/**
 * \brief Objects used to communicate with the writeout task.
 */
static struct
{
    /**
     * \brief A queue onto which job requests are pushed to send to the
     * writeout queue.
     */
    QueueHandle_t request_queue;

    /**
     * \brief A queue onto which job status responses are pushed as the
     * writeout queue finishes jobs.
     */
    QueueHandle_t response_queue;
} upgrade_dfu_writeout_iface;

/**
 * \brief The writeout task.
 */
static void upgrade_dfu_writeout_task(void *UNUSED(param))
{
    bool prepared = false, running = true, erased = false;
    size_t buffer_used = 0;
    uint32_t byte_count, crc, start_sector = 0, next_sector = 0, header_magic,
                              header_flags;
    char *buffer = upgrade_common_get_sector_dma_buffer();

    while (running)
    {
        upgrade_dfu_writeout_job_t job;
        xQueueReceive(upgrade_dfu_writeout_iface.request_queue, &job, portMAX_DELAY);
        bool reply;
        switch (job.type)
        {
            case UPGRADE_WRITEOUT_JOB_TYPE_PREPARE:
                assert((job.integer == 0) || (job.integer == 1));
                prepared     = true;
                reply        = false;
                erased       = false;
                buffer_used  = 0;
                byte_count   = 0;
                crc          = CRC32_EMPTY;
                start_sector = (job.integer == 0) ? UPGRADE_FW_FIRST_SECTOR
                                                  : UPGRADE_FPGA_FIRST_SECTOR;
                next_sector  = start_sector + 1;
                header_magic = (job.integer == 0) ? UPGRADE_FW_MAGIC : UPGRADE_FPGA_MAGIC;
                header_flags = (job.integer == 0) ? 0x00000001 : 0x00000000;
                break;

            case UPGRADE_WRITEOUT_JOB_TYPE_DATA:
                assert(job.pointer || !job.integer);
                assert(prepared);
                reply = true;
                if (job.pointer)
                {
                    bool ok = true;

                    // Erase the storage area, if we haven’t already done so.
                    if (!erased)
                    {
                        ok = sd_erase(start_sector, UPGRADE_SD_AREA_SECTORS) ==
                             SD_STATUS_OK;
                        erased = ok;
                    }

                    // Grab temporaries to walk through the received data.
                    const uint8_t *data = job.pointer;
                    size_t left         = job.integer;

                    // Update the running records.
                    byte_count += left;
                    crc = crc32_be(data, left, crc);

                    // If there is data in the buffer, combine it with the
                    // beginning of the new data to fill the buffer more.
                    if (ok && buffer_used)
                    {
                        size_t to_copy = MIN(SD_SECTOR_SIZE - buffer_used, left);
                        memcpy(buffer + buffer_used, data, to_copy);
                        buffer_used += to_copy;
                        data += to_copy;
                        left -= to_copy;
                    }

                    // If the previous step completely filled the buffer, flush
                    // it.
                    if (ok && (buffer_used == SD_SECTOR_SIZE))
                    {
                        // The buffer is full, so write it out.
                        ok          = sd_write(next_sector++, buffer) == SD_STATUS_OK;
                        buffer_used = 0;
                    }

                    // Write out any whole sectors in the new data.
                    while (ok && (left >= SD_SECTOR_SIZE))
                    {
                        // There is a whole sector ready to write, so write it.
                        // Don’t just use the data as is; it may not be
                        // properly aligned for DMA.
                        memcpy(buffer, data, SD_SECTOR_SIZE);
                        ok = sd_write(next_sector++, buffer) == SD_STATUS_OK;
                        data += SD_SECTOR_SIZE;
                        left -= SD_SECTOR_SIZE;
                    }

                    // If there is a partial sector left at the end of the
                    // input data, buffer it.
                    if (ok && left)
                    {
                        // There is some data left, so buffer it.
                        assert(left < SD_SECTOR_SIZE);
                        memcpy(buffer, data, left);
                        buffer_used = left;
                        data += left;
                        left = 0;
                    }

                    // Clean up and build a reply.
                    free(job.pointer);
                    job.pointer = 0;
                    job.integer = ok ? 1 : 0;
                }
                else
                {
                    // A request with no data is pointless but successful.
                    job.integer = 1;
                }
                break;

            case UPGRADE_WRITEOUT_JOB_TYPE_COMMIT:
                assert(prepared);
                reply = true;
                {
                    bool ok = true;

                    // It is possible, for a sufficiently small image, that we
                    // have not yet erased the card. If so, do it now.
                    if (!erased)
                    {
                        ok = sd_erase(start_sector, UPGRADE_SD_AREA_SECTORS) ==
                             SD_STATUS_OK;
                        erased = ok;
                    }

                    // There may be a residual partial sector in the buffer. If
                    // so, flush it.
                    if (ok && buffer_used)
                    {
                        memset(buffer + buffer_used, 0, SD_SECTOR_SIZE - buffer_used);
                        ok = sd_write(next_sector++, buffer) == SD_STATUS_OK;
                    }

                    // Write the area header.
                    if (ok)
                    {
                        memset(buffer, 0, SD_SECTOR_SIZE);
                        memcpy(&buffer[0], &header_magic, 4);
                        memcpy(&buffer[4], &header_flags, 4);
                        memcpy(&buffer[8], &byte_count, 4);
                        memcpy(&buffer[12], &crc, 4);
                        ok = sd_write(start_sector, buffer) == SD_STATUS_OK;
                    }

                    // Clean up and build a reply.
                    prepared    = false;
                    job.pointer = 0;
                    job.integer = ok ? 1 : 0;
                }
                break;

            case UPGRADE_WRITEOUT_JOB_TYPE_EXIT:
                reply       = true;
                running     = false;
                job.pointer = 0;
                job.integer = 0;
                break;

            default:
                abort();
        }
        if (reply)
        {
            xQueueSend(upgrade_dfu_writeout_iface.response_queue, &job, portMAX_DELAY);
        }
    }

    vTaskSuspend(0);
}

/**
 * \brief Initializes the writeout task.
 *
 * This function is called by the supervisor as part of bringing up DFU mode.
 */
static void upgrade_dfu_writeout_init(void)
{
    static StaticQueue_t request_queue_storage, response_queue_storage;
    static uint8_t request_queue_buffer[4 * sizeof(upgrade_dfu_writeout_job_t)],
        response_queue_buffer[4 * sizeof(upgrade_dfu_writeout_job_t)];
    upgrade_dfu_writeout_iface.request_queue =
        xQueueCreateStatic(4U, sizeof(upgrade_dfu_writeout_job_t), request_queue_buffer,
                           &request_queue_storage);
    upgrade_dfu_writeout_iface.response_queue =
        xQueueCreateStatic(4U, sizeof(upgrade_dfu_writeout_job_t), response_queue_buffer,
                           &response_queue_storage);
    static StaticTask_t upgrade_dfu_writeout_task_tcb;
    STACK_ALLOCATE(upgrade_dfu_writeout_task_stack, 4096);
    xTaskCreateStatic(&upgrade_dfu_writeout_task, "upg-writeout",
                      sizeof(upgrade_dfu_writeout_task_stack) /
                          sizeof(*upgrade_dfu_writeout_task_stack),
                      0, PRIO_TASK_UPGRADE_WRITEOUT, upgrade_dfu_writeout_task_stack,
                      &upgrade_dfu_writeout_task_tcb);
}

/**
 * \brief Shuts down the writeout task.
 *
 * This function is called by the supervisor as part of exiting DFU mode.
 */
static void upgrade_dfu_writeout_deinit(void)
{
    upgrade_dfu_writeout_job_t job = {UPGRADE_WRITEOUT_JOB_TYPE_EXIT, 0, 0};
    xQueueSend(upgrade_dfu_writeout_iface.request_queue, &job, portMAX_DELAY);
    do
    {
        xQueueReceive(upgrade_dfu_writeout_iface.response_queue, &job, portMAX_DELAY);
    } while (job.type != UPGRADE_WRITEOUT_JOB_TYPE_EXIT);
}

/**
 * \brief Submits a job to the writeout task.
 *
 * \param[in] type the type of job to submit
 * \param[in] pointer the pointer parameter of the job
 * \param[in] integer the integer parameter of the job
 */
static void upgrade_dfu_writeout_submit(upgrade_dfu_writeout_job_type_t type,
                                        void *pointer, size_t integer)
{
    upgrade_dfu_writeout_job_t job;
    job.type    = type;
    job.pointer = pointer;
    job.integer = integer;
    xQueueSend(upgrade_dfu_writeout_iface.request_queue, &job, portMAX_DELAY);
}

/**
 * \brief Polls for a response from the writeout task.
 *
 * \param[out] job a structure to fill with data if a response is available
 * \retval true a response was popped and returned
 * \retval false no response was available
 */
static bool upgrade_dfu_writeout_pop_status(upgrade_dfu_writeout_job_t *job)
{
    return xQueueReceive(upgrade_dfu_writeout_iface.response_queue, job, 0) == pdTRUE;
}



/**
 * \brief The state data for USB DFU operations.
 */
static struct
{
    /**
     * \brief A semaphore signalling that it’s time to exit DFU mode and
     * reboot.
     *
     * This semaphore is given by the USB stack internal task when it sees
     * reset signalling.
     *
     * This semaphore is taken by the supervisor task after setting up DFU
     * operations, allowing it to start shutting down DFU mode.
     */
    SemaphoreHandle_t done_sem;

    /**
     * \brief Which storage area is currently being accessed.
     */
    unsigned int area;

    /**
     * \brief Information used by an upload operation.
     */
    union
    {
        /**
         * \brief The next byte to send, for a firmware upload.
         */
        const char *fw_next_byte;

        /**
         * \brief Information used for an FPGA upload.
         */
        struct
        {
            /**
             * \brief The number of bytes in the complete FPGA bitstream.
             */
            size_t length;

            /**
             * \brief The position of the next byte to read, measured from the
             * start of the FPGA bitstream.
             */
            size_t pos;

            /**
             * \brief The expected CRC of the complete bitstream image.
             */
            uint32_t expected_crc;

            /**
             * \brief The CRC of the part of the bitstream image that has been
             * read so far.
             */
            uint32_t current_crc;
        } fpga;
    } upload_info;

    /**
     * \brief The DFU status block.
     */
    usb_dfu_status_block_t status_block;

    /**
     * \brief Whether a data or commit job has been pushed and is pending a
     * response from the writeout task.
     */
    bool job_response_pending;
} upgrade_dfu_state;

static bool upgrade_dfu_control_handler_dnload(const usb_setup_packet_t *pkt)
{
    // Sanity check that the host has not tried to send too much data.
    if (pkt->wLength > UPGRADE_DFU_MAX_TRANSFER_SIZE)
    {
        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
        return false;
    }

    // Dispatch based on state.
    switch (upgrade_dfu_state.status_block.bState)
    {
        case USB_DFU_STATE_DFU_IDLE:
            // In this state, we do not accept zero-length blocks (USB DFU spec
            // v1.1 section A.2.3).
            if (!pkt->wLength)
            {
                upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
                upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
                return false;
            }

            // It is possible that a previous DFU operation left a writeout job
            // in progress and the operation was then ripped out from under us
            // by e.g. a SET INTERFACE request. In that case,
            // job_response_pending will be true here. Then try to get the job
            // status, but only if it’s already finished; if it’s still in
            // progress, don’t block the USB, but just return an error.
            if (upgrade_dfu_state.job_response_pending)
            {
                upgrade_dfu_writeout_job_t job;
                if (upgrade_dfu_writeout_pop_status(&job))
                {
                    upgrade_dfu_state.job_response_pending = false;
                }
                else
                {
                    upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
                    upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_ERASE;
                    return false;
                }
            }

            // Fall through.

        case USB_DFU_STATE_DFU_DNLOAD_IDLE:
            if (pkt->wLength)
            {
                // The host wants to send us some data to write out.
                uint8_t *buffer = malloc(pkt->wLength);
                if (uep0_data_read(buffer))
                {
                    upgrade_dfu_writeout_submit(UPGRADE_WRITEOUT_JOB_TYPE_DATA, buffer,
                                                pkt->wLength);
                    upgrade_dfu_state.job_response_pending = true;
                    // Submitting the block transitions to DNLOAD-SYNC state.
                    // The host must poll to discover when the block is done.
                    upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_DNLOAD_SYNC;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                // The host wants us to start manifestation. First transition
                // to MANIFEST-SYNC, at which point a GETSTATUS request starts
                // actual manifestation.
                upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_MANIFEST_SYNC;
                return true;
            }
            break;

        default:
            // This request is not acceptable in other states.
            upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
            upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
            return false;
    }
}

static bool upgrade_dfu_control_handler_upload(const usb_setup_packet_t *pkt)
{
    // Sanity check that the host is not asking for too much data or an empty transfer.
    if (!pkt->wLength || (pkt->wLength > UPGRADE_DFU_MAX_TRANSFER_SIZE))
    {
        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
        return false;
    }

    // Dispatch based on state.
    switch (upgrade_dfu_state.status_block.bState)
    {
        case USB_DFU_STATE_DFU_IDLE:
            // Set up the upload from the beginning.
            if (upgrade_dfu_state.area == 0)
            {
                // Uploading firmware.
                upgrade_dfu_state.upload_info.fw_next_byte = (const char *)0x08000000U;
            }
            else
            {
                // Uploading FPGA bitstream; read the header to figure out
                // what’s going on.
                dma_memory_handle_t header_handle = dma_alloc(SD_SECTOR_SIZE);
                uint32_t *header                  = dma_get_buffer(header_handle);
                bool ok = sd_read(UPGRADE_FPGA_FIRST_SECTOR, header) == SD_STATUS_OK;
                ok      = ok && (header[0] == UPGRADE_FPGA_MAGIC);
                upgrade_dfu_state.upload_info.fpga.length       = header[2];
                upgrade_dfu_state.upload_info.fpga.pos          = 0;
                upgrade_dfu_state.upload_info.fpga.expected_crc = header[3];
                upgrade_dfu_state.upload_info.fpga.current_crc  = CRC32_EMPTY;
                dma_free(header_handle);
                if (!ok)
                {
                    upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
                    upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_VERIFY;
                    return false;
                }
            }
            upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_UPLOAD_IDLE;
            // Fall through.

        case USB_DFU_STATE_DFU_UPLOAD_IDLE:
            // Upload some data.
            if (upgrade_dfu_state.area == 0)
            {
                // Uploading firmware.
                extern const char linker_data_lma, linker_data_size;
                const char *start = (const char *)0x08000000U;
                size_t total_length =
                    (&linker_data_lma - start) + (size_t)&linker_data_size;
                const char *end = start + total_length;
                size_t left     = end - upgrade_dfu_state.upload_info.fw_next_byte;
                bool ok =
                    uep0_data_write(upgrade_dfu_state.upload_info.fw_next_byte, left);
                upgrade_dfu_state.upload_info.fw_next_byte += left;
                if (left < pkt->wLength)
                {
                    // We sent a short transfer signalling end of firmware
                    // image. Note, this is not precisely equivalent to
                    // fw_next_byte reaching end; the latter would also be
                    // satisfied by a perfectly sized transfer being completely
                    // filled.
                    upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_IDLE;
                }
                return ok;
            }
            else
            {
                // Uploading FPGA bitstream.
                size_t bytes_left = upgrade_dfu_state.upload_info.fpga.length -
                                    upgrade_dfu_state.upload_info.fpga.pos;
                size_t bytes_this = MIN(bytes_left, pkt->wLength);
                if (bytes_this)
                {
                    // Read all the sectors included in the needed range of bytes.
                    uint32_t first_sector =
                        upgrade_dfu_state.upload_info.fpga.pos / SD_SECTOR_SIZE +
                        UPGRADE_FPGA_FIRST_SECTOR + 1;
                    uint32_t last_sector =
                        (upgrade_dfu_state.upload_info.fpga.pos + bytes_this - 1) /
                            SD_SECTOR_SIZE +
                        UPGRADE_FPGA_FIRST_SECTOR + 1;
                    size_t sector_count = last_sector - first_sector + 1;
                    dma_memory_handle_t buffer_handle =
                        dma_alloc(sector_count * SD_SECTOR_SIZE);
                    char *buffer = dma_get_buffer(buffer_handle);
                    bool ok      = true;
                    for (size_t i = 0; i < sector_count; ++i)
                    {
                        ok = ok && sd_read(first_sector + i,
                                           buffer + i * SD_SECTOR_SIZE) == SD_STATUS_OK;
                    }
                    size_t first_byte =
                        upgrade_dfu_state.upload_info.fpga.pos % SD_SECTOR_SIZE;
                    upgrade_dfu_state.upload_info.fpga.pos += bytes_this;
                    upgrade_dfu_state.upload_info.fpga.current_crc =
                        crc32_be(buffer + first_byte, bytes_this,
                                 upgrade_dfu_state.upload_info.fpga.current_crc);
                    if (bytes_this != pkt->wLength)
                    {
                        // We got to the end of the bitstream and are preparing
                        // to send a short transfer. This signals the end of
                        // the image and returns to idle state.
                        upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_IDLE;
                    }
                    if (upgrade_dfu_state.upload_info.fpga.pos ==
                        upgrade_dfu_state.upload_info.fpga.length)
                    {
                        // We got to the end of the bitstream (whether or not
                        // we are going to send a short transfer), so check
                        // CRC.
                        ok = ok && (upgrade_dfu_state.upload_info.fpga.current_crc ==
                                    upgrade_dfu_state.upload_info.fpga.expected_crc);
                    }
                    ok = ok && uep0_data_write(buffer + first_byte, bytes_this);
                    if (!ok)
                    {
                        upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_ERROR;
                        upgrade_dfu_state.status_block.bStatus =
                            USB_DFU_STATUS_ERR_VERIFY;
                    }
                    dma_free(buffer_handle);
                    return ok;
                }
                else
                {
                    // No more bytes to send; send a ZLP and exit this state.
                    upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_IDLE;
                    return uep0_data_write(0, 0);
                }
            }

        default:
            // This request is not acceptable in other states.
            upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
            upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
            return false;
    }
}

static void upgrade_dfu_poststatus_manifest_sync(void)
{
    // Detach from the bus, because we advertise that we will detach on exiting
    // DFU mode.
    udev_detach();

    // Inform the supervisor to exit DFU mode and reboot.
    xSemaphoreGive(upgrade_dfu_state.done_sem);
}

static bool upgrade_dfu_control_handler_getstatus(const usb_setup_packet_t *pkt)
{
    // Sanity check.
    if ((pkt->wValue != 0) || (pkt->wLength != sizeof(upgrade_dfu_state.status_block)))
    {
        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
        return false;
    }

    // Dispatch based on state.
    switch (upgrade_dfu_state.status_block.bState)
    {
        case USB_DFU_STATE_DFU_DNLOAD_SYNC:
            // GETSTATUS checks whether the job is complete and transitions to
            // either DNBUSY or DNLOAD-IDLE.
            assert(upgrade_dfu_state.job_response_pending);
            {
                upgrade_dfu_writeout_job_t job;
                if (upgrade_dfu_writeout_pop_status(&job))
                {
                    // Job is done.
                    upgrade_dfu_state.job_response_pending = false;
                    if (job.integer)
                    {
                        // Job was successful.
                        upgrade_dfu_state.status_block.bState =
                            USB_DFU_STATE_DFU_DNLOAD_IDLE;
                    }
                    else
                    {
                        // Job failed.
                        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
                        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_PROG;
                    }
                    return uep0_data_write(&upgrade_dfu_state.status_block,
                                           sizeof(upgrade_dfu_state.status_block));
                }
                else
                {
                    // Job is not done yet.
                    upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_DNBUSY;
                    bool ret = uep0_data_write(&upgrade_dfu_state.status_block,
                                               sizeof(upgrade_dfu_state.status_block));
                    // As per USB DFU spec v1.1 section A.2.5, the device
                    // transitions automatically from DNBUSY back to
                    // DNLOAD-SYNC after a poll timeout has expired, and the
                    // host is not permitted to send any traffic to the device
                    // until then. Rather than actually counting time, just
                    // change back to DNLOAD-SYNC immediately instead. The host
                    // can poll whenever it wants, and will get updated
                    // information on whether the job has finished.
                    upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_DNLOAD_SYNC;
                    return ret;
                }
            }

        case USB_DFU_STATE_DFU_MANIFEST_SYNC:
            // GETSTATUS starts manifestation and transitions to MANIFEST.
            assert(!upgrade_dfu_state.job_response_pending);
            upgrade_dfu_writeout_submit(UPGRADE_WRITEOUT_JOB_TYPE_COMMIT, 0, 0);
            upgrade_dfu_state.job_response_pending = true;
            upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_MANIFEST;

            // We are intolerant during manifestation and advertise intent to
            // detach, so set a post-status callback that will detach and
            // initiate DFU exit and reboot as soon as the control transfer
            // completes.
            uep0_set_poststatus(&upgrade_dfu_poststatus_manifest_sync);

            return uep0_data_write(&upgrade_dfu_state.status_block,
                                   sizeof(upgrade_dfu_state.status_block));

        case USB_DFU_STATE_DFU_IDLE:
        case USB_DFU_STATE_DFU_DNLOAD_IDLE:
        case USB_DFU_STATE_DFU_UPLOAD_IDLE:
            // GETSTATUS has no side effects, but is acceptable and returns the
            // current status.
            return uep0_data_write(&upgrade_dfu_state.status_block,
                                   sizeof(upgrade_dfu_state.status_block));

        default:
            // This request is not acceptable in other states.
            upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
            upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
            return false;
    }
}

static bool upgrade_dfu_control_handler_clrstatus(const usb_setup_packet_t *pkt)
{
    // Sanity check.
    if ((pkt->wValue != 0) || (pkt->wLength != 0) ||
        (upgrade_dfu_state.status_block.bState != USB_DFU_STATE_DFU_ERROR))
    {
        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
        return false;
    }

    // Clear the error.
    upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_IDLE;
    upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_OK;
    return true;
}

static bool upgrade_dfu_control_handler_getstate(const usb_setup_packet_t *pkt)
{
    // Sanity check.
    if ((pkt->wValue != 0) || (pkt->wLength != 1))
    {
        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
        return false;
    }

    // Return the state.
    uint8_t state = upgrade_dfu_state.status_block.bState;
    return uep0_data_write(&state, 1);
}

static bool upgrade_dfu_control_handler_abort(const usb_setup_packet_t *pkt)
{
    // Sanity check.
    if ((pkt->wValue != 0) || (pkt->wLength != 0))
    {
        upgrade_dfu_state.status_block.bState  = USB_DFU_STATE_DFU_ERROR;
        upgrade_dfu_state.status_block.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
        return false;
    }

    // Dispatch based on state.
    switch (upgrade_dfu_state.status_block.bState)
    {
        case USB_DFU_STATE_DFU_IDLE:
        case USB_DFU_STATE_DFU_DNLOAD_IDLE:
        case USB_DFU_STATE_DFU_UPLOAD_IDLE:
            // Return to DFU-IDLE.
            upgrade_dfu_state.status_block.bState = USB_DFU_STATE_DFU_IDLE;
            return true;

        default:
            // This request is not acceptable in other states.
            return false;
    }
}

static bool upgrade_dfu_control_handler(const usb_setup_packet_t *pkt)
{
    if (pkt->bmRequestType.type == USB_CTYPE_CLASS)
    {
        switch (pkt->bRequest)
        {
            case USB_DFU_CREQ_DNLOAD:
                return upgrade_dfu_control_handler_dnload(pkt);
            case USB_DFU_CREQ_UPLOAD:
                return upgrade_dfu_control_handler_upload(pkt);
            case USB_DFU_CREQ_GETSTATUS:
                return upgrade_dfu_control_handler_getstatus(pkt);
            case USB_DFU_CREQ_CLRSTATUS:
                return upgrade_dfu_control_handler_clrstatus(pkt);
            case USB_DFU_CREQ_GETSTATE:
                return upgrade_dfu_control_handler_getstate(pkt);
            case USB_DFU_CREQ_ABORT:
                return upgrade_dfu_control_handler_abort(pkt);
            default:
                return false;
        }
    }
    return false;
}

static void upgrade_dfu_on_enter_common(unsigned int area)
{
    // On entering a configuration, signal the writeout task to prepare to
    // operate on the specified memory area.
    upgrade_dfu_writeout_submit(UPGRADE_WRITEOUT_JOB_TYPE_PREPARE, 0, area);

    // Record, locally, which area we are accessing.
    upgrade_dfu_state.area = area;

    // Set up the DFU status block.
    upgrade_dfu_state.status_block.bStatus       = USB_DFU_STATUS_OK;
    upgrade_dfu_state.status_block.bwPollTimeout = 1;
    upgrade_dfu_state.status_block.bState        = USB_DFU_STATE_DFU_IDLE;
    upgrade_dfu_state.status_block.iString       = 0;
}

static void upgrade_dfu_on_enter_fw(void)
{
    upgrade_dfu_on_enter_common(0);
}

static void upgrade_dfu_on_enter_fpga(void)
{
    upgrade_dfu_on_enter_common(1);
}

/**
 * \brief Initializes the DFU layer.
 */
static void upgrade_dfu_init(void)
{
    static StaticSemaphore_t done_sem_storage;
    upgrade_dfu_state.done_sem = xSemaphoreCreateBinaryStatic(&done_sem_storage);
    upgrade_dfu_state.job_response_pending = false;
}

/**
 * \brief Shuts down the DFU layer.
 */
static void upgrade_dfu_deinit(void) {}



/**
 * \brief The type of the DFU-mode configuration descriptor and associated
 * auxiliary descriptors.
 */
typedef struct __attribute__((packed))
{
    /**
     * \brief The configuration descriptor.
     */
    usb_configuration_descriptor_t config;

    /**
     * \brief The interface alternate setting for upgrading microcontroller
     * firmware.
     */
    usb_interface_descriptor_t fw_dfu_interface;

    /**
     * \brief The interface alternate setting for upgrading the FPGA bitstream.
     */
    usb_interface_descriptor_t fpga_dfu_interface;

    /**
     * \brief The DFU functional descriptor.
     */
    usb_dfu_functional_descriptor_t dfu_functional;
} config_descriptor_t;

/**
 * \brief The DFU-mode configuration descriptor and associated auxiliary
 * descriptors.
 */
static const config_descriptor_t UPGRADE_CONFIG_DESCRIPTOR = {
    .config =
        {
            .bLength             = sizeof(usb_configuration_descriptor_t),
            .bDescriptorType     = USB_DTYPE_CONFIGURATION,
            .wTotalLength        = sizeof(UPGRADE_CONFIG_DESCRIPTOR),
            .bNumInterfaces      = 1U,
            .bConfigurationValue = 1U,
            .iConfiguration      = 0U,
            .bmAttributes =
                {
                    .remoteWakeup = 0U,
                    .selfPowered  = 1U,
                    .one          = 1U,
                },
            .bMaxPower = 50U,
        },
    .fw_dfu_interface =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = 0U,
            .bAlternateSetting  = 0U,
            .bNumEndpoints      = 0U,
            .bInterfaceClass    = USB_DFU_CLASS_APPLICATION_SPECIFIC,
            .bInterfaceSubClass = USB_DFU_SUBCLASS_DFU,
            .bInterfaceProtocol = USB_DFU_PROTOCOL_DFU,
            .iInterface         = STRING_INDEX_DFU_FW,
        },
    .fpga_dfu_interface =
        {
            .bLength            = sizeof(usb_interface_descriptor_t),
            .bDescriptorType    = USB_DTYPE_INTERFACE,
            .bInterfaceNumber   = 0U,
            .bAlternateSetting  = 1U,
            .bNumEndpoints      = 0U,
            .bInterfaceClass    = USB_DFU_CLASS_APPLICATION_SPECIFIC,
            .bInterfaceSubClass = USB_DFU_SUBCLASS_DFU,
            .bInterfaceProtocol = USB_DFU_PROTOCOL_DFU,
            .iInterface         = STRING_INDEX_DFU_FPGA,
        },
    .dfu_functional =
        {
            .bLength         = sizeof(usb_dfu_functional_descriptor_t),
            .bDescriptorType = USB_DFU_DTYPE_FUNCTIONAL,
            .bmAttributes =
                {
                    .bitCanDnload             = 1U,
                    .bitCanUpload             = 1U,
                    .bitManifestationTolerant = 0U,
                    .bitWillDetach            = 1U,
                },
            .wDetachTimeout = 0U,
            .wTransferSize  = UPGRADE_DFU_MAX_TRANSFER_SIZE,
            .bcdDFUVersion  = 0x0110U,
        },
};

static const udev_interface_info_t UPGRADE_DFU_INTF = {
    .control_handler = 0,
    .endpoints       = {0, 0, 0, 0, 0, 0},
    .alternate_settings =
        {
            {
                .can_enter       = 0,
                .on_enter        = &upgrade_dfu_on_enter_fw,
                .on_exit         = 0,
                .control_handler = &upgrade_dfu_control_handler,
                .endpoints       = {0, 0, 0, 0, 0, 0},
            },
            {
                .can_enter       = 0,
                .on_enter        = &upgrade_dfu_on_enter_fpga,
                .on_exit         = 0,
                .control_handler = &upgrade_dfu_control_handler,
                .endpoints       = {0, 0, 0, 0, 0, 0},
            },
        },
};

static const udev_config_info_t UPGRADE_USB_CONFIGURATION = {
    .can_enter           = 0,
    .on_enter            = 0,
    .on_exit             = 0,
    .control_handler     = 0,
    .descriptors         = &UPGRADE_CONFIG_DESCRIPTOR.config,
    .transmit_fifo_words = {16, 16, 16},
    .endpoints           = {0, 0, 0, 0, 0, 0},
    .interfaces          = {&UPGRADE_DFU_INTF},
};

static const udev_info_t UPGRADE_USB_INFO = {
    .flags =
        {
            .vbus_sensing        = 1,
            .minimize_interrupts = 0,
            .self_powered        = 1,
        },
    .internal_task_priority   = PRIO_TASK_USB,
    .internal_task_stack_size = 1024U,
    .receive_fifo_words       = 10U /* SETUP packets */ + 1U /* Global OUT NAK status */ +
                          ((64U / 4U) + 1U) * 2U /* Packets */ +
                          4U /* Transfer complete status */,
    .device_descriptor =
        {
            .bLength            = sizeof(usb_device_descriptor_t),
            .bDescriptorType    = USB_DTYPE_DEVICE,
            .bcdUSB             = 0x0200U,
            .bDeviceClass       = 0x00U,
            .bDeviceSubClass    = 0x00U,
            .bDeviceProtocol    = 0x00U,
            .bMaxPacketSize0    = 64U,
            .idVendor           = VENDOR_ID,
            .idProduct          = PRODUCT_ID_DFU,
            .bcdDevice          = 0x0101U,
            .iManufacturer      = STRING_INDEX_MANUFACTURER,
            .iProduct           = STRING_INDEX_PRODUCT_DFU,
            .iSerialNumber      = STRING_INDEX_SERIAL,
            .bNumConfigurations = 1U,
        },
    .string_count           = STRING_INDEX_COUNT - 1U /* exclude zero */,
    .string_zero_descriptor = &MAIN_STRING_ZERO,
    .language_table         = MAIN_LANGUAGE_TABLE,
    .control_handler        = 0,
    .configurations =
        {
            &UPGRADE_USB_CONFIGURATION,
        },
};

/**
 * \brief Runs the DFU mode, allowing images to be uploaded and downloaded.
 */
void upgrade_dfu_run(void)
{
    // Initialize the two layers.
    upgrade_dfu_writeout_init();
    upgrade_dfu_init();

    // Attach to USB.
    udev_init(&UPGRADE_USB_INFO);
    udev_attach();

    // Wait until done. The task that terminates operation also detaches from
    // USB.
    while (!xSemaphoreTake(upgrade_dfu_state.done_sem, 100U / portTICK_PERIOD_MS))
    {
        // Kick the hardware watchdog.
        IWDG.KR = 0xAAAAU;
        gpio_toggle(PIN_LED_STATUS);
    }

    // Shut down the two layers.
    upgrade_dfu_deinit();
    upgrade_dfu_writeout_deinit();

    // Wait for USB detach to be visible.
    vTaskDelay(100U / portTICK_PERIOD_MS);
}

/**
 * \}
 */
