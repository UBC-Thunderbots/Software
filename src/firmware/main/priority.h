#ifndef PRIORITY_H
#define PRIORITY_H

/**
 * \defgroup PRIO Task and Exception Priorities
 *
 * @{
 */

/**
 * \name Task Priorities
 *
 * These are the priorities of FreeRTOS tasks, which should range from 0 to one less than
 * \c configMAX_PRIORITIES.
 *
 * FreeRTOS defines priorities “forwards”; that is, priority 0 is the least important and
 * will yield to all others.
 *
 * @{
 */

/**
 * \brief The maximum possible task priority.
 */
#define PRIO_TASK_MAX 7U

/**
 * \brief The priority of the supervisor task, which is also the first task at startup
 * that initializes the system.
 *
 * The supervisor needs to run from time to time, but is not very time-critical.
 * If it is starved of CPU time due to a firmware bug, eventually the hardware watchdog
 * will kill the system safely. Therefore, a low priority is suitable.
 */
#define PRIO_TASK_SUPERVISOR 2U

/**
 * \brief The priority of the USB stack internal task.
 *
 * The USB stack is only used for CDC ACM debug printing, so should be very low priority.
 */
#define PRIO_TASK_USB 1U

/**
 * \brief The priority of the CDC ACM buffer flushing task.
 *
 * The CDC ACM channel is only used for debug printing, so should be very low priority.
 */
#define PRIO_TASK_CDC_ACM 1U

/**
 * \brief The priority of the ICB IRQ polling and dispatching task.
 *
 * There are many different possible ICB IRQ sources of different priorities.
 * Therefore, a high priority is suitable to ensure urgent IRQs are not unduly delayed.
 */
#define PRIO_TASK_ICB_IRQ 6U

/**
 * \brief The priority of the radio receive path task.
 *
 * This is fairly important, so has a fairly high priority.
 */
#define PRIO_TASK_RX 3U

/**
 * \brief The priority of the radio feedback task.
 *
 * This is not very critical.
 */
#define PRIO_TASK_FEEDBACK 2U

/**
 * \brief The priority of the normal tick task.
 *
 * This needs to be quite high priority to keep jitter low.
 */
#define PRIO_TASK_NORMAL_TICK 5U

/**
 * \brief The priority of the data logging writeout task.
 *
 * This must be very low priority as it contains a busy wait!
 */
#define PRIO_TASK_LOG_WRITEOUT 1U

/**
 * \brief The priority of the firmware upgrade data writeout task.
 *
 * This must be very low priority as it contains a busy wait!
 */
#define PRIO_TASK_UPGRADE_WRITEOUT 1U

/**
 * @}
 */

/**
 * @}
 */

#endif
