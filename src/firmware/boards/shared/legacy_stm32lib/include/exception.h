/**
 * \addtogroup EXC
 * @{
 */
#ifndef STM32LIB_EXCEPTION_H
#define STM32LIB_EXCEPTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * \brief The functionality that must be provided by a core dump writer.
 */
typedef struct
{
    /**
     * \brief Starts writing a core dump.
     */
    void (*start)(void);

    /**
     * \brief Writes a block of data to the core dump.
     *
     * \param[in] data the data to write
     *
     * \param[in] length the number of bytes to write, which is guaranteed to be a
     * multiple of four
     */
    void (*write)(const void *, size_t);

    /**
     * \brief Finishes writing a core dump.
     *
     * \return \c true if the dump was written successfully, or \c false if an error
     * occurred saving the core dump
     */
    bool (*end)(void);
} exception_core_writer_t;

/**
 * \brief The functionality that an application can provide to be invoked during a fatal
 * error.
 */
typedef struct
{
    /**
     * \brief Invoked almost immediately when an exception is taken.
     *
     * This callback should safe any dangerous hardware, perhaps display a preliminary
     * indication, and return.
     *
     * This may be null to not execute an early callback.
     */
    void (*early)(void);

    /**
     * \brief Invoked after exception handling is complete.
     *
     * This callback should display an indication to the user indefinitely.
     * It should generally not return.
     *
     * This may be null to just lock up forever with no further activity.
     *
     * \param[in] core_written \c true if a core dump was written, or \c false if not
     */
    void (*late)(bool);
} exception_app_cbs_t;

void exception_init(const exception_core_writer_t *core_writer,
                    const exception_app_cbs_t *app_cbs, const uint8_t *prios);
void exception_hard_fault_isr(void) __attribute__((naked, noreturn));
void exception_memory_manage_fault_isr(void) __attribute__((naked, noreturn));
void exception_bus_fault_isr(void) __attribute__((naked, noreturn));
void exception_usage_fault_isr(void) __attribute__((naked, noreturn));
void exception_debug_fault_isr(void) __attribute__((naked, noreturn));

/**
 * \brief The number of bits used for group priority.
 */
#define EXCEPTION_GROUP_PRIO_BITS 3U

/**
 * \brief The number of bits used for subprioroity.
 */
#define EXCEPTION_SUB_PRIO_BITS 1U

/**
 * \brief Generates an exception priority byte from a group priority and a subpriority.
 *
 * \param[in] group the group priority
 * \param[in] sub the subpriority
 * \return the priority byte
 */
#define EXCEPTION_MKPRIO(group, sub)                                                     \
    ((((group) << EXCEPTION_SUB_PRIO_BITS) | (sub))                                      \
     << (8U - EXCEPTION_GROUP_PRIO_BITS - EXCEPTION_SUB_PRIO_BITS))

/**
 * \brief Extracts the group priority from an exception priority.
 *
 * \param[in] prio the priority byte
 * \return the group priority
 */
#define EXCEPTION_GROUP_PRIO(prio) ((prio) >> (8U - EXCEPTION_GROUP_PRIO_BITS))

/**
 * \brief Extracts the subpriority from an exception priority.
 *
 * \param[in] prio the priority byte
 * \return the subpriority
 */
#define EXCEPTION_SUB_PRIO(prio)                                                         \
    (((prio) >> (8U - EXCEPTION_GROUP_PRIO_BITS - EXCEPTION_SUB_PRIO_BITS)) &            \
     ((1U << EXCEPTION_SUB_PRIO_BITS) - 1U))

/**
 * \brief Inserts the barrier needed before an exception return to handle
 * Cortex-M4 erratum 838869.
 */
#define EXCEPTION_RETURN_BARRIER()                                                       \
    do                                                                                   \
    {                                                                                    \
        __atomic_signal_fence(__ATOMIC_SEQ_CST);                                         \
        asm volatile("dsb");                                                             \
    } while (0)

#endif

/**
 * @}
 */
