/**
 * \defgroup STACK Stack Allocation
 *
 * \{
 */
#if !defined(STM32LIB_STACK_H)
#define STM32LIB_STACK_H

/**
 * \brief Allocates a stack.
 *
 * \param[in] name The name of the variable that will be the stack space.
 * \param[in] size The size of the stack, in bytes.
 */
#define STACK_ALLOCATE(name, size)                                                       \
    _Static_assert(!((size) % 32), "Stack size must be a multiple of 32.");              \
    static unsigned long name[(size) / sizeof(unsigned long)]                            \
        __attribute__((aligned(32), section("stacks")))

#endif
/**
 * \}
 */
