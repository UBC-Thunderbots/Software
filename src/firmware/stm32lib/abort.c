#include <stdlib.h>

// This overrides the abort function from newlib.
void abort(void)
{
    asm volatile("bkpt");
    __builtin_unreachable();
}

void vApplicationMallocFailedHook(void)
{
    abort();
}
