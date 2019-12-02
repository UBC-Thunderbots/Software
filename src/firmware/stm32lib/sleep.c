#include <registers/systick.h>
#include <sleep.h>
#include <stdint.h>

void sleep_systick_overflows(unsigned long ticks)
{
    while (ticks--)
    {
        while (!SYSTICK.CSR.COUNTFLAG)
            ;
    }
}
