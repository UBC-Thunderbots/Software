#pragma once

#include <time.h>

struct ScopedTimespecTimer
{
    /**
     * Constructs a scoped timer that sets time_elapsed to the time that passed between
     * construction and destruction
     *
     * @param time_elapsed timespec pointer to put the time elapsed
     */
    ScopedTimespecTimer(struct timespec* time_elapsed);

    /**
     * Subtracts the time value in b from the time value in a, and places the result
     * in the timeval pointed to by res. The result is normalized such that res->tv_usec
     * has  a value in the range 0 to 999,999. Reimplementation of timersub
     * (http://manpages.ubuntu.com/manpages/xenial/man3/timeradd.3.html) for portability
     *
     * @param a the minuend
     * @param b the subtrahend
     * @param result a - b
     */
    static void timespecDiff(struct timespec* a, struct timespec* b,
                             struct timespec* result);

    ~ScopedTimespecTimer(void);

   private:
    struct timespec start_time;
    struct timespec* time_elapsed;
};
