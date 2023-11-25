#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"

ScopedTimespecTimer::ScopedTimespecTimer(struct timespec* time_elapsed)
    : time_elapsed(time_elapsed)
{
    // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since CLOCK_REALTIME can jump
    // backwards
    clock_gettime(CLOCK_MONOTONIC, &start_time);
}

void ScopedTimespecTimer::timespecDiff(struct timespec* a, struct timespec* b,
                                       struct timespec* result)
{
    result->tv_sec  = a->tv_sec - b->tv_sec;
    result->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (result->tv_nsec < 0)
    {
        --result->tv_sec;
        result->tv_nsec += 1000000000L;
    }
}

ScopedTimespecTimer::~ScopedTimespecTimer(void)
{
    struct timespec end_time;
    // Note: CLOCK_MONOTONIC is used over CLOCK_REALTIME since CLOCK_REALTIME can jump
    // backwards
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    timespecDiff(&end_time, &start_time, time_elapsed);
}
