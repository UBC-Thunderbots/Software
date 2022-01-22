#include "software/jetson_nano/thunderloop.h"

#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>      // Needed for mlockall()
#include <sys/resource.h>  // needed for getrusage
#include <sys/time.h>      // needed for getrusage
#include <unistd.h>        // needed for sysconf(int name);

#include <chrono>
#include <iostream>
#include <thread>

#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"


static Thunderloop* g_tloop;


#define NSEC_PER_SEC    1000000000
#define PRE_ALLOCATION_SIZE (100 * 1024 * 1024) /* 100MB pagefault free buffer */
#define MY_STACK_SIZE (100 * 1024)              /* 100 kB is enough for now. */

static void setprio(int prio, int sched)
{
    struct sched_param param;
    // Set realtime priority for this thread
    param.sched_priority = prio;
    if (sched_setscheduler(0, sched, &param) < 0)
        perror("sched_setscheduler");
}

void show_new_pagefault_count(const char* logtext, const char* allowed_maj,
                              const char* allowed_min)
{
    static long int last_majflt = 0, last_minflt = 0;
    struct rusage usage;

    getrusage(RUSAGE_SELF, &usage);

    LOGF(DEBUG,
         "%-30.30s: Pagefaults, Major:%ld (Allowed %s), "
         "Minor:%ld (Allowed %s)\n",
         logtext, usage.ru_majflt - last_majflt, allowed_maj,
         usage.ru_minflt - last_minflt, allowed_min);

    last_majflt = usage.ru_majflt;
    last_minflt = usage.ru_minflt;
}

static void prove_thread_stack_use_is_safe(int stacksize)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvla"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    volatile char buffer[stacksize];
    long int i;

    /* Prove that this thread is behaving well */
    for (i = 0; i < stacksize; i += sysconf(_SC_PAGESIZE))
    {
        /* Each write to this buffer shall NOT generate a
            pagefault. */
        buffer[i] = static_cast<char>(i);
    }

#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
    show_new_pagefault_count("Caused by using thread stack", "0", "0");
}

/* using clock_nanosleep of librt */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec* __req, struct timespec* __rem);

/* the struct timespec consists of nanoseconds
 * and seconds. if the nanoseconds are getting
 * bigger than 1000000000 (= 1 second) the
 * variable containing seconds has to be
 * incremented and the nanoseconds decremented
 * by 1000000000.
 */
static inline void tsnorm(struct timespec* ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC)
    {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

static void* my_rt_thread(void* args)
{
    struct timespec t;
    int interval_ns = 1000000; // 1ms

    setprio(sched_get_priority_max(SCHED_RR), SCHED_RR);

    LOGF(DEBUG,
         "I am an RT-thread with a stack that does not generate "
         "page-faults during use, stacksize=%i\n",
         MY_STACK_SIZE);

    /* get current time */
    clock_gettime(0, &t);

    /* start after one second */
    t.tv_sec++;

    uint32_t millisecond_counter = 0;

    while (1)
    {
        /* wait untill next shot */
        clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);

        millisecond_counter++;

        /* do the stuff */
        g_tloop->motor_service_->periodicJob(millisecond_counter);

        /* calculate next shot */
        t.tv_nsec += interval_ns;
        tsnorm(&t);
    }


    show_new_pagefault_count("Caused by creating thread", ">=0", ">=0");
    prove_thread_stack_use_is_safe(MY_STACK_SIZE);
    return NULL;
}

/*************************************************************/

static void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occured at %d", at);
    exit(1);
}

static void start_rt_thread(void)
{
    pthread_t thread;
    pthread_attr_t attr;

    /* init to default values */
    if (pthread_attr_init(&attr))
        error(1);

    /* Set the requested stacksize for this thread */
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + MY_STACK_SIZE))
        error(2);

    /* And finally start the actual thread */
    pthread_create(&thread, &attr, my_rt_thread, NULL);
}

static void configure_malloc_behavior(void)
{
    /* Now lock all current and future pages
       from preventing of being paged */
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        perror("mlockall failed:");

    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);

    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);
}

static void reserve_process_memory(int size)
{
    long int i;
    char* buffer;

    buffer = static_cast<char*>(malloc(size));

    /* Touch each page in this piece of memory to get it mapped into RAM */
    for (i = 0; i < size; i += sysconf(_SC_PAGESIZE))
    {
        /* Each write to this buffer will generate a pagefault.
           Once the pagefault is handled a page will be locked in
           memory and never given back to the system. */
        buffer[i] = 0;
    }

    /* buffer will now be released. As Glibc is configured such that it
       never gives back memory to the kernel, the memory allocated above is
       locked for this process. All malloc() and new() calls come from
       the memory pool reserved and locked above. Issuing free() and
       delete() does NOT make this locking undone. So, with this locking
       mechanism we can build C++ applications that will never run into
       a major/minor pagefault, even with swapping enabled. */
    free(buffer);
}

Thunderloop::Thunderloop(const RobotConstants_t& robot_constants,
                         const WheelConstants_t& wheel_consants)
{
    robot_id_        = 0;
    robot_constants_ = robot_constants;
    wheel_consants_  = wheel_consants;

    motor_service_ = std::make_unique<MotorService>(robot_constants, wheel_consants);

    // TODO (#2331) remove this once we receive actual vision data
    current_robot_state_ =
        std::make_unique<RobotState>(Point(), Vector(), Angle(), AngularVelocity());

    g_tloop = this;
}

Thunderloop::~Thunderloop()
{
    // De-initialize Services
    motor_service_->stop();
}

/*
 * Run the main robot loop!
 *
 * @param The rate to run the loop
 */
void Thunderloop::run(unsigned run_at_hz)
{
    using clock     = std::chrono::steady_clock;
    auto next_frame = clock::now();

    show_new_pagefault_count("Initial count", ">=0", ">=0");

    configure_malloc_behavior();

    show_new_pagefault_count("mlockall() generated", ">=0", ">=0");

    reserve_process_memory(PRE_ALLOCATION_SIZE);

    show_new_pagefault_count("malloc() and touch generated", ">=0", ">=0");

    /* Now allocate the memory for the 2nd time and prove the number of
       pagefaults are zero */
    reserve_process_memory(PRE_ALLOCATION_SIZE);
    show_new_pagefault_count("2nd malloc() and use generated", "0", "0");

    LOGF(DEBUG,
         "\n\nLook at the output of ps -leyf, and see that the "
         "RSS is now about %d [MB]\n",
         PRE_ALLOCATION_SIZE / (1024 * 1024));

    start_rt_thread();
    motor_service_->start();

    for (;;)
    {
        // TODO (#2335) add loop timing introspection and use Preempt-RT (maybe)
        next_frame += std::chrono::milliseconds(
            static_cast<int>(MILLISECONDS_PER_SECOND / run_at_hz));

        // TODO (#2331) poll network service and update current_robot_state_
        // TODO (#2333) poll redis service

        // Execute latest primitive
        primitive_executor_.startPrimitive(robot_constants_, primitive_);
        direct_control_ = *primitive_executor_.stepPrimitive(*current_robot_state_);

        // Poll motor service with wheel velocities and dribbler rpm
        // TODO (#2332) properly implement, this is just a placeholder
        drive_units_status_ =
            *motor_service_->poll(direct_control_.direct_velocity_control(),
                                  direct_control_.dribbler_speed_rpm());

        // TODO (#2334) power service poll
        std::this_thread::sleep_until(next_frame);
    }
}
