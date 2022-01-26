#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/thunderloop.h"
#include "software/logger/logger.h"
#include "software/world/robot_state.h"

// clang-format off
std::string BANNER =
"        ,/                                                                                                                     ,/   \n"
"      ,'/         _    _ ____   _____   _______ _    _ _    _ _   _ _____  ______ _____  ____   ____ _______ _____           ,'/    \n"
"    ,' /         | |  | |  _ \\ / ____| |__   __| |  | | |  | | \\ | |  __ \\|  ____|  __ \\|  _ \\ / __ \\__   __/ ____|        ,' /     \n"
"  ,'  /_____,    | |  | | |_) | |         | |  | |__| | |  | |  \\| | |  | | |__  | |__) | |_) | |  | | | | | (___        ,'  /_____,\n"
".'____    ,'     | |  | |  _ <+ |         | |  |  __  | |  | | . ` | |  | |  __| |  _  /|  _ <+ |  | | | |  \\___ \\     .'____    ,' \n"
"     /  ,'       | |__| | |_) | |____     | |  | |  | | |__| | |\\  | |__| | |____| | \\ \\| |_) | |__| | | |  ____) |         /  ,'   \n"
"    / ,'          \\____/|____/ \\_____|    |_|  |_|  |_|\\____/|_| \\_|_____/|______|_|  \\_\\____/ \\____/  |_| |_____/         / ,'     \n"
"   /,'                                                                                                                    /,'       \n"
"  /'                                                                                                                     /'          \n";
// clang-format on

static const g_nsec_per_sec        = 1000000000;
static const g_pre_allocation_size = (100 * 1024 * 1024);  // 100MB pagefault free buffer
static const g_periodic_job_stack_size = (100 * 1024);     // 100kb

static void setprio(int prio, int sched)
{
    struct sched_param param;
    // Set realtime priority for this thread
    param.sched_priority = prio;
    if (sched_setscheduler(0, sched, &param) < 0)
    {
        LOG(FATAL) << "sched_setscheduler: " << strerror(errno);
    }
}

// https://rt.wiki.kernel.org/index.php/Squarewave-example
// https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example
// using clock_nanosleep of librt
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec* __req, struct timespec* __rem);

/* The struct timespec consists of nanoseconds and seconds. If the nanoseconds
 * are getting bigger than 1000000000 (= 1 second) the variable containing
 * seconds has to be incremented and the nanoseconds decremented by 1000000000.
 *
 * @param ts timespec to modify
 */
static inline void tsnorm(struct timespec& ts)
{
    while (ts.tv_nsec >= g_nsec_per_sec)
    {
        ts.tv_nsec -= g_nsec_per_sec;
        ts.tv_sec++;
    }
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


static void* motorServicePeriodicJob(void* args)
{
    struct timespec t;
    int interval_ns              = 1000000;  // 1ms
    uint32_t millisecond_counter = 0;

    setprio(sched_get_priority_max(SCHED_RR), SCHED_RR);

    /* get current time */
    clock_gettime(0, &t);

    /* start after one second */
    t.tv_sec++;

    LOG(INFO) << "Motor Service Periodic Job Starting";

    for (;;)
    {
        // wait until next shot
        clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);

        millisecond_counter++;

        g_tloop->motor_service_->periodicJob(millisecond_counter);

        // calculate next shot
        t.tv_nsec += interval_ns;
        tsnorm(t);
    }

    return NULL;
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

    // Buffer will now be released. As Glibc is configured such that it
    // never gives back memory to the kernel, the memory allocated above is
    // locked for this process. All malloc() and new() calls come from
    // the memory pool reserved and locked above. Issuing free() and
    // delete() does NOT make this locking undone. So, with this locking
    // mechanism we can build C++ applications that will never run into
    // a major/minor pagefault, even with swapping enabled.
    free(buffer);
}

int main(int argc, char** argv)
{
    std::cout << BANNER << std::endl;

    // Page faults are bad, lets setup malloc and reserve some memory
    configure_malloc_behavior();
    reserve_process_memory(PRE_ALLOCATION_SIZE);

    // TODO (#2338) replace with network logger
    LoggerSingleton::initializeLogger("/tmp");

    auto thunderloop =
        Thunderloop(create2021RobotConstants(), create2021WheelConstants());

    pthread_t thread;
    pthread_attr_t attr;

    /* init to default values */
    if (pthread_attr_init(&attr))
        error(1);

    /* Set the requested stacksize for this thread */
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + g_periodic_job_stack_size))
        error(2);

    /* And finally start the actual thread */
    pthread_create(&thread, &attr, my_rt_thread, NULL);

    return 0;
}
