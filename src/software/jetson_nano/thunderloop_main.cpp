#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>      // Needed for mlockall()
#include <sys/resource.h>  // needed for getrusage
#include <sys/time.h>      // needed for getrusage
#include <unistd.h>        // needed for sysconf(int name);

#include "proto/tbots_software_msgs.pb.h"
#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/thunderloop.h"
#include "software/logger/network_logger.h"
#include "software/world/robot_state.h"

// clang-format off
std::string BANNER =
"        ,/                                                                                       ,/    \n"
"      ,'/         _____ _   _ _   _ _   _ ____  _____ ____  _     ___   ___  ____              ,'/     \n"
"    ,' /         |_   _| | | | | | | \\ | |  _ \\| ____|  _ \\| |   / _ \\ / _ \\|  _ \\           ,' /      \n"
"  ,'  /_____,      | | | |_| | | | |  \\| | | | |  _| | |_ )| |  | | | | | | | |_) |        ,'  /_____, \n"
".'____    ,'       | | |  _  | |_| | |\\  | |_| | |___|  _ <| |__| |_| | |_| |  __/       .'____    ,'  \n"
"     /  ,'         |_| |_| |_|\\___/|_| \\_|____/|_____|_| \\_\\_____\\___/ \\___/|_|               /  ,'    \n"
"    / ,'                                                                                     / ,'      \n"
"   /,'                                                                                      /,'        \n"
"  /'                                                                                       /'          \n";
// clang-format on

static const unsigned MOTOR_SPI_COMMUNICATION_STACK_SIZE = 100 * 1024;

// static void setPriority(int prio, int sched)
//{
// struct sched_param param;
//// Set realtime priority for this thread
// param.sched_priority = prio;

// if (sched_setscheduler(0, sched, &param) < 0)
//{
// perror("sched_setscheduler");
//}
//}

// void showNewPagefaultCount(const char* logtext, const char* allowed_maj,
// const char* allowed_min)
//{
// static long int last_majflt = 0, last_minflt = 0;
// struct rusage usage;

// getrusage(RUSAGE_SELF, &usage);

// printf(
//"%-30.30s: Pagefaults, Major:%ld (Allowed %s), "
//"Minor:%ld (Allowed %s)\n",
// logtext, usage.ru_majflt - last_majflt, allowed_maj,
// usage.ru_minflt - last_minflt, allowed_min);

// last_majflt = usage.ru_majflt;
// last_minflt = usage.ru_minflt;
//}

// static void proveThreadStackUseIsSafe(int stacksize)
//{
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wvla"
//#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
// volatile char buffer[stacksize];
//#pragma GCC diagnostic pop
// long int i;

//[> Prove that this thread is behaving well <]
// for (i = 0; i < stacksize; i += sysconf(_SC_PAGESIZE))
//{
//// Each write to this buffer shall NOT generate a pagefault.
// buffer[i] = static_cast<char>(i);
//}

// showNewPagefaultCount("Caused by using thread stack", "0", "0");
//}

static void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occurred at %d", at);
    exit(1);
}

static void startRtThread(void* (*thread_main)(void*), void* arg, unsigned stack_size)
{
    pthread_t thread;
    pthread_attr_t attr;

    // init to default values
    if (pthread_attr_init(&attr))
    {
        error(1);
    }

    // Set the requested stacksize for this thread
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + stack_size))
    {
        error(2);
    }

    // And finally start the actual thread
    pthread_create(&thread, &attr, thread_main, arg);
}

/*
 * Configure malloc for real-time linux
 *
 * https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example
 */
static void configureMallocBehaviour(void)
{
    // Now lock all current and future pages
    // from preventing of being paged
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
    {
        perror("mlockall failed:");
    }

    // Turn off malloc trimming.
    mallopt(M_TRIM_THRESHOLD, -1);

    // Turn off mmap usage.
    mallopt(M_MMAP_MAX, 0);
}


/*
 * Reserve memory for this process
 *
 * @param size How many bytes to reserve
 */
static void reserveProcessMemory(int size)
{
    long int i;
    char* buffer;

    buffer = static_cast<char*>(malloc(size));

    // Touch each page in this piece of memory to get it mapped into RAM
    for (i = 0; i < size; i += sysconf(_SC_PAGESIZE))
    {
        // Each write to this buffer will generate a pagefault.
        // Once the pagefault is handled a page will be locked in
        // memory and never given back to the system.
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
    configureMallocBehaviour();

    // 100MB pagefault free buffer
    const int pre_allocation_size = 20 * 1024 * 1024;
    reserveProcessMemory(pre_allocation_size);

    static auto thunderloop = Thunderloop(create2021RobotConstants(), CONTROL_LOOP_HZ);
    startRtThread(thunderloop.runThunderloopRealtime, &thunderloop,
                  MOTOR_SPI_COMMUNICATION_STACK_SIZE);

    return 0;
}
