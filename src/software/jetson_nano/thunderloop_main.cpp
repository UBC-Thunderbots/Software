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
    thunderloop.runLoop();

    return 0;
}
