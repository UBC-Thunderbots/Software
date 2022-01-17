#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <malloc.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>      // Needed for mlockall()
#include <sys/resource.h>  // needed for getrusage
#include <sys/stat.h>
#include <sys/time.h>  // needed for getrusage
#include <time.h>
#include <unistd.h>  // needed for sysconf(int name);

#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"

#define SPI_TX_OCTAL 0x2000
#define SPI_RX_OCTAL 0x4000
#define PRE_ALLOCATION_SIZE (100 * 1024 * 1024) /* 100MB pagefault free buffer */
#define MY_STACK_SIZE (100 * 1024)              /* 100 kB is enough for now. */

typedef struct
{
    uint16_t startVoltage;
    uint16_t initWaitTime;
    uint16_t actualInitWaitTime;
    uint8_t initState;
    uint8_t initMode;
    uint16_t torqueMeasurementFactor;  // uint8_t.uint8_t
    uint8_t motionMode;
    uint8_t motorMan;
    int32_t actualVelocityPT1;
    int64_t akkuActualVelocity;
    int16_t actualTorquePT1;
    int64_t akkuActualTorque;
    int32_t positionScaler;
    int32_t linearScaler;
    int16_t hall_phi_e_old;
    int16_t hall_phi_e_new;
    int16_t hall_actual_coarse_offset;
    uint16_t last_Phi_E_Selection;
    uint32_t last_UQ_UD_EXT;
    int16_t last_PHI_E_EXT;
} TMinimalMotorConfig;

static TMinimalMotorConfig motorConfig[TMC4671_MOTORS];
static int fd  = -1;
static int fd2 = -1;
static void pabort(const char *s)
{
    if (errno != 0)
        perror(s);
    else
        printf("%s\n", s);
    abort();
}

static const char *device  = "/dev/spidev0.0";
static const char *device2 = "/dev/spidev1.0";
static uint32_t mode;
static uint8_t bits = 8;
static char *input_file;
static char *output_file;
static uint32_t speed = 1000000;  // 1Mhz
static uint16_t delay;
static int verbose;
static int transfer_size;
static int iterations;
static char *input_tx;


static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx,
        .rx_buf        = (unsigned long)rx,
        .len           = len,
        .delay_usecs   = delay,
        .speed_hz      = speed,
        .bits_per_word = bits,
    };

    if (mode & SPI_TX_OCTAL)
        tr.tx_nbits = 8;
    else if (mode & SPI_TX_QUAD)
        tr.tx_nbits = 4;
    else if (mode & SPI_TX_DUAL)
        tr.tx_nbits = 2;
    if (mode & SPI_RX_OCTAL)
        tr.rx_nbits = 8;
    else if (mode & SPI_RX_QUAD)
        tr.rx_nbits = 4;
    else if (mode & SPI_RX_DUAL)
        tr.rx_nbits = 2;
    if (!(mode & SPI_LOOP))
    {
        if (mode & (SPI_TX_OCTAL | SPI_TX_QUAD | SPI_TX_DUAL))
            tr.rx_buf = 0;
        else if (mode & (SPI_RX_OCTAL | SPI_RX_QUAD | SPI_RX_DUAL))
            tr.tx_buf = 0;
    }

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        pabort("can't send spi message");
}


uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    static int num_transfers_pending = 0;
    static bool currently_reading    = false;
    static uint8_t tx[20]            = {};
    static uint8_t rx[20]            = {};

    // on the first transfer, lets figure out if its a read or write
    if (num_transfers_pending == 0)
    {
        if (data & 0x80)
        {
            currently_reading = false;
        }
        else
        {
            currently_reading = true;
            memset(tx, 0, 20);
            tx[0] = data;
            transfer(fd, tx, rx, 5);
            num_transfers_pending++;
            return rx[0];
        }
    }

    if (currently_reading)
    {
        if (num_transfers_pending < 4)
        {
            return rx[num_transfers_pending++];
        }
        else if (num_transfers_pending == 4)
        {
            currently_reading     = false;
            num_transfers_pending = 0;
            return rx[num_transfers_pending];
        }
    }

    else
    {
        tx[num_transfers_pending++] = data;
        if (lastTransfer)
        {
            printf("================== TRANSFERRING TO 4672 ===============\n");
            transfer(fd, tx, rx, num_transfers_pending);
            num_transfers_pending = 0;
            return rx[0];
        }
    }

    return rx[0];
}

uint8_t tmc6100_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    static int num_transfers_pending = 0;
    static bool currently_reading    = false;
    static uint8_t tx[20]            = {};
    static uint8_t rx[20]            = {};

    // on the first transfer, lets figure out if its a read or write
    if (num_transfers_pending == 0)
    {
        if (data & 0x80)
        {
            currently_reading = false;
        }
        else
        {
            currently_reading = true;
            memset(tx, 0, 20);
            tx[0] = data;
            transfer(fd2, tx, rx, 5);
            num_transfers_pending++;
            return rx[0];
        }
    }

    if (currently_reading)
    {
        if (num_transfers_pending < 4)
        {
            return rx[num_transfers_pending++];
        }
        else if (num_transfers_pending == 4)
        {
            currently_reading     = false;
            num_transfers_pending = 0;
            return rx[num_transfers_pending];
        }
    }

    else
    {
        tx[num_transfers_pending++] = data;
        if (lastTransfer)
        {
            printf("================== TRANSFERRING TO 6100 ===============\n");
            transfer(fd2, tx, rx, num_transfers_pending);
            num_transfers_pending = 0;
            return rx[0];
        }
    }

    return rx[0];
}


static void print_usage(const char *prog)
{
    printf("Usage: %s [-DsbdlHOLC3vpNR24SI]\n", prog);
    puts(
        "  -D --device   device to use (default /dev/spidev1.1)\n"
        "  -s --speed    max speed (Hz)\n"
        "  -d --delay    delay (usec)\n"
        "  -b --bpw      bits per word\n"
        "  -i --input    input data from a file (e.g. \"test.bin\")\n"
        "  -o --output   output data to a file (e.g. \"results.bin\")\n"
        "  -l --loop     loopback\n"
        "  -H --cpha     clock phase\n"
        "  -O --cpol     clock polarity\n"
        "  -L --lsb      least significant bit first\n"
        "  -C --cs-high  chip select active high\n"
        "  -3 --3wire    SI/SO signals shared\n"
        "  -v --verbose  Verbose (show tx buffer)\n"
        "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
        "  -N --no-cs    no chip select\n"
        "  -R --ready    slave pulls low to pause\n"
        "  -2 --dual     dual transfer\n"
        "  -4 --quad     quad transfer\n"
        "  -8 --octal    octal transfer\n"
        "  -S --size     transfer size\n"
        "  -I --iter     iterations\n");
    exit(1);
}

static void parse_opts(int argc, char *argv[])
{
    while (1)
    {
        static const struct option lopts[] = {
            {"device", 1, 0, 'D'},  {"speed", 1, 0, 's'},   {"delay", 1, 0, 'd'},
            {"bpw", 1, 0, 'b'},     {"input", 1, 0, 'i'},   {"output", 1, 0, 'o'},
            {"loop", 0, 0, 'l'},    {"cpha", 0, 0, 'H'},    {"cpol", 0, 0, 'O'},
            {"lsb", 0, 0, 'L'},     {"cs-high", 0, 0, 'C'}, {"3wire", 0, 0, '3'},
            {"no-cs", 0, 0, 'N'},   {"ready", 0, 0, 'R'},   {"dual", 0, 0, '2'},
            {"verbose", 0, 0, 'v'}, {"quad", 0, 0, '4'},    {"octal", 0, 0, '8'},
            {"size", 1, 0, 'S'},    {"iter", 1, 0, 'I'},    {NULL, 0, 0, 0},
        };
        int c;

        c = getopt_long(argc, argv, "D:s:d:b:i:o:lHOLC3NR248p:vS:I:", lopts, NULL);

        if (c == -1)
            break;

        switch (c)
        {
            case 'D':
                device = optarg;
                break;
            case 's':
                speed = atoi(optarg);
                break;
            case 'd':
                delay = atoi(optarg);
                break;
            case 'b':
                bits = atoi(optarg);
                break;
            case 'i':
                input_file = optarg;
                break;
            case 'o':
                output_file = optarg;
                break;
            case 'l':
                mode |= SPI_LOOP;
                break;
            case 'H':
                mode |= SPI_CPHA;
                break;
            case 'O':
                mode |= SPI_CPOL;
                break;
            case 'L':
                mode |= SPI_LSB_FIRST;
                break;
            case 'C':
                mode |= SPI_CS_HIGH;
                break;
            case '3':
                mode |= SPI_3WIRE;
                break;
            case 'N':
                mode |= SPI_NO_CS;
                break;
            case 'v':
                verbose = 1;
                break;
            case 'R':
                mode |= SPI_READY;
                break;
            case 'p':
                input_tx = optarg;
                break;
            case '2':
                mode |= SPI_TX_DUAL;
                break;
            case '4':
                mode |= SPI_TX_QUAD;
                break;
            case '8':
                mode |= SPI_TX_OCTAL;
                break;
            case 'S':
                transfer_size = atoi(optarg);
                break;
            case 'I':
                iterations = atoi(optarg);
                break;
            default:
                print_usage(argv[0]);
        }
    }
    if (mode & SPI_LOOP)
    {
        if (mode & SPI_TX_DUAL)
            mode |= SPI_RX_DUAL;
        if (mode & SPI_TX_QUAD)
            mode |= SPI_RX_QUAD;
        if (mode & SPI_TX_OCTAL)
            mode |= SPI_RX_OCTAL;
    }
}

static void periodicJob(uint32_t actualSystick)
{
    int32_t motor;

    // do encoder initialization if necessary
    for (motor = 0; motor < TMC4671_MOTORS; motor++)
    {
        tmc4671_periodicJob(
            motor, actualSystick, motorConfig[motor].initMode,
            &(motorConfig[motor].initState), motorConfig[motor].initWaitTime,
            &(motorConfig[motor].actualInitWaitTime), motorConfig[motor].startVoltage,
            &(motorConfig[motor].hall_phi_e_old), &(motorConfig[motor].hall_phi_e_new),
            &(motorConfig[motor].hall_actual_coarse_offset),
            &(motorConfig[motor].last_Phi_E_Selection),
            &(motorConfig[motor].last_UQ_UD_EXT), &(motorConfig[motor].last_PHI_E_EXT));
    }
}

static void setprio(int prio, int sched)
{
    struct sched_param param;
    // Set realtime priority for this thread
    param.sched_priority = prio;
    if (sched_setscheduler(0, sched, &param) < 0)
        perror("sched_setscheduler");
}

void show_new_pagefault_count(const char *logtext, const char *allowed_maj,
                              const char *allowed_min)
{
    static int last_majflt = 0, last_minflt = 0;
    struct rusage usage;

    getrusage(RUSAGE_SELF, &usage);

    printf(
        "%-30.30s: Pagefaults, Major:%ld (Allowed %s), "
        "Minor:%ld (Allowed %s)\n",
        logtext, usage.ru_majflt - last_majflt, allowed_maj,
        usage.ru_minflt - last_minflt, allowed_min);

    last_majflt = usage.ru_majflt;
    last_minflt = usage.ru_minflt;
}

static void prove_thread_stack_use_is_safe(int stacksize)
{
    volatile char buffer[stacksize];
    int i;

    /* Prove that this thread is behaving well */
    for (i = 0; i < stacksize; i += sysconf(_SC_PAGESIZE))
    {
        /* Each write to this buffer shall NOT generate a
           pagefault. */
        buffer[i] = i;
    }

    show_new_pagefault_count("Caused by using thread stack", "0", "0");
}

/*************************************************************/
/* The thread to start */
static void *my_rt_thread(void *args)
{
    struct timespec ts;
    ts.tv_sec  = 30;
    ts.tv_nsec = 0;

    setprio(sched_get_priority_max(SCHED_RR), SCHED_RR);
    printf(
        "I am an RT-thread with a stack that does not generate "
        "page-faults during use, stacksize=%i\n",
        MY_STACK_SIZE);

    for (;;)
    {
        /* get monotonic clock time */
        struct timespec monotime;
        clock_gettime(CLOCK_MONOTONIC, &monotime);
        printf("%ld\n", monotime.tv_nsec / 1000);
    }

    show_new_pagefault_count("Caused by creating thread", ">=0", ">=0");
    prove_thread_stack_use_is_safe(MY_STACK_SIZE);
    clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);

    return NULL;
}

/*************************************************************/

static void error(int at)
{
    /* Just exit on error */
    fprintf(stderr, "Some error occurred at %d", at);
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
    int i;
    char *buffer;

    printf("malloc start\n");
    buffer = malloc(size);
    printf("malloc end: %p\n", buffer);

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
    printf("free start\n");
    free(buffer);
    printf("free end\n");
}

int main(int argc, char *argv[])
{
    printf("THUNDERWARE LOL\n");

    int ret = 0;

    parse_opts(argc, argv);

    if (input_tx && input_file)
        pabort("only one of -p and --input may be selected");

    fd = open(device, O_RDWR);
    if (fd < 0)
        pabort("can't open device");

    ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
        pabort("can't set spi mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
    if (ret == -1)
        pabort("can't get spi mode");

    /*
     * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't set bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't get bits per word");

    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't set max speed hz");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't get max speed hz");

    fd2 = open(device2, O_RDWR);
    if (fd2 < 0)
        pabort("can't open device2");

    ret = ioctl(fd2, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
        pabort("can't set spi mode");

    ret = ioctl(fd2, SPI_IOC_RD_MODE32, &mode);
    if (ret == -1)
        pabort("can't get spi mode");

    /*
     * bits per word
     */
    ret = ioctl(fd2, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't set bits per word");

    ret = ioctl(fd2, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't get bits per word");

    /*
     * max speed hz
     */
    ret = ioctl(fd2, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't set max speed hz");

    ret = ioctl(fd2, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't get max speed hz");

    printf("spi mode: 0x%x\n", mode);
    printf("bits per word: %u\n", bits);
    printf("max speed: %u Hz (%u kHz)\n", speed, speed / 1000);

    printf"WRITING TO THIS SHIT\n");
    tmc6100_writeInt(0, TMC6100_GCONF, 32);
    printf("DONE WRITING TO THIS SHIT\n");

    show_new_pagefault_count("Initial count", ">=0", ">=0");
    configure_malloc_behavior();
    show_new_pagefault_count("mlockall() generated", ">=0", ">=0");
    reserve_process_memory(PRE_ALLOCATION_SIZE);

    /* Now allocate the memory for the 2nd time and prove the number of
       pagefaults are zero */
    show_new_pagefault_count("malloc() and touch generated", ">=0", ">=0");
    reserve_process_memory(PRE_ALLOCATION_SIZE);
    show_new_pagefault_count("2nd malloc() and use generated", "0", "0");

    // Motor type &  PWM configuration
    tmc4671_writeInt(0, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
    tmc4671_writeInt(0, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(0, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(0, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(0, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(0, TMC4671_ADC_I_SELECT, 0x09000100);
    tmc4671_writeInt(0, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(0, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(0, TMC4671_dsADC_MCLK_B, 0x00000000);
    tmc4671_writeInt(0, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(0, TMC4671_ADC_I0_SCALE_OFFSET, 0x0100820A);
    tmc4671_writeInt(0, TMC4671_ADC_I1_SCALE_OFFSET, 0x0100819E);

    // ABN encoder settings
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_MODE, 0x00000000);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_PPR, 0x00001000);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x00000ED5);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);

    // Limits
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);

    // PI settings
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
    tmc4671_writeInt(0, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

    // ===== ABN encoder test drive =====
    //
    // Init Encoder (mode 0)
    tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000001);
    tmc4671_writeInt(0, TMC4671_PHI_E_EXT, 0x00000000);
    tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 0x000007D0);
    sleep(1);
    tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x00000000);

    // Feedback selection
    tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000003);
    tmc4671_writeInt(0, TMC4671_VELOCITY_SELECTION, 0x00000009);

    // Switch to torque mode
    tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

    // Rotate right
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
    sleep(1);

    // Rotate left
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
    int output = tmc4671_readInt(0, TMC4671_PID_TORQUE_FLUX_TARGET);
    printf("output of read %d\n", output);
    sleep(1);

    // Stop
    tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);

    close(fd);
    close(fd2);

    return ret;
}
