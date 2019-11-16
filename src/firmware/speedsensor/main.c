#include <FreeRTOS.h>
#include <cdcacm.h>
#include <gpio.h>
#include <init.h>
#include <nvic.h>
#include <rcc.h>
#include <registers/exti.h>
#include <registers/flash.h>
#include <registers/otg_fs.h>
#include <registers/power.h>
#include <registers/scb.h>
#include <registers/syscfg.h>
#include <registers/systick.h>
#include <registers/timer.h>
#include <sleep.h>
#include <stack.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unused.h>
#include <usb.h>

#include "constants.h"
#include "format.h"
#include "pins.h"
#include "usb_config.h"

volatile uint8_t G_status = 0;  // 1 for first triggered, 2 for second triggered, 0 for
                                // last wrap_count is output to screen

static void stm32_main(void) __attribute__((noreturn));
static void external_interrupt_15_10_vector(void);
static void timer2_wrap_interrupt(void);

// static void display( float );
static void LCD_write(char a);

static void tic(void);
static void toc(void);

typedef void (*fptr)(void);

STACK_ALLOCATE(mstack, 4096);

static const fptr exception_vectors[16]
    __attribute__((used, section(".exception_vectors"))) = {
        // Vector 0 contains the reset stack pointer
        [0] = (fptr)(mstack + sizeof(mstack) / sizeof(*mstack)),
        // Vector 1 contains the reset vector
        [1] = &stm32_main,
        // Vector 2 contains the NMI vector
        //[2] = &nmi_vector,
        // Vector 3 contains the HardFault vector
        [3] = &exception_hard_fault_isr,
        // Vector 4 contains the MemManage vector
        [4] = &exception_memory_manage_fault_isr,
        // Vector 5 contains the BusFault vector
        [5] = &exception_bus_fault_isr,
        // Vector 6 contains the UsageFault vector
        [6] = &exception_usage_fault_isr,
        // Vector 11 contains the SVCall vector
        [11] = &vPortSVCHandler,
        // Vector 12 contains debug fault vector?
        [12] = &exception_debug_fault_isr,
        // Vector 14 contains the PendSV vector
        [14] = &vPortPendSVHandler,
        // Vector 15 contains the SysTick vector
        [15] = &vPortSysTickHandler,
};

static const fptr interrupt_vectors[82U]
    __attribute__((used, section(".interrupt_vectors"))) = {
        [NVIC_IRQ_TIM2]      = &timer2_wrap_interrupt,
        [NVIC_IRQ_EXTI15_10] = &external_interrupt_15_10_vector,
        [NVIC_IRQ_OTG_FS]    = &udev_isr,
};

/* This runs before core is dumped. No core dump for now. */
/* TODO Add core dump functionality */
/*
static void app_exception_early(void) {
  // Power down the USB engine to disconnect from the host.
  OTG_FS.GCCFG.PWRDWN = 0;

  // Turn on the LEDs.
  gpio_set(PIN_LED_LEFT);
  gpio_set(PIN_LED_RIGHT);
}

static void app_exception_late(bool core_written) {
}
*/

// This function is where the state machine goes
static void external_interrupt_15_10_vector(void)
{
    /* EXTI.PR are pending bits, bit is set to 1 when interrupt occurs.
       12th bit is set to 1 to clear the trigger event.
       This external interrupt is connected to the first breakbeam sensor.
    */
    if (EXTI.PR & (1 << 12))
    {
        EXTI.PR = 1 << 12;
        gpio_toggle(PIN_LED_LEFT);
        if (G_status == 0)
        {
            G_status = 1;
            tic();
        }
    }

    if (EXTI.PR & (1 << 13))
    {
        EXTI.PR = 1 << 13;
        gpio_toggle(PIN_LED_RIGHT);
        if (G_status == 1)
        {
            G_status = 2;
            toc();
        }
    }

    EXCEPTION_RETURN_BARRIER();
}

static const init_specs_t INIT_SPECS = {
    .flags =
        {
            .hse_crystal          = true,
            .freertos             = true,
            .io_compensation_cell = false,
            .uses_usb             = true,
        },
    .hse_frequency         = 8,
    .pll_frequency         = 288,
    .sys_frequency         = 144,
    .cpu_frequency         = 144,
    .apb1_frequency        = 36,
    .apb2_frequency        = 72,
    .exception_core_writer = 0,
    .exception_app_cbs =
        {
            .early = 0,
            .late  = 0,
        },
    .exception_prios =
        {
            [NVIC_IRQ_TIM2]      = EXCEPTION_MKPRIO(6, 0),
            [NVIC_IRQ_EXTI15_10] = EXCEPTION_MKPRIO(6, 0),
            [NVIC_IRQ_OTG_FS]    = EXCEPTION_MKPRIO(5, 0),
        },
};

static const usb_string_descriptor_t STRING_EN_CA_MANUFACTURER =
    USB_STRING_DESCRIPTOR_INITIALIZER(u"UBC Thunderbots Football Club");
static const usb_string_descriptor_t STRING_EN_CA_PRODUCT =
    USB_STRING_DESCRIPTOR_INITIALIZER(u"Ball Speed Sensor");
static const usb_string_descriptor_t STRING_SERIAL =
    USB_STRING_DESCRIPTOR_INITIALIZER(u"                        ");
static const usb_string_descriptor_t *const STRINGS_EN_CA[] = {
    [STRING_INDEX_MANUFACTURER - 1U] = &STRING_EN_CA_MANUFACTURER,
    [STRING_INDEX_PRODUCT - 1U]      = &STRING_EN_CA_PRODUCT,
    [STRING_INDEX_SERIAL - 1U]       = &STRING_SERIAL,
};

/* Only one language at the moment. */
static const udev_language_info_t LANGUAGE_TABLE[] = {
    {.id = 0x1009U /* en_CA */, .strings = STRINGS_EN_CA},
    {.id = 0, .strings = 0},
};

static const usb_string_zero_descriptor_t STRING_ZERO = {
    .bLength         = sizeof(usb_string_zero_descriptor_t) + sizeof(uint16_t),
    .bDescriptorType = USB_DTYPE_STRING,
    .wLANGID =
        {
            0x1009U /* en_CA */,
        },
};

static bool usb_control_handler(const usb_setup_packet_t *UNUSED(pkt))
{
    return false;
}

STACK_ALLOCATE(usb_task_stack, 4096);

static const udev_info_t USB_INFO = {
    .flags =
        {
            .vbus_sensing        = 0,
            .minimize_interrupts = 0,
            .self_powered        = 0,
        },
    .internal_task_priority   = 4U,
    .internal_task_stack      = usb_task_stack,
    .internal_task_stack_size = sizeof(usb_task_stack),
    .receive_fifo_words       = 10U /* SETUP packets */ + 1U /* Global OUT NAK status */ +
                          ((64U / 4U) + 1U) * 2U /* Packets */ +
                          4U /* Transfer complete status */,
    .device_descriptor =
        {
            .bLength            = sizeof(usb_device_descriptor_t),
            .bDescriptorType    = USB_DTYPE_DEVICE,
            .bcdUSB             = 0x0200U,
            .bDeviceClass       = 0x00U,
            .bDeviceSubClass    = 0x00U,
            .bDeviceProtocol    = 0x00U,
            .bMaxPacketSize0    = 64U,
            .idVendor           = VENDOR_ID,
            .idProduct          = PRODUCT_ID,
            .bcdDevice          = 0x0101U,
            .iManufacturer      = STRING_INDEX_MANUFACTURER,
            .iProduct           = STRING_INDEX_PRODUCT,
            .iSerialNumber      = STRING_INDEX_SERIAL,
            .bNumConfigurations = 1U,
        },
    .string_count           = STRING_INDEX_COUNT - 1U /* exclude zero */,
    .string_zero_descriptor = &STRING_ZERO,
    .language_table         = LANGUAGE_TABLE,
    .control_handler        = &usb_control_handler,
    .configurations =
        {
            &USB_CONFIGURATION,
        },
};

void vApplicationIdleHook(void)
{
    asm volatile("wfi");
}

static void main_task(void *param) __attribute__((noreturn));
TaskHandle_t main_task_handle;



/***********************************************************
 *    		 	Timing Functions		   *
 ***********************************************************/
volatile uint32_t wrap_count;

static void tic_toc_setup(void)
{
    rcc_enable_reset(APB1, TIM2);

    {
        TIM2_5_CR1_t tmp = {
            .CKD  = 0,  // Timer runs at full clock frequency.
            .ARPE = 0,  // Auto-reload register is not buffered.
            .CMS  = 0,  // Counter counts in one direction.
            .DIR  = 0,  // Counter counts up.
            .OPM  = 0,  // Counter counts forever.
            .URS  = 0,  // Counter overflow, UG bit set, and slave mode update generate an
                        // interrupt.
            .UDIS = 0,  // Updates to control registers are allowed.
            .CEN  = 0,  // Counter is not counting right now.
        };
        TIM2.CR1 = tmp;
    }

    {
        TIM2_5_CR2_t tmp = {
            .TI1S = 0,  // TIM2_CH1 pin is connected to TI1.
            .MMS  = 0,  // TIM2.EGR.UG is sent as output to slave timers.
            .CCDS = 0,  // If DMA is enabled, requests are sent when CC events occur.
        };
        TIM2.CR2 = tmp;
    }

    {
        TIM2_5_SMCR_t tmp = {0};  // No external triggers or slave synchronization.
        TIM2.SMCR         = tmp;
    }

    {
        TIM2_5_DIER_t tmp = {
            .UIE = 1,  // Enable interrupt on timer update
        };
        TIM2.DIER = tmp;  // Enable interrupt on timer update
    }

    TIM2.PSC = 0b10000000;

    TIM2.ARR = 0x40;

    NVIC.ISER[NVIC_IRQ_TIM2 / 32] = 1 << (NVIC_IRQ_TIM2 % 32);
}

static void timer2_wrap_interrupt(void)
{
    wrap_count++;
    {
        TIM2_5_SR_t tmp = {0};  // Clear interrupt flag.
        TIM2.SR         = tmp;
    }

    EXCEPTION_RETURN_BARRIER();
}

static void tic(void)
{
    // clear counter, by setting UG
    TIM2.EGR.UG = 1;
    // enable clock, by setting CEN
    TIM2.CR1.CEN = 1;
    wrap_count   = 0;
}

static void toc(void)
{
    // stop clock, clearing all bits
    TIM2_5_CR1_t tmp = {0};
    TIM2.CR1         = tmp;
}

/***********************************************************
 *    util functions when for the lcd control output       *
 ***********************************************************/

// line states
#define LCD_READ true
#define LCD_WRITE false
#define LCD_COMMAND false
#define LCD_DATA true

// byte command that doesn't vary
#define LCD_CLEAR_SCREEN 0x00000001
#define LCD_HOME_SCREEN 0x00000010

// byte command that this prefer
#define LCD_FUNCTION_SET_P 0x00111100  // display on, two line mode
#define LCD_ON_CONTROL_P 0x00001100    // display one everything else off
#define LCD_ENTRY_MODE_P 0x00000110    // increment, entire shift off

// change lcd mode
static void LCD_switch_mode(bool signal_rs, bool signal_rw)
{
    gpio_set_output(PIN_LCD_RS, signal_rs);
    gpio_set_output(PIN_LCD_RW, signal_rw);
}

/***************************************************
 *		Print functions			   *
 ***************************************************/
// write
static void LCD_write(char a)
{
    gpio_set_reset_mask(GPIOC, a, 0xFF);
    LCD_switch_mode(LCD_DATA, LCD_WRITE);
    gpio_set(PIN_LCD_E);
    vTaskDelay(1);
    gpio_reset(PIN_LCD_E);
    vTaskDelay(1);
}

static void LCD_command(char a)
{
    gpio_set_reset_mask(GPIOC, a, 0xFF);
    LCD_switch_mode(LCD_COMMAND, LCD_WRITE);
    gpio_set(PIN_LCD_E);
    vTaskDelay(1);
    gpio_reset(PIN_LCD_E);
    vTaskDelay(1);
}

// initialize screen
static void LCD_init_routine()
{
    vTaskDelay(500U / portTICK_PERIOD_MS);  // recommended waiting time is 40ms
    LCD_command(0x30);
    vTaskDelay(1);
    LCD_command(0x30);
    vTaskDelay(1);
    LCD_command(0x30);
    vTaskDelay(1);
    LCD_command(0x38);  // function set
    vTaskDelay(1);
    LCD_command(0x1c);  // set cursor
    vTaskDelay(1);
    LCD_command(0x0c);  // display on, cursor on
    vTaskDelay(1);
    LCD_command(0x06);  // entry mode set
    vTaskDelay(1);
    LCD_command(0x02);  // return home
    vTaskDelay(1);
}

static void LCD_print(char *a, unsigned int size)
{
    unsigned int i = 0;
    for (i = 0; i < size; i++)
    {
        LCD_write(a[i]);
    }
}

/***************************************************
 *	Math: turning float to char		   *
 ***************************************************/
#if 0
static int ftoi_single ( float fl ){
	static int i = 0;
	for( i = 1; i < 10; i++ ){
		if( fl < i ){
			return i-1;
		}
	}
	return 0;
}
#endif
// turn to scientific notation, with 10 significant figures
#if 0
static void ftoa_sci ( float fl, char* a, int* dec ) {
	static unsigned int decimal_marker = 0;
	static int digit = 0;
	static unsigned int i = 0, j = 0;

	a[0] = (char)ftoi_single(fl);
	a[0] += 48;
	fl = (fl-ftoi_single(fl))*10;
	
	return;
}
#endif

#if 0
// output float to char array with dynamic scaling. The output is in the form of xx.xxxk, x.xxxx or xxx.xm
static void ftoa_tho (float fl, char* a, int size){
	char sci_a[10];
	int decimal_mark;
	int num1, num2;
	int i, j; // i represents the index of char array a, j represents the index of char array sci_a. There is a bit of shuffling here for inserting the decimal point.

	ftoa_sci(fl,sci_a,&decimal_mark);
	switch( decimal_mark ){
		case 	-6:
		case 	-5:
		case	-4:	a[size-1]='u'; break;
		case	-3:	
		case	-2:	
		case	-1:	a[size-1]='m';	break;
		case	0:
		case	1:
		case	2:	a[size-1]=' ';	break;
		case	3:
		case	4:
		case	5:	a[size-1]='k';	break;
		default	:	a[size-1]='x'; 	break;
	}
	num1 = (decimal_mark+12)%3;
	num2 = num1 +1; // the index of a where the decimal point should be.
	for( i = size-2, j=size-3; i>=0; i--, j-- ){
		if( i == num2 ){
		// then we insert the decimal point
			a[i]='.';
			i--;
			continue;
		}
		a[i]=sci_a[j];
	}
	return;
}
#endif

/***************************************************
 *			Main			   *
 ***************************************************/

static void stm32_main(void)
{
    // Initialize chip
    init_chip(&INIT_SPECS);

    // Set up pins
    gpio_init(PINS_INIT, sizeof(PINS_INIT) / sizeof(*PINS_INIT));

    // Start FreeRTOS
    static StaticTask_t main_task_tcb;
    STACK_ALLOCATE(main_task_stack, 4096);
    main_task_handle = xTaskCreateStatic(
        &main_task, "main", sizeof(main_task_stack) / sizeof(*main_task_stack), 0, 1U,
        main_task_stack, &main_task_tcb);
    vTaskStartScheduler();
    __builtin_unreachable();
}

static void main_task(void *UNUSED(param))
{
    // int counter_i = 0;
    char buffer[10];
    int temp;

    gpio_set(PIN_LED_RIGHT);
    gpio_set(PIN_LED_LEFT);
    vTaskDelay(1000U / portTICK_PERIOD_MS);
    gpio_reset(PIN_LED_RIGHT);
    gpio_reset(PIN_LED_LEFT);
    vTaskDelay(1000U / portTICK_PERIOD_MS);

    // setup interrupt
    rcc_enable_reset(APB2, SYSCFG);
    SYSCFG.EXTICR[3] = 0b0001000100010001;
    rcc_disable(APB2, SYSCFG);
    // Unmasking interrupts 13 and 12.
    EXTI.IMR = 0b0011000000000000;
    // Falling trigger enabled for interrupts 13 and 12.
    EXTI.FTSR                          = 0b0011000000000000;
    NVIC.ISER[NVIC_IRQ_EXTI15_10 / 32] = 1 << (NVIC_IRQ_EXTI15_10 % 32);

    gpio_reset(PIN_LCD_E);

    //  Initialize CDC ACM.
    cdcacm_init(2U, PRIO_TASK_CDC_ACM);

    //  Initialize USB.
    udev_init(&USB_INFO);
    udev_attach();

    // Handle activity
    tic_toc_setup();

    // Initialize LCD
    LCD_init_routine();
    formatuint8(buffer, wrap_count);
    LCD_command(0x02);
    LCD_print(buffer, 8);

    // Turn on LED
    gpio_set(PIN_LED_RIGHT);
    gpio_reset(PIN_LED_LEFT);
    // TODO print a setup message
    // check syscalls.c under firmware main to see the connection
    // between cdcacm and printing out
    iprintf("Test\r\n");

    // Wait a bit
    vTaskDelay(100U / portTICK_PERIOD_MS);

    for (;;)
    {
        if (G_status == 2)
        {
            if (wrap_count != 0)
            {
                temp = 1716000 / wrap_count;
                formatuint8(buffer, temp);
                // TODO Send buffer over USB here.
                iprintf("%i mm/s\r\n", temp);
                LCD_command(0x02);  // return home
                vTaskDelay(1);
                LCD_print(buffer, 8);
            }
            G_status = 0;
        }
        vTaskDelay(1);
    }
    __builtin_unreachable();
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                   uint32_t *stack_size)
{
    static StaticTask_t tcb_storage;
    *tcb = &tcb_storage;
    STACK_ALLOCATE(stack_storage, configMINIMAL_STACK_SIZE * sizeof(StackType_t));
    *stack      = stack_storage;
    *stack_size = sizeof(stack_storage) / sizeof(StackType_t);
}

void vApplicationGetTimerTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                    uint32_t *stack_size)
{
    static StaticTask_t tcb_storage;
    *tcb = &tcb_storage;
    STACK_ALLOCATE(stack_storage, configTIMER_TASK_STACK_DEPTH * sizeof(StackType_t));
    *stack      = stack_storage;
    *stack_size = sizeof(stack_storage) / sizeof(StackType_t);
}
