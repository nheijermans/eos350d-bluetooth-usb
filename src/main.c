#include "compiler.h"
#include "preprocessor.h"
#include "board.h"
#include "gpio.h"
#include <sysclk.h>
#include <delay.h>
#include "sleepmgr.h"
#include "conf_usb_host.h"
#include "uhc.h"
#include "stdio_serial.h"
#include "utils.h"
#include "camera_control.h"
#include "rn42.h"

/* @brief Initializes board to use USART0 for stdio. */
static void
init_stdio(void)
{
    /* USART options. */
    static const usart_options_t options =
    {
        .baudrate     = 115200,
        .charlength   = 8,
        .paritytype   = USART_NO_PARITY,
        .stopbits     = USART_1_STOPBIT,
        .channelmode  = USART_NORMAL_CHMODE
    };

    stdio_serial_init(DBG_USART, &options);
}

#define USART3              (&AVR32_USART3)
#define USART3_RXD_PIN      AVR32_USART3_RXD_0_1_PIN        /* PX08, function 1. */
#define USART3_RXD_FUNCTION AVR32_USART3_RXD_0_1_FUNCTION
#define USART3_TXD_PIN      AVR32_USART3_TXD_0_1_PIN        /* PX09, function 1. */
#define USART3_TXD_FUNCTION AVR32_USART3_TXD_0_1_FUNCTION

/** @brief Pin connected to the RN42's status output signal. Note that the
           EVK1100 schematics incorrectly indicate that pin 5 is on the right.
*/
#define BT_STATUS_PIN       AVR32_PIN_PC04

static void
extra_board_init(void)
{
    /* GPIO pin configuration for USART3 */
    gpio_map_t usart3_map =
    {
        { USART3_RXD_PIN, USART3_RXD_FUNCTION },
        { USART3_TXD_PIN, USART3_TXD_FUNCTION },
    };
    gpio_enable_module(usart3_map, ARRAY_LENGTH(usart3_map));
}


int
main(void)
{
    sysclk_init();
    delay_init(F_CPU);

    irq_initialize_vectors();
    cpu_irq_enable();
    sleepmgr_init();

    board_init();
    extra_board_init();

    /* Set up STDIO to use one of the on-board UARTs. */
    init_stdio();

    /* Initialize our RN42 device driver. */
    rn42_init(USART3, AVR32_USART3_IRQ, BT_STATUS_PIN);

    /* Start USB host stack. */
    uhc_start();

    while (true)
    {
        rn42_task();
        camera_task();
    }
}
