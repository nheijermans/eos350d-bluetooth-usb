/** 
    @file
    @brief Driver for the RN42 Bluetooth module.
    
    This code implements a datagram layer on top of a RN42 serial port
    protocol. Whenever a datagram has arrived, a user-specified callback is
    invoked to notify the necessary client code that data is available.
*/
#include <delay.h>
#include <gpio.h>
#include <pdca.h>
#include <serial.h>
#include <stdio.h>
#include <string.h>

#include "rn42.h"
#include "camera_control.h"
#include "utils.h"


/** @brief Magic bytes at the beggining of a command header. */
#define CMD_MAGIC   { 'C', 'C', 'M', 'D' }

/** @brief Channel to use for the USART DMA transfers (transmit). */
#define PDCA_CHANNEL_TX    1

/** @brief Channel to use for the USART DMA transfers (receive). */
#define PDCA_CHANNEL_RX    0

/** @brief DMA IRQ used for the USART DMA transfers (receive). */
#define PDCA_IRQ        PASTE(AVR32_PDCA_IRQ_, PDCA_CHANNEL_RX)

/** @brief Convenience macro for setting up the next DMA transfer. */
#define setup_usart_tx_dma(buf, length) \
    pdca_load_channel(PDCA_CHANNEL_TX, (buf), (length))

/** @brief Convenience macro for setting up the next DMA transfer. */
#define setup_usart_rx_dma(buf, length) \
    pdca_load_channel(PDCA_CHANNEL_RX, (buf), (length))

/** @brief Convenience macro for enabling DMA completion interrupts. */
#define usart_irq_enable() \
    pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_RX)

/** @brief Convenience macro for disabling DMA completion interrupts. */
#define usart_irq_disable() \
    pdca_disable_interrupt_transfer_complete(PDCA_CHANNEL_RX)


/** @brief Supported commands. */
enum {
    CMD_GET_STATUS = 0x30,
    CMD_TRIGGER_SHUTTER_RELEASE,
    CMD_GET_IMAGE,
    CMD_GET_RELEASE_PARAMS,
    CMD_SET_RELEASE_PARAMS,
};


/** @brief Status codes returned from commands. */
enum {
    STATUS_SUCCESS = 0x00,
    STATUS_CAMERA_DISCONNECTED = 0x80,
    STATUS_GENERAL_ERROR = 0xFF,
};


/** @brief Command structure definition. */
typedef struct PACKED command_t
{
    /** @brief Magic byte sequence indicating start of command. */
    uint8_t magic[4];

    /** @brief Number of bytes in the "data" array. */
    uint32_t data_length;
    
    /** @brief Command ID.  */
    uint8_t cmd_id;

    /** @brief Data associated with the command. */
    uint8_t data[0];

} command_t;


/** @brief Command response structure definition. */
typedef struct PACKED response_t
{
    /** @brief Magic byte sequence indicating start of command. */
    uint8_t magic[4];

    /** @brief Command ID.  */
    uint8_t cmd_id;

    /** @brief Status. */
    uint8_t status;

    /** @brief Number of bytes in the "data" array.  */
    uint32_t data_length;
} response_t;



/** @brief Pointer to the USART to which the RN42 is attached. */
static volatile avr32_usart_t *rn42_usart;

/** @brief Buffer to read Bluetooth characters into. */
static uint8_t bt_buf[1024];

/** @brief Buffer to command currently being processed. */
static uint8_t cmd_buf[1024];

/** @brief Flag indicating that a command is ready for the main loop. */
static volatile bool cmd_ready;

static uint32_t rn42_status_pin = 0;


/******************************************************************************
    usart_isr
*//**
    @brief Handles interrupts from the USART's PDCA channel.
******************************************************************************/
ISR(usart_isr, PDCA_IRQ, AVR32_INTC_INT0)
{
    static bool waiting_for_header = true;
    command_t *cmd = (command_t*) bt_buf;
    uint32_t data_length = fetchBigU32((uint8_t*) &cmd->data_length);

    LED_On(LED2);

    if (waiting_for_header && data_length)
    {
        /* Set up the PDCA to read the data. */
        waiting_for_header = false;
        setup_usart_rx_dma(cmd->data, data_length);
    }
    else
    {
        /* We have a complete command. Make it available to the main loop. */
        memcpy(cmd_buf, bt_buf, sizeof(command_t) + data_length);
        waiting_for_header = true;
        cmd_ready = true;
        usart_irq_disable();
    }
}


/******************************************************************************
    rn42_init
*//**
    @brief Initializes RN42 driver.

    @brief usart    Pointer to the USART hardware structure the RN42 is on.
    @brief irq      IRQ number to process interrupts on.
    @brief gpio_pin Pin connected to the RN42's status output.
******************************************************************************/
void
rn42_init(
    volatile avr32_usart_t *usart,
    uint32_t irq,
    uint32_t gpio_pin)
{ 
    static usart_options_t usart_options =
    {
        .baudrate     = 115200,
        .charlength   = 8,
        .paritytype   = USART_NO_PARITY,
        .stopbits     = USART_1_STOPBIT,
        .channelmode  = USART_NORMAL_CHMODE
    };

    static pdca_channel_options_t pdca_rx_options =
    {
        .pid = AVR32_PDCA_PID_USART3_RX,
        .addr = NULL,
        .size = 0,
        .r_addr = NULL,
        .r_size = 0,
        .transfer_size = PDCA_TRANSFER_SIZE_BYTE
    };

    static pdca_channel_options_t pdca_tx_options =
    {
        .pid = AVR32_PDCA_PID_USART3_TX,
        .addr = NULL,
        .size = 0,
        .r_addr = NULL,
        .r_size = 0,
        .transfer_size = PDCA_TRANSFER_SIZE_BYTE
    };

    /* Initialize the USART to use hardware handshaking. Without the
       handshaking, the RN42 has trouble keeping up when continuosly sending
       fairly small amounts of data. Note that when hardware handshaking is
       enabled, the PDCA must be used for receiving data.
     */
    rn42_usart = usart;
    usart_serial_init(rn42_usart, &usart_options);
    rn42_usart->mr = (rn42_usart->mr & ~0xf) | 0x02;

    /* Register the ISR for USART's DMA completion IRQs. */
    pdca_init_channel(PDCA_CHANNEL_RX, &pdca_rx_options);
    pdca_init_channel(PDCA_CHANNEL_TX, &pdca_tx_options);
    irq_register_handler(usart_isr, PDCA_IRQ, 0);

    setup_usart_rx_dma(bt_buf, sizeof(command_t));
    usart_irq_enable();
    pdca_enable(PDCA_CHANNEL_RX);
    pdca_enable(PDCA_CHANNEL_TX);

    /* Configure RN42 status GPIO. */
    gpio_configure_pin(gpio_pin, GPIO_DIR_INPUT | GPIO_PULL_DOWN);
    gpio_disable_pin_pull_up(gpio_pin);

    rn42_status_pin = gpio_pin;
}


/******************************************************************************
    rn42_connected
*//**
    @brief Returns whether the RN42 is connected by examining the status GPIO.

    The RN42's status line appears to bounce, so this function takes care of
    debouncing it.
******************************************************************************/
static bool
rn42_connected(void)
{
    bool connected1;
    bool connected2;

    do {
        connected1 = gpio_pin_is_high(rn42_status_pin);
        delay_ms(50);
        connected2 = gpio_pin_is_high(rn42_status_pin);
    } while (connected1 != connected2);

    return connected1 & connected2;
}


/******************************************************************************
    rn42_send_data
*//**
    @brief Transmits data to a Bluetooth peer over the RN42 module.

    @param[in] buf  Pointer to the data that should be sent.
    @param[in] size Number of bytes to send.
******************************************************************************/
void
rn42_send_data(uint8_t *buf, uint32_t size)
{
    uint32_t status;

    setup_usart_tx_dma(buf, size);

    while (true)
    {
        status = pdca_get_transfer_status(PDCA_CHANNEL_TX);

        if (status & PDCA_TRANSFER_COMPLETE)
        {
            break;
        }
    }
}


/******************************************************************************
    send_response
*//**
    @brief Sends a response packet to the host.

    @param[in] cmd_id       The command ID we're responding to.
    @param[in] status       The status code.
    @param[in] data         Pointer to data to send to the PC.
    @param[in] data_length  Number of bytes to send.
******************************************************************************/
static void
send_response(
    uint8_t cmd_id,
    uint8_t status,
    uint8_t *data,
    uint32_t data_length)
{
    response_t response = {
        {'C', 'C', 'M', 'D'},
        cmd_id,
        status,
    };
    storeBigU32(&response.data_length, data_length);

    rn42_send_data((uint8_t*) &response, sizeof(response));
    rn42_send_data(data, data_length);
}


/******************************************************************************
    send_image_chunk
*//**
    @brief Callback that sends an image chunk to the PC.

    @param[in] buf  Pointer to the image chunk to send to the PC.
    @param[in] size Number of bytes to send.
******************************************************************************/
static void
send_image_chunk(uint8_t *buf, uint32_t size)
{
    send_response(CMD_GET_IMAGE, STATUS_SUCCESS, buf, size);
}


/******************************************************************************
    process_commands
*//**
    @brief Function that processes incoming commands from the PC.
******************************************************************************/
static void
process_commands(void)
{
    static bool waiting_for_camera = false;

    if (waiting_for_camera)
    {
        if (camera_control_shutter_release_complete())
        {
            /* @bug Status might not always be success. */
            uint8_t status = STATUS_SUCCESS;
            uint32_t res[2];

            camera_control_get_picture_results(&res[0], &res[1]);

            /* Ensure big endian. */
            storeBigU32((uint8_t*) &res[0], res[0]);
            storeBigU32((uint8_t*) &res[1], res[1]);

            send_response(CMD_TRIGGER_SHUTTER_RELEASE, status,
                        (uint8_t*) res, sizeof(res));
            waiting_for_camera = false;
        }
        else
        {
            /* Don't process more commands until the camera's done. */
            return;
        }
    }

    if (cmd_ready)
    {
        command_t *command = (command_t*) cmd_buf;
        uint32_t data_length = fetchBigU32((uint8_t*) &command->data_length);

        dump_hex((uint8_t*) command, sizeof(command_t) + data_length);

        if (camera_control_camera_connected())
        {
            switch (command->cmd_id)
            {
            case CMD_GET_STATUS:
                send_response(command->cmd_id, STATUS_SUCCESS, NULL, 0);
                break;

            case CMD_TRIGGER_SHUTTER_RELEASE:
                camera_control_request_shutter_release();
                waiting_for_camera = true;
                break;

            case CMD_GET_RELEASE_PARAMS:
                {
                    uint8_t buf[0x40];

                    memset(buf, 0x00, sizeof(buf));
                    camera_control_get_release_params(buf, sizeof(buf));

                    send_response(command->cmd_id,
                                  STATUS_SUCCESS,
                                  buf, sizeof(buf));
                }
                break;

            case CMD_SET_RELEASE_PARAMS:
                /** @TODO This might generate errors. Return them back OTA! */
                camera_control_set_release_params(command->data, data_length);
                send_response(command->cmd_id, STATUS_SUCCESS, NULL, 0);
                break;

            case CMD_GET_IMAGE:
                if (data_length == 2 * sizeof(uint32_t))
                {
                    uint32_t image_key = fetchBigU32(command->data);
                    uint32_t chunk_size = fetchBigU32(command->data + 4);

                    camera_control_request_image(image_key,
                                                 chunk_size,
                                                 send_image_chunk);
                }
                break;

            default:
                send_response(command->cmd_id, 0xff, NULL, 0);
                printf("Ignoring unsupported command 0x%02x.\n",
                       command->cmd_id);
            }
        }
        else
        {
            send_response(command->cmd_id, STATUS_CAMERA_DISCONNECTED, NULL, 0);
        }

        cmd_ready = false;
        setup_usart_rx_dma(bt_buf, sizeof(command_t));
        usart_irq_enable();
    }

    LED_Off(LED2);
}


/******************************************************************************
    rn42_task
*//**
    @brief Handles receiving data from the RN42 bluetooth module.
******************************************************************************/
void
rn42_task(void)
{
    static bool connected = false;
    bool current_status = rn42_connected();

    if (connected != current_status)
    {
        connected = current_status;

        if (connected)
        {
            LED_On(LED1);
        }
        else
        {
            LED_Off(LED1);
        }
    }

    if (connected)
    {
        process_commands();
    }
}
