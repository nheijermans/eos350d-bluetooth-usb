#include "uhi_canon.h"
#include "led.h"
#include "string.h"
#include "stdio.h"
#include "utils.h"
#include "camera_control.h"

/** @brief EOS-350D's USB Vendor ID. */
#define EOS_350D_VID 0x04A9

/** @brief EOS-350D's USB Product ID. */
#define EOS_350D_PID 0x30EE


/* @brief USB Canon device information. */
typedef struct
{
    uhc_device_t *dev;
    usb_ep_t bulk_in;
    usb_ep_t bulk_out;
    usb_ep_t interrupt_in;

    /** @brief SETUP request. */
    usb_setup_req_t req;

    /** @brief Buffer used for SETUP data transfers. */
    uint8_t buf[2048];

    bool ready;

    bool initialized;

    uint32_t sequence_number;

} uhi_canon_dev_t;


/** @brief Structure to hold state of enumerated camera device. */
static uhi_canon_dev_t s_camera;

/** @brief Convenience macro to keep camera references as a pointers. */
#define camera (&s_camera)


/** @brief Flag indicating that a setup request has completed. */
static volatile bool setup_req_done;


/** @brief Flag indicating that a bulk transfer has completed. */
static volatile bool bulk_transfer_done;

/** @brief Status of the completed bulk transfer. */
static volatile uhd_trans_status_t bulk_transfer_status;

/** @brief Actual number of bytes transferred during the bulk transfer. */
static volatile iram_size_t bulk_bytes_transferred;


/** @brief Flag indicating that an interrupt transfer has completed. */
static volatile bool interrupt_transfer_done;

/** @brief Status of the completed interrupt transfer. */
static volatile uhd_trans_status_t interrupt_transfer_status;

/** @brief Actual number of bytes transferred during the interrupt transfer. */
static volatile iram_size_t interrupt_bytes_transferred;


/** @brief Flag indicating that a shutter release was requested. */
static bool shutter_release_requested = false;

/** @brief Flag indicating that a shutter release has been completed. */
static bool shutter_release_complete = false;

/** @brief The image key of the last remote-capture image. */
static uint32_t last_image_key = 0;

/** @brief The size, in bytes, of the last remote-capture image. */
static uint32_t last_image_size = 0;


/** @brief Flag indicating that an image was requested.*/
static bool image_requested = false;

/** @brief Image key that was requested. */
static uint32_t requested_image_key = 0;

/** @brief Requested image chunk size. */
static uint32_t requested_chunk_size = 0;

/** @brief Callback function to call for each chunk of the picture. */
static process_image_chunk_cb_t process_image_chunk = NULL; 

/** @brief Storage for a good-sized chunk of image data. */
static uint8_t image_buf[0x1000];


/******************************************************************************
  GENERIC USB SUPPORT
******************************************************************************/
void
camera_control_usb_mode_change(bool host_mode_enabled)
{
}

void
camera_control_usb_vbus_change(bool b_vbus_present)
{

}


void
camera_control_usb_wakeup_event(void)
{

}

void
camera_control_usb_enum_event(uhc_device_t *dev, uhc_enum_status_t status)
{

}


/******************************************************************************
    initialize_endpoints
*//**
    @brief Initializes USB host endpoints.
******************************************************************************/
static uhc_enum_status_t
initialize_endpoints(uhc_device_t *dev)
{
    bool interface_supported = true;
    uint16_t size = le16_to_cpu(dev->conf_desc->wTotalLength);
    uint8_t *descriptor = (uint8_t*) dev->conf_desc;

    while (size)
    {
        uint8_t bLength = descriptor[0];
        uint8_t bDescriptorType = descriptor[1];

        switch (bDescriptorType)
        {
        case USB_DT_INTERFACE:
            break;

        case USB_DT_ENDPOINT:
            if (interface_supported)
            {
                usb_ep_desc_t *ep = (usb_ep_desc_t *) descriptor;
                U8 ep_type = ep->bmAttributes & USB_EP_TYPE_MASK;
                U8 ep_is_in = (ep->bEndpointAddress & USB_EP_DIR_IN) == USB_EP_DIR_IN;

                if (!ep_is_in && ep_type == USB_EP_TYPE_BULK)
                {
                    camera->bulk_out = ep->bEndpointAddress;
                }

                if (ep_is_in && ep_type == USB_EP_TYPE_BULK)
                {
                    camera->bulk_in = ep->bEndpointAddress;
                }

                if (ep_is_in && ep_type == USB_EP_TYPE_INTERRUPT)
                {
                    camera->interrupt_in = ep->bEndpointAddress;
                }

                if (!uhd_ep_alloc(dev->address, ep))
                {
                    return UHC_ENUM_HARDWARE_LIMIT;
                }

                if (camera->bulk_in && camera->bulk_out && camera->interrupt_in)
                {
                    return UHC_ENUM_SUCCESS;
                }

            }
            break;
        }

        size -= bLength;
        descriptor += bLength;
    }

    return UHC_ENUM_UNSUPPORTED;
}


/******************************************************************************
    uhi_canon_install
*//**
    @brief USB host interface installation function. Gets everything set up
           to control a Canon EOS-350D camera.
******************************************************************************/
uhc_enum_status_t
uhi_canon_install(uhc_device_t * dev)
{
    uhc_enum_status_t status = UHC_ENUM_UNSUPPORTED;

    U16 vid = le16_to_cpu(dev->dev_desc.idVendor);
    U16 pid = le16_to_cpu(dev->dev_desc.idProduct);

    if (vid == EOS_350D_VID && pid == EOS_350D_PID)
    {
        camera->dev = dev;
        status = initialize_endpoints(dev);
    }

    if (status != UHC_ENUM_SUCCESS)
    {
        LED_On(LED6);
    }

    return status;
}


/******************************************************************************
    uhi_canon_enable
*//**
    @brief Callback indicating that the camera is enabled.
******************************************************************************/
void
uhi_canon_enable(uhc_device_t * dev)
{
    LED_On(LED0);
    camera->ready = true;
}


/******************************************************************************
    uhi_canon_enable
*//**
    @brief Callback indicating that the camera has been disconnected.
******************************************************************************/
void
uhi_canon_uninstall(uhc_device_t * dev)
{
    camera->ready = false;
    camera->initialized = false;

    LED_Off(LED0);
    LED_Off(LED6);
}


/******************************************************************************
    signal_setup_done
*//**
    @brief Callback signalling that a setup transfer has completed.
******************************************************************************/
static void
signal_setup_done(
        usb_add_t address,
        uhd_trans_status_t status,
        uint16_t nb_transferred)
{
    if (status)
    {
        DBG_TRACE(("Setup transfer done. addr=%u, status=%d, nbt=%u\n",
                   address, status, nb_transferred));
    }
    setup_req_done = true;
}


/******************************************************************************
    wait_for_setup_done
*//**
    @brief Waits until a setup transfer has completed.
******************************************************************************/
static void
wait_for_setup_done(void)
{
    while (!setup_req_done)
    {
        ;
    }
}


/******************************************************************************
    do_setup_request
*//**
    @brief Sends a USB setup request to the USB device.

    @param[in] request The request to issue to the device.
******************************************************************************/
void
do_setup_request(usb_setup_req_t *request)
{
    setup_req_done = false;

    uhd_setup_request(camera->dev->address,
                      request, camera->buf,
                      sizeof(camera->buf), NULL, signal_setup_done);
    wait_for_setup_done();
}


/******************************************************************************
    signal_bulk_transfer_done
*//**
    @brief Callback signalling that a bulk transfer has completed.
******************************************************************************/
static void
signal_bulk_transfer_done(
        usb_add_t address,
        uhd_trans_status_t status,
        iram_size_t nb_transferred)
{
    if (status)
    {
        DBG_TRACE(("Bulk transfer done. addr=%u, status=%d, nbt=%lu\n",
                address, status, (uint32_t) nb_transferred));
    }

    bulk_transfer_done = true;
    bulk_transfer_status = status;
    bulk_bytes_transferred = nb_transferred;
}


/******************************************************************************
    wait_for_bulk_transfer
*//**
    @brief Waits until a bulk transfer has completed.
******************************************************************************/
static void
wait_for_bulk_transfer(void)
{
    while (!bulk_transfer_done)
    {
        ;
    }
}


/******************************************************************************
    read_bulk_in
*//**
    @brief Reads data from the bulk in endpoint.

    @param[in] buf  Pointer to buffer where data should be stored.
    @param[in] size Maximum number of bytes to read.

    @retval Actual number of bytes read.
******************************************************************************/
static uint32_t
read_bulk_in(uint8_t *buf, uint16_t size)
{
    bulk_transfer_done = false;
    bulk_bytes_transferred = 0;

    DBG_TRACE(("Reading %u bytes from ep %x.\n", size, camera->bulk_in));

    uhd_ep_run(camera->dev->address,
               camera->bulk_in,
               true, buf, size, 5000,
               signal_bulk_transfer_done);

    wait_for_bulk_transfer();

    return bulk_bytes_transferred;
}


/******************************************************************************
    signal_interrupt_transfer_done
*//**
    @brief Callback signalling that an interrupt transfer has completed.
******************************************************************************/
static void
signal_interrupt_transfer_done(
        usb_add_t address,
        uhd_trans_status_t status,
        iram_size_t nb_transferred)
{
    interrupt_transfer_done = true;
    interrupt_transfer_status = status;
    interrupt_bytes_transferred = nb_transferred;
}


/******************************************************************************
    wait_for_interrupt_transfer
*//**
    @brief Waits until an interrupt transfer has completed.
******************************************************************************/
static void
wait_for_interrupt_transfer(void)
{
    while (!interrupt_transfer_done)
    {
        ;
    }
}


/******************************************************************************
    read_interrupt_in
*//**
    @brief Reads data from the interrupt in endpoint.

    @param[in] buf  Pointer to buffer where data should be stored.
    @param[in] size Maximum number of bytes to read.

    @retval Actual number of bytes read.
******************************************************************************/
static uint32_t
read_interrupt_in(uint8_t *buf, uint16_t size)
{
    interrupt_transfer_done = false;
    interrupt_bytes_transferred = 0;

    uhd_ep_run(camera->dev->address,
               camera->interrupt_in,
               true, buf, size, 500,
               signal_interrupt_transfer_done);

    wait_for_interrupt_transfer();

    return interrupt_bytes_transferred;
}


/******************************************************************************
  USB COMMAND LOGIC FOR CANON CAMERAS.
******************************************************************************/
/** @brief Canon USB command structure. */
typedef struct usb_command_t {
    uint8_t cmd1;
    uint8_t cmd2;
    uint32_t cmd3;
    uint32_t reply_length;
    char *name;
} usb_command_t;


/** @brief Canon USB command structure for control subcommands. */
typedef struct control_command_t
{
    uint8_t subcmd;
    uint16_t cmd_length;
    uint16_t reply_length;
    char *name;
} control_command_t;

/* Generate the USB command structures. */
#define D(a, b, c, d, e) \
    static usb_command_t (a) = { (b), (c), (d), (e), #a };
# include "usb_commands.txt"
#undef D

/* Generate the control sub-command structures. */
#define D(a, b, c, d) \
    static control_command_t (a) = { (b), (c), (d), #a };
# include "control_commands.txt"
#undef D


/******************************************************************************
    send_command
*//**
    @brief Issues a USB command to the camera.

    @param[in] cmd              The USB command to issue.
    @param[in] payload          Payload buffer to send after command.
    @param[in] payload_length   Number of payload bytes to send.
    @param[in] out_buf          Buffer to read response into.
    @param[in] out_buf_size     Buffer size available.
    @param[in] reply_length     Anticipated additional reply length.

    @return Number of reply bytes actually read.
******************************************************************************/
static uint32_t
send_command(
    usb_command_t *cmd,
    uint8_t *payload,
    uint32_t payload_length,
    uint8_t *out_buf,
    uint32_t out_buf_size,
    uint16_t reply_length)
{
    usb_setup_req_t* req = &camera->req;
    uint8_t *cmd_buf = camera->buf;
    uint32_t length = 0x10 + payload_length;
    uint32_t res;

    /* Clear out the command buffer. */
    memset(camera->buf, 0x00, sizeof(camera->buf));

    /* Put together the command packet. */
    storeLittleU32(cmd_buf, length);
    storeLittleU32(cmd_buf + 4, cmd->cmd3);
    cmd_buf[0x40] = 0x02;
    cmd_buf[0x44] = cmd->cmd1;
    cmd_buf[0x46] = cmd->cmd3 == 0x202? 0x20: 0x10;
    cmd_buf[0x47] = cmd->cmd2;
    storeLittleU32(cmd_buf + 0x48, length);
    storeLittleU32(cmd_buf + 0x4c, camera->sequence_number++);

    if (payload_length)
    {
        memcpy(cmd_buf + 0x50, payload, payload_length);
    }

    req->bmRequestType = 0x40;
    req->bRequest = (payload_length > 1)? 0x04: 0x0c;
    req->wValue = 0x0010;
    req->wIndex = 0x0000;
    req->wLength = 0x50 + payload_length;

    DBG_TRACE(("\nCommand %s (cmd1=%x, cmd2=%x, cmd3=%lx, reply=%lx)\n",
               cmd->name, cmd->cmd1, cmd->cmd2, cmd->cmd3, cmd->reply_length));

    DBG_TRACE(("Sending packet (len=%lx):\n", 0x50 + payload_length);
               dump_hex(cmd_buf, req->wLength));

    do_setup_request(req);

    res = read_bulk_in(out_buf, 
                       MIN(cmd->reply_length + reply_length, out_buf_size));

    DBG_TRACE(("\nReceived packet (len=%lu)\n", res));
    dump_hex(out_buf, res);

    return res;
}


/******************************************************************************
    control_command
*//**
    @brief Issues a USB control command to the camera.

    @param[in] cmd      The control command to issue.
    @param[in] arg1     First command-specific argument.
    @param[in] arg2     Second command-specific argument.

    @return Number of reply bytes read.
******************************************************************************/
static uint32_t
control_command(
    control_command_t *cmd,
    uint32_t arg1,
    uint32_t arg2,
    uint8_t *reply_buf,
    uint32_t reply_len)
{
    uint8_t payload[ 3 * sizeof(uint32_t) ];
    uint8_t out_buf[1024];
    uint32_t payload_size = cmd->cmd_length - 0x10;
    uint32_t ret;

    storeLittleU32(payload, cmd->subcmd);
    storeLittleU32(payload + 4, arg1);
    storeLittleU32(payload + 8, arg2);

    payload_size = MIN(sizeof(payload), payload_size);

    DBG_TRACE(("\nControl command: %s (value=%u, len=%x, reply=%x)\n",
               cmd->name, cmd->subcmd, cmd->cmd_length, cmd->reply_length));

    dump_hex(payload, sizeof(payload));

    ret = send_command(&cmd_remote_control_new,
                       payload, payload_size,
                       out_buf, sizeof(out_buf),
                       cmd->reply_length);

    if (ret > 0x5c && reply_len >= (ret - 0x5c))
    {
        /* Copy the reply into the caller's reply buffer. */
        memcpy(reply_buf, out_buf + 0x5c, ret - 0x5c);
    }

    return ret;
}


/******************************************************************************
    get_status
*//**
    @brief Requests thet camera's currents status.

    @retval 'A' The camera is actively connected. 
    @retval 'C' The camera hasn't been connected yet.
******************************************************************************/
static uint8_t
get_status(void)
{
    usb_setup_req_t *req = &camera->req;

    /* Request the camera status. */
    req->bmRequestType = 0xc0;
    req->bRequest = 0x0c;
    req->wValue = 0x0055;
    req->wIndex = 0x0000;
    req->wLength = 0x0001;

    do_setup_request(req);

    return camera->buf[0];
}


/******************************************************************************
    get_initial_response
*//**
    @brief Retrieves an "initial response" buffer from the camera.

    Some fields in this response can be used to determine maximum transfer
    sizes or other run-time parameters.
******************************************************************************/
static void
get_initial_response(void)
{
    usb_setup_req_t *req = &camera->req;

    req->bmRequestType = 0xc0;
    req->bRequest = 0x04;
    req->wValue = 0x0001;
    req->wIndex = 0x0000;
    req->wLength = 0x58;

    do_setup_request(req);
}


/******************************************************************************
    control_init
*//**
    @brief Puts the camera into remote control mode.
******************************************************************************/
static uint32_t
control_init(void)
{
    return control_command(&control_cmd_init, 0, 0, NULL, 0);
}


/******************************************************************************
    control_exit
*//**
    @brief Takes the camera out of remote control mode.
******************************************************************************/
static uint32_t
control_exit(void)
{
    return control_command(&control_cmd_exit, 0, 0, NULL, 0);
}


/******************************************************************************
    lock_keys
*//**
    @brief Locks the camera's keys and LCD (presumably).
******************************************************************************/
static uint32_t
lock_keys(uint32_t arg)
{
    uint8_t payload[4];

    storeLittleU32(payload, arg);

    return send_command(&cmd_lock_keys, payload, 4, camera->buf, sizeof(camera->buf), 0);
}


/******************************************************************************
    unlock_keys
*//**
    @brief Unlocks the camera's keys and LCD (presumably).
******************************************************************************/
static uint32_t
unlock_keys(void)
{
    return send_command(&cmd_unlock_keys, NULL, 0, camera->buf, sizeof(camera->buf), 0);
}


/** @brief Transfer modes supported with cemote capture. */
typedef enum {
    REMOTE_CAPTURE_THUMB_TO_PC = 0x0001,
    REMOTE_CAPTURE_FULL_TO_PC = 0x0002,
    REMOTE_CAPTURE_THUMB_TO_DRIVE = 0x0003,
    REMOTE_CAPTURE_FULL_TO_DRIVE = 0x0004,
} remote_capture_mode_t;

/******************************************************************************
    set_transfer_mode
*//**
    @brief Sets the transfer mode that is to be used for remote captures.
******************************************************************************/
static uint32_t
set_transfer_mode(remote_capture_mode_t mode)
{
    return control_command(&control_cmd_set_transfer_mode, 4, mode, NULL, 0);
}

static uint32_t current_image_size = 0;
static uint32_t current_chunk_idx = 0;

/******************************************************************************
    get_captured_image
*//**
    @brief Sets the transfer mode that is to be used for remote captures.
******************************************************************************/
static void
get_captured_image(uint32_t image_key)
{
    uint8_t cmd_payload[16];
    uint8_t reply[0x40];
    uint32_t transfer_length = requested_chunk_size;
    uint32_t res;

#define CAMERA_DOWNLOAD_FULL 2

    storeLittleU32(cmd_payload + 0, 0);
    storeLittleU32(cmd_payload + 4, transfer_length);
    storeLittleU32(cmd_payload + 8, CAMERA_DOWNLOAD_FULL);
    storeLittleU32(cmd_payload + 12, image_key);

    /* This command returns the image size. */
    res = send_command(&cmd_retrieve_capture2,
                       cmd_payload, sizeof(cmd_payload),
                       reply, sizeof(reply),
                       0);

    current_image_size = fetchLittleU32(reply + 6);
    current_chunk_idx = 0;
}


static void
read_next_image_chunk(void)
{
    uint32_t transfer_length = requested_chunk_size;
    uint32_t to_read = MIN(current_image_size, transfer_length);
    uint32_t nread = read_bulk_in(image_buf, to_read);

    current_image_size -= nread;

    if (process_image_chunk)
    {
        process_image_chunk(image_buf, nread);
    }
}


/******************************************************************************
    command_identify
*//**
    @brief Requests the camera to identify itself.
******************************************************************************/
static void
command_identify(void)
{
    uint8_t buf[0x9d];
    uint32_t reply_size;

    memset(buf, 0x00, sizeof(buf));
    reply_size = send_command(&cmd_identify, NULL, 0, buf, sizeof(buf), 0);
}


/******************************************************************************
    command_get_owner
*//**
    @brief Requests the camera to identify its owner.
******************************************************************************/
static void
command_get_owner(void)
{
    uint8_t buf[0x9d];
    uint32_t reply_size;

    memset(buf, 0x00, sizeof(buf));
    reply_size = send_command(&cmd_get_owner, NULL, 0, buf, sizeof(buf), 0);
}


void
camera_control_get_release_params(uint8_t *buf, uint32_t size)
{
    control_init();
    control_command(&control_cmd_get_release_params, 4, 0, buf, size);
    control_exit();
}

void
camera_control_set_release_params(uint8_t *buf, uint32_t size)
{
    control_init();
    control_command(&control_cmd_set_release_params, 0, 0, buf, size);
    control_exit();
}

/******************************************************************************
    release_shutter
*//**
    @brief Issues a sequence of commands to perform a remote capture.

    This function sets up the transfer mode to REMOTE_CAPTURE_FULL_TO_PC,
    indicating to the camera that the full picture will be retrieved by the
    USB host.

    This function stores local copies of the last image key and size returned
    by the camera after it has successfully taken a picture. These can be used
    to read image from the camera.

    Note that if captured images are not retrieved, the camera will not enter
    sleep mode, which could be detrimental to battery life.
******************************************************************************/
static void
release_shutter(void)
{
    bool done = false;
    uint8_t buf[0x40];

    control_init();
    set_transfer_mode(REMOTE_CAPTURE_FULL_TO_PC);
    lock_keys(6);
    control_command(&control_cmd_release_shutter, 0, 0, NULL, 0);

    while (!done)
    {
        /* The camera indicates completion via the interrupt pipe. */
        uint32_t nread = read_interrupt_in(buf, sizeof(buf));

        if (nread > 4)
        {
            DBG_TRACE(("Read %lu bytes from interrupt in endpoint:\n", nread));
            dump_hex(buf, nread);

            switch (buf[4])
            {
            case 0x0c:
                if (nread == 0x17)
                {
                    last_image_key = fetchLittleU32(buf + 0x0c);
                    last_image_size = fetchLittleU32(buf + 0x11);
                    done = true;
                    unlock_keys();
                }
                break;
            }
        }
    }

    control_exit();
}


/******************************************************************************
   PUBLICALLY EXPOSED CAMERA CONTROL API FUNCTIONS
******************************************************************************/


/******************************************************************************
    camera_control_camera_connected
*//**
    @brief Returns whether a camera is currently connected.
******************************************************************************/
bool
camera_control_camera_connected(void)
{
    return camera->initialized;
}


/******************************************************************************
    camera_control_request_shutter_release
*//**
    @brief Queues a remote capture.
******************************************************************************/
void
camera_control_request_shutter_release(void)
{
    shutter_release_requested = true;
    shutter_release_complete = false;
}


/******************************************************************************
    camera_control_shutter_release_complete
*//**
    @brief Returns whether the remote capture has completed.
******************************************************************************/
bool
camera_control_shutter_release_complete(void)
{
    return shutter_release_complete;
}


/******************************************************************************
    camera_control_get_picture_results
*//**
    @brief Returns the image key and image size of the last captured image.

    @param[out] image_key   Image key that can be sent to the camera.
    @param[out] image_size  Size of the image, in bytes.
******************************************************************************/
void
camera_control_get_picture_results(uint32_t *image_key, uint32_t *image_size)
{
    *image_key = last_image_key;
    *image_size = last_image_size;
}


/******************************************************************************
    camera_control_request_image
*//**
    @brief Queues an image request.

    @param[in] image_key    Image key to request from the camera.
    @param[in] cb           Function to call for each chunk.
******************************************************************************/
void
camera_control_request_image(
    uint32_t image_key,
    uint32_t chunk_size,
    process_image_chunk_cb_t cb)
{
    requested_image_key = image_key;
    requested_chunk_size = MIN(chunk_size, sizeof(image_buf));
    process_image_chunk = cb;

    image_requested = true;
}


/******************************************************************************
    camera_task
*//**
    @brief The main loop task for communicating with the camera.
******************************************************************************/
void
camera_task(void)
{
    if (!camera->ready)
    {
        return;
    }

    if (!camera->initialized)
    {
        usb_setup_req_t* req = &camera->req;
        uint8_t status;

        status = get_status();

        get_initial_response();

        if (status == 'A')
        {
            req->bmRequestType = 0xc0;
            req->bRequest = 0x04;
            req->wValue = 0x0004;
            req->wIndex = 0x0000;
            req->wLength = 0x50;

            do_setup_request(req);
        }
        else
        {
            uint8_t buf[0x44];

            /* Send back a command containing the initial response received
               from the camera.
             */
            req->bmRequestType = 0x40;
            req->bRequest = 0x04;
            req->wValue = 0x0011;
            req->wIndex = 0x0000;
            req->wLength = 0x58;

            camera->buf[0] = 0x10;
            memset(camera->buf + 0x01, 0x00, 0x3f);
            memcpy(camera->buf + 0x40, camera->buf + 0x10, 0x10);

            do_setup_request(req);

            /* Now the camera should have 0x44 bytes available on bulk_in. */
            read_bulk_in(buf, 0x44);
        }

        command_identify();

        /* The camera is ready to be used now. */
        camera->initialized = true;

    }

    if (shutter_release_requested)
    {
        shutter_release_requested = false;
        release_shutter();
        shutter_release_complete = true;
    }

    if (image_requested)
    {
        static uint32_t last_image_key = -1;

        /* Check to see a new image is being requested. If so, we need to let
           the camera know.
         */
        if (last_image_key != requested_image_key)
        {
            get_captured_image(requested_image_key);
            last_image_key = requested_image_key;
        }

        read_next_image_chunk();
        image_requested = false;
    }
}
