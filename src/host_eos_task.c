#include "conf_usb.h"
#include "board.h"
#include "usb_drv.h"
#include "usb_host_enum.h"
#include "usb_host_task.h"
#include "host_eos_task.h"
#include "uhd.h"

static U16 sof_cnt;

static U8 ep_bulk_in = 0x81;
static U8 ep_bulk_out = 0x02;
static U8 ep_interrupt_in = 0x83;

volatile bool camera_connected = false;

static U8
do_control_transfer(
    U8 bmRequestType,
    U8 bRequest,
    U16 wValue,
    U16 wIndex,
    U16 wLength,
    U8 *pData)
{
    usb_setup_req_t req;

    req.bmRequestType = bmRequestType;
    req.bRequest = bRequest;
    req.wValue = wValue;
    req.wIndex = wIndex;
    req.wLength = wLength;

    uhd_setup_request(
            DEVICE_ADDRESS,
            &req,
            pData,
            wLength,
            NULL,
            NULL);

    return 0;
}


void
host_sof_action(void)
{
}


void
host_canon_eos_task_init(void)
{
    sof_cnt = 0;
    eos_new_device_connected = false;
}


enum {
    CAMERA_STATE_STARTING,
    CAMERA_STATE_INITIALIZED,
    CAMERA_STATE_BUSY,
};


void
host_canon_eos_task(void)
{
    if (Is_host_ready() && camera_connected)
    {
        /* Manage requests here. */
    }

    if (Is_host_suspended())
    {
        Host_request_resume();
    }
}


