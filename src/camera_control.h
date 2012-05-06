#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H


#include "compiler.h"
#include "uhc.h"


void
camera_control_usb_mode_change(bool host_mode_enabled);

void
camera_control_usb_vbus_change(bool b_vbus_present);

void
camera_control_usb_wakeup_event(void);

void
camera_control_usb_enum_event(uhc_device_t *dev, uhc_enum_status_t status);


void
camera_task(void);


/******************************************************************************
   PUBLICALLY EXPOSED CAMERA CONTROL API FUNCTIONS
******************************************************************************/
typedef void (*process_image_chunk_cb_t)(uint8_t *buf, uint32_t size);

/** @brief Returns whether the camera is currently connected. */
bool
camera_control_camera_connected(void);


/** @brief Requests that the camera take a picture. */
void
camera_control_request_shutter_release(void);

/** @brief Returns whether the picture has been taken. */
bool
camera_control_shutter_release_complete(void);

/** @brief Returns the results of the last shutter release. */
void
camera_control_get_picture_results(uint32_t *image_key, uint32_t *image_size);


/** @brief Requests a specific image key to be retrieved from the camera.
 */
void
camera_control_request_image(
    uint32_t image_key,
    uint32_t chunk_size,
    process_image_chunk_cb_t cb);

#endif /* CAMERA_CONTROL_H */

