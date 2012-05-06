#ifndef HOST_EOS_TASK_H
#define HOST_EOS_TASK_H


//_____ I N C L U D E S ____________________________________________________

#include "conf_usb.h"

#if USB_HOST_FEATURE == false
  #error host_eos_task.h is #included although USB_HOST_FEATURE is disabled
#endif


#include "usb_host_task.h"


//_____ M A C R O S ________________________________________________________


//_____ D E C L A R A T I O N S ____________________________________________

void
host_canon_eos_task_init(void);

void
host_canon_eos_task(void);

void
host_sof_action(void);


#endif  // HOST_EOS_TASK_H
