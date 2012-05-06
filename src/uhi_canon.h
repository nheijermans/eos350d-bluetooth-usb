#ifndef UHI_CANON_H
#define UHI_CANON_H

#include "uhc.h"
#include "uhi.h"

#define UHI_CANON {                     \
    .install = uhi_canon_install,       \
    .enable = uhi_canon_enable,         \
    .uninstall = uhi_canon_uninstall,   \
    .sof_notify = NULL,                 \
}


uhc_enum_status_t
uhi_canon_install(uhc_device_t* dev);

void
uhi_canon_enable(uhc_device_t* dev);

void
uhi_canon_uninstall(uhc_device_t* dev);


bool
uhi_canon_is_available(void);


#endif /* UHI_CANON_H */
