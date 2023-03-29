#ifndef _USB_H_
#define _USB_H_

#include "hal.h"
#include "usb_descriptors.h"

// USB driver shorthand.
#define USB_DRIVER (USBD1)

void usb_event_cb(USBDriver *usbp, usbevent_t event);
const USBDescriptor *usb_get_descriptor_cb(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang);

#endif // _USB_H_
