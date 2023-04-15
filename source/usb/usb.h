// Copyright 2023 elagil

/**
 * @file
 * @brief   General USB setup and callback handlers: headers.
 *
 * @addtogroup usb
 * @{
 */

#ifndef SOURCE_USB_USB_H_
#define SOURCE_USB_USB_H_

#include "hal.h"
#include "usb_descriptors.h"

// USB driver shorthand.
#define USB_DRIVER (USBD1)

void                 usb_setup(void);
void                 usb_event_cb(USBDriver *usbp, usbevent_t event);
const USBDescriptor *usb_get_descriptor_cb(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang);

#endif  // SOURCE_USB_USB_H_

/**
 * @}
 */
