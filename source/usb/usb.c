// Copyright 2023 elagil

/**
 * @file
 * @brief   General USB setup and callback handlers.
 *
 * @addtogroup usb
 * @{
 */

#include "usb.h"

#include "audio_feedback.h"
#include "audio_playback.h"

/**
 * @brief Settings structure for the USB driver.
 */
static const USBConfig g_usb_config = {
    .event_cb          = usb_event_cb,
    .get_descriptor_cb = usb_get_descriptor_cb,
    .requests_hook_cb  = audio_request_hook_cb,
    .sof_cb            = NULL,
};

/**
 * @brief A structure that holds the state of endpoint 1.
 */
static USBOutEndpointState endpoint1_out_state;

/**
 * @brief The configuration structure for endpoint 1.
 */
static const USBEndpointConfig endpoint1_config = {.ep_mode       = USB_EP_MODE_TYPE_ISOC,
                                                   .setup_cb      = NULL,
                                                   .in_cb         = NULL,
                                                   .out_cb        = audio_playback_received_cb,
                                                   .in_maxsize    = 0u,
                                                   .out_maxsize   = AUDIO_MAX_PACKET_SIZE,
                                                   .in_state      = NULL,
                                                   .out_state     = &endpoint1_out_state,
                                                   .in_multiplier = 1u,
                                                   .setup_buf     = NULL};

/**
 * @brief A structure that holds the state of endpoint 2.
 */
static USBInEndpointState endpoint2_in_state;

/**
 * @brief The configuration structure for endpoint 2.
 */
static const USBEndpointConfig endpoint2_config = {.ep_mode       = USB_EP_MODE_TYPE_ISOC,
                                                   .setup_cb      = NULL,
                                                   .in_cb         = audio_feedback_cb,
                                                   .out_cb        = NULL,
                                                   .in_maxsize    = USB_DESC_MAX_IN_SIZE,
                                                   .out_maxsize   = 0u,
                                                   .in_state      = &endpoint2_in_state,
                                                   .out_state     = NULL,
                                                   .in_multiplier = 1u,
                                                   .setup_buf     = NULL};

/**
 * @brief Handles global events that the USB driver triggers.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param event The event that was triggered.
 */
void usb_event_cb(USBDriver *usbp, usbevent_t event) {
    switch (event) {
        case USB_EVENT_ADDRESS:
            return;

        case USB_EVENT_CONFIGURED:
            // Enables configured endpoints.
            chSysLockFromISR();
            usbInitEndpointI(usbp, AUDIO_PLAYBACK_ENDPOINT, &endpoint1_config);
            usbInitEndpointI(usbp, AUDIO_FEEDBACK_ENDPOINT, &endpoint2_config);
            chSysUnlockFromISR();
            return;

        case USB_EVENT_RESET:
        case USB_EVENT_UNCONFIGURED:
        case USB_EVENT_SUSPEND:
            audio_reset(usbp);
            return;

        case USB_EVENT_WAKEUP:
            return;

        case USB_EVENT_STALLED:
            return;
    }
    return;
}
/**
 * @brief Callback function that returns the requested USB descriptor.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param dtype The descriptor type that is requested.
 * @param dindex The index of data within the descriptor to fetch.
 * @param lang Currently unused - language specifier.
 * @return const USBDescriptor* The requested descriptor.
 */
const USBDescriptor *usb_get_descriptor_cb(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang) {
    (void)usbp;
    (void)lang;
    switch (dtype) {
        case USB_DESCRIPTOR_DEVICE:
            return &audio_device_descriptor;

        case USB_DESCRIPTOR_CONFIGURATION:
            return &audio_configuration_descriptor;

        case USB_DESCRIPTOR_STRING:
            if (dindex < 4) return &audio_strings[dindex];
    }
    return NULL;
}

/**
 * @brief Activate the USB peripheral.
 */
void usb_setup(void) {
    usbDisconnectBus(&USB_DRIVER);
    usbStart(&USB_DRIVER, &g_usb_config);
    usbConnectBus(&USB_DRIVER);
}

/**
 * @}
 */
