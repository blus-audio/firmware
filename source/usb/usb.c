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
 * @param p_usb A pointer to the USB driver structure.
 * @param event The event that was triggered.
 */
void usb_event_cb(USBDriver *p_usb, usbevent_t event) {
    switch (event) {
        case USB_EVENT_ADDRESS:
            return;

        case USB_EVENT_CONFIGURED:
            // Enables configured endpoints.
            chSysLockFromISR();
            usbInitEndpointI(p_usb, USB_DESC_ENDPOINT_PLAYBACK, &endpoint1_config);
            usbInitEndpointI(p_usb, USB_DESC_ENDPOINT_FEEDBACK, &endpoint2_config);
            chSysUnlockFromISR();
            return;

        case USB_EVENT_RESET:
        case USB_EVENT_UNCONFIGURED:
        case USB_EVENT_SUSPEND:
            audio_reset(p_usb);
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
 * @param p_usb A pointer to the USB driver structure.
 * @param descriptor_type The descriptor type that is requested.
 * @param data_index The index of data within the descriptor to fetch.
 * @param language Currently unused - language specifier.
 * @return const USBDescriptor* The requested descriptor.
 */
const USBDescriptor *usb_get_descriptor_cb(USBDriver *p_usb, uint8_t descriptor_type, uint8_t data_index,
                                           uint16_t language) {
    (void)p_usb;
    (void)language;

    switch (descriptor_type) {
        case USB_DESCRIPTOR_DEVICE:
            return &audio_device_descriptor;

        case USB_DESCRIPTOR_CONFIGURATION:
            return &audio_configuration_descriptor;

        case USB_DESCRIPTOR_STRING:
            if (data_index < ARRAY_LENGTH(audio_strings)) {
                return &audio_strings[data_index];
            }
            return NULL;
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
