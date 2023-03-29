#include "usb.h"
#include "audio.h"

/**
 * @brief A structure that holds the state of endpoint 1.
 */
static USBOutEndpointState endpoint1_state;

/**
 * @brief The configuration structure for endpoint 1.
 */
static const USBEndpointConfig endpoint1_config = {
    USB_EP_MODE_TYPE_ISOC,
    NULL,
    NULL,
    audio_received_cb,
    0x0000,
    AUDIO_MAX_PACKET_SIZE,
    NULL,
    &endpoint1_state,
    1,
    NULL};

/**
 * @brief A structure that holds the state of endpoint 2.
 */
static USBInEndpointState endpoint2_state;

/**
 * @brief The configuration structure for endpoint 2.
 */
static const USBEndpointConfig endpoint2_config = {
    USB_EP_MODE_TYPE_ISOC,
    NULL,
    audio_feedback_cb,
    NULL,
    0x0004,
    0x0000,
    &endpoint2_state,
    NULL,
    1,
    NULL};

/**
 * @brief Handles global events that the USB driver triggers.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param event The event that was triggered.
 */
void usb_event_cb(USBDriver *usbp, usbevent_t event)
{
    struct audio_context *p_audio_context = audio_get_context();

    chSysLockFromISR();
    chEvtBroadcastFlagsI(&p_audio_context->audio_events, AUDIO_EVENT_USB_STATE);
    chSysUnlockFromISR();

    switch (event)
    {
    case USB_EVENT_RESET:
        audio_stop_playback_cb(usbp);
        return;
    case USB_EVENT_ADDRESS:
        return;
    case USB_EVENT_CONFIGURED:
        // Enables configured endpoints.
        chSysLockFromISR();
        usbInitEndpointI(usbp, AUDIO_PLAYBACK_ENDPOINT, &endpoint1_config);
        usbInitEndpointI(usbp, AUDIO_FEEDBACK_ENDPOINT, &endpoint2_config);
        chSysUnlockFromISR();
        return;
    case USB_EVENT_SUSPEND:
        return;
    case USB_EVENT_WAKEUP:
        return;
    case USB_EVENT_STALLED:
        return;
    case USB_EVENT_UNCONFIGURED:
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
const USBDescriptor *usb_get_descriptor_cb(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang)
{
    (void)usbp;
    (void)lang;
    switch (dtype)
    {
    case USB_DESCRIPTOR_DEVICE:
        return &audio_device_descriptor;

    case USB_DESCRIPTOR_CONFIGURATION:
        return &audio_configuration_descriptor;

    case USB_DESCRIPTOR_STRING:
        if (dindex < 4)
            return &audio_strings[dindex];
    }
    return NULL;
}