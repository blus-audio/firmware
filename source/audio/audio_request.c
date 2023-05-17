// Copyright 2023 elagil

/**
 * @file
 * @brief   Audio request module.
 * @details Contains functionality for handling UAC 1.0 audio requests.
 *
 * @addtogroup audio
 * @{
 */

#include "audio_request.h"

#include <string.h>

#include "audio_playback.h"
#include "common.h"
#include "usb_descriptors.h"

// Control selectors
#define AUDIO_REQUEST_CS_SAMPLING_FREQ 0x01u

/**
 * @brief Supported control requests from the USB Audio Class.
 */
enum AUDIO_REQUEST {
    AUDIO_REQUEST_SET_CUR = 0x01u,
    AUDIO_REQUEST_SET_MIN = 0x02u,
    AUDIO_REQUEST_SET_MAX = 0x03u,
    AUDIO_REQUEST_SET_RES = 0x04u,
    AUDIO_REQUEST_GET_CUR = 0x81u,
    AUDIO_REQUEST_GET_MIN = 0x82u,
    AUDIO_REQUEST_GET_MAX = 0x83u,
    AUDIO_REQUEST_GET_RES = 0x84u,
};

/**
 * @brief A structure that holds the content of an audio request message.
 */
static volatile struct audio_request {
    uint8_t  request_type;  ///< The type of request.
    uint8_t  request;       ///< The request code itself.
    uint16_t value;         ///< The wValue field of the request.
    uint16_t index;         ///< The wIndex field of the request.
    uint16_t length;        ///< The valid data length.
    uint8_t  data[8u];      ///< The data buffer.
} g_request;

/**
 * @brief A structure that holds all information that can be controlled with requests.
 * @details Includes information about volume, mute, and sample rate.
 */
static volatile struct audio_request_controls {
    struct {
        uint8_t channel_index;                                      ///< The channel index.
        bool    b_channel_mute_states[AUDIO_CHANNEL_COUNT];         ///< Channel mute states.
        int16_t channel_volume_levels_8q8_db[AUDIO_CHANNEL_COUNT];  ///< Channel volumes in 8.8 format (in dB).
    } volume;                                                       ///< The volume control structure.
    uint32_t sample_rate_hz;                                        ///< The audio sample rate in Hz.
} g_controls;

/**
 * @brief A pointer to the \a audio module's mailbox.
 */
static mailbox_t *gp_mailbox;

/**
 * @brief Check the mute state of an audio channel.
 *
 * @param audio_channel The audio channel to check.
 * @return true if the channel is muted.
 * @return false if the channel is not muted.
 */
bool audio_request_is_channel_muted(enum AUDIO_COMMON_CHANNEL audio_channel) {
    return g_controls.volume.b_channel_mute_states[audio_channel];
}

/**
 * @brief Get the volume of an audio channel.
 *
 * @param audio_channel The audio channel, for which to get the volume.
 * @return int16_t The volume level in 8.8 fractional dB.
 */
int16_t audio_request_get_channel_volume(enum AUDIO_COMMON_CHANNEL audio_channel) {
    return g_controls.volume.channel_volume_levels_8q8_db[audio_channel];
}

/**
 * @brief Get the currently set sample rate in Hz.
 *
 * @return uint32_t The sample rate in Hz.
 */
uint32_t audio_request_get_sample_rate_hz(void) { return g_controls.sample_rate_hz; }

/**
 * @brief Update changed volume levels.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_request_update_volumes(USBDriver *usbp) {
    (void)usbp;
    uint8_t *p_data = (uint8_t *)g_request.data;

    chSysLockFromISR();
    if (g_controls.volume.channel_index == AUDIO_COMMON_CHANNEL_MASTER) {
        memcpy((int16_t *)g_controls.volume.channel_volume_levels_8q8_db, (int16_t *)p_data + sizeof(int16_t),
               2u * sizeof(int16_t));
    } else {
        size_t audio_channel_index = g_controls.volume.channel_index - 1u;
        chDbgAssert(audio_channel_index < ARRAY_LENGTH(g_controls.volume.channel_volume_levels_8q8_db),
                    "Invalid volume channel index.");

        memcpy((int16_t *)&g_controls.volume.channel_volume_levels_8q8_db[audio_channel_index], (int16_t *)p_data,
               sizeof(int16_t));
    }

    chMBPostI(gp_mailbox, AUDIO_COMMON_MSG_SET_VOLUME);

    chSysUnlockFromISR();
}

/**
 * @brief Update changed mute states.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_request_update_mute_states(USBDriver *usbp) {
    (void)usbp;
    uint8_t *p_data = (uint8_t *)g_request.data;

    chSysLockFromISR();
    if (g_controls.volume.channel_index == AUDIO_COMMON_CHANNEL_MASTER) {
        g_controls.volume.b_channel_mute_states[AUDIO_COMMON_CHANNEL_LEFT]  = p_data[1u];
        g_controls.volume.b_channel_mute_states[AUDIO_COMMON_CHANNEL_RIGHT] = p_data[2u];
    } else {
        size_t audio_channel_index = g_controls.volume.channel_index - 1u;
        chDbgAssert(audio_channel_index < ARRAY_LENGTH(g_controls.volume.b_channel_mute_states),
                    "Invalid mute channel index.");

        g_controls.volume.b_channel_mute_states[audio_channel_index] = p_data[0u];
    }

    chMBPostI(gp_mailbox, AUDIO_COMMON_MSG_SET_MUTE_STATE);

    chSysUnlockFromISR();
}

/**
 * @brief Update changed audio sample rate.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_request_update_sample_rate(USBDriver *usbp) {
    (void)usbp;
    uint8_t *p_data = (uint8_t *)g_request.data;

    byte_array_to_value(p_data, (uint32_t *)&g_controls.sample_rate_hz, g_request.length);

    chSysLockFromISR();
    chMBPostI(gp_mailbox, AUDIO_COMMON_MSG_SET_SAMPLE_RATE);
    chSysUnlockFromISR();
}

/**
 * @brief Handle class interface function unit requests.
 *
 * @param usbp A pointer to the  USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
static bool audio_request_handle_class_interface_fu(USBDriver *usbp) {
    uint8_t *p_data = (uint8_t *)g_request.data;

    uint8_t channel_index = GET_BYTE(g_request.value, 0u);
    uint8_t control_unit  = GET_BYTE(g_request.value, 1u);

    // In UAC 1.0, the volume level is given as an int16 value. An increment of 1 bit equals 1/256 dB of volume.
    // Do not change, this is defined by the standard.
    const int16_t AUDIO_VOLUME_STEPS_PER_DB = 256;

    switch (g_request.request) {
        case AUDIO_REQUEST_SET_MAX:
        case AUDIO_REQUEST_SET_MIN:
        case AUDIO_REQUEST_SET_RES:
            if (control_unit == UAC_FU_VOLUME_CONTROL) {
                usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                return true;
            }
            break;

        case AUDIO_REQUEST_GET_MAX:
            if (control_unit == UAC_FU_VOLUME_CONTROL) {
                for (size_t i = 0; i < g_request.length; i++) {
                    ((int16_t *)p_data)[i] = (int16_t)AUDIO_MAX_VOLUME_DB * AUDIO_VOLUME_STEPS_PER_DB;
                }
                usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                return true;
            }
            break;

        case AUDIO_REQUEST_GET_MIN:
            if (control_unit == UAC_FU_VOLUME_CONTROL) {
                for (size_t i = 0; i < g_request.length; i++) {
                    ((int16_t *)p_data)[i] = (int16_t)AUDIO_MIN_VOLUME_DB * AUDIO_VOLUME_STEPS_PER_DB;
                }
                usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                return true;
            }
            break;

        case AUDIO_REQUEST_GET_RES:
            if (control_unit == UAC_FU_VOLUME_CONTROL) {
                for (size_t i = 0; i < g_request.length; i++) {
                    ((int16_t *)p_data)[i] = (int16_t)AUDIO_VOLUME_INCREMENT_STEPS;
                }
                usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                return true;
            }
            break;

        case AUDIO_REQUEST_GET_CUR:
            if (control_unit == UAC_FU_MUTE_CONTROL) {
                if (channel_index == AUDIO_COMMON_CHANNEL_MASTER) {
                    uint8_t value[3] = {0, g_controls.volume.b_channel_mute_states[AUDIO_COMMON_CHANNEL_LEFT],
                                        g_controls.volume.b_channel_mute_states[AUDIO_COMMON_CHANNEL_RIGHT]};
                    memcpy(p_data, value, sizeof(value));
                    usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                    return true;
                } else if (channel_index >= 1) {
                    *p_data = g_controls.volume.b_channel_mute_states[channel_index - 1];
                    usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                    return true;
                }
                return true;
            } else if (control_unit == UAC_FU_VOLUME_CONTROL) {
                if (channel_index == AUDIO_COMMON_CHANNEL_MASTER) {
                    int16_t value[3] = {0, g_controls.volume.channel_volume_levels_8q8_db[AUDIO_COMMON_CHANNEL_LEFT],
                                        g_controls.volume.channel_volume_levels_8q8_db[AUDIO_COMMON_CHANNEL_RIGHT]};
                    memcpy(p_data, value, sizeof(value));
                    usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                    return true;
                } else if (channel_index >= 1) {
                    memcpy(p_data, (uint8_t *)&g_controls.volume.channel_volume_levels_8q8_db[channel_index - 1],
                           sizeof(int16_t));
                    usbSetupTransfer(usbp, p_data, g_request.length, NULL);
                    return true;
                }
            }
            break;

        case AUDIO_REQUEST_SET_CUR:
            if (control_unit == UAC_FU_MUTE_CONTROL) {
                g_controls.volume.channel_index = channel_index;
                usbSetupTransfer(usbp, p_data, g_request.length, audio_request_update_mute_states);
                return true;
            } else if (control_unit == UAC_FU_VOLUME_CONTROL) {
                g_controls.volume.channel_index = channel_index;
                usbSetupTransfer(usbp, p_data, g_request.length, audio_request_update_volumes);
                return true;
            }
            break;

        default:
            return false;
    }

    return false;
}

/**
 * @brief Handle USB audio control messages.
 *
 * @param usbp A pointer the USB driver structure.
 */
static bool audio_request_handle_class_interface(USBDriver *usbp) {
    uint8_t interface = GET_BYTE(g_request.index, 0u);
    uint8_t entity    = GET_BYTE(g_request.index, 1u);

    if (interface != AUDIO_CONTROL_INTERFACE) {
        // Only requests to audio control interface are supported.
        return false;
    }

    if (entity == AUDIO_FUNCTION_UNIT_ID) {
        // Handle requests to the audio function unit
        return audio_request_handle_class_interface_fu(usbp);
    }

    // No control message handling took place.
    return false;
}

/**
 * @brief Handle standard interface requests.
 *
 * @param usbp A pointer to the  USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
static bool audio_request_handle_standard_interface(USBDriver *usbp) {
    switch (g_request.request) {
        case USB_REQ_SET_INTERFACE:
            // Switch between operational and zero-bandwidth alternate modes.
            if (g_request.index == AUDIO_STREAMING_INTERFACE) {
                if (g_request.value == USB_DESC_INTERFACE_ALT_SETTING_OPERATIONAL) {
                    audio_playback_start_streaming(usbp);
                } else {
                    audio_playback_stop_streaming(usbp);
                }

                usbSetupTransfer(usbp, NULL, 0, NULL);
                return true;
            } else {
                return false;
            }
        default:
            return false;
    }
}

/**
 * @brief Handle class endpoint audio requests.
 *
 * @param usbp A pointer to the  USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
static bool audio_request_handle_class_endpoint(USBDriver *usbp) {
    uint8_t *p_data = (uint8_t *)g_request.data;

    uint8_t  control_selector = GET_BYTE(g_request.value, 1u);
    uint16_t endpoint_index   = g_request.index;

    (void)endpoint_index;

    if (control_selector != AUDIO_REQUEST_CS_SAMPLING_FREQ) {
        return false;
    }

    usbSetupTransfer(usbp, p_data, g_request.length, audio_request_update_sample_rate);

    return true;
}

/**
 * @brief Handle standard audio requests.
 *
 * @param usbp A pointer to the  USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
static bool audio_request_handle_standard(USBDriver *usbp) {
    switch (g_request.request_type & USB_RTYPE_RECIPIENT_MASK) {
        case USB_RTYPE_RECIPIENT_INTERFACE:
            return audio_request_handle_standard_interface(usbp);

        default:
            return false;
    }
}

/**
 * @brief Handle class audio requests.
 *
 * @param usbp A pointer to the  USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
static bool audio_request_handle_class(USBDriver *usbp) {
    switch (g_request.request_type & USB_RTYPE_RECIPIENT_MASK) {
        case USB_RTYPE_RECIPIENT_INTERFACE:
            return audio_request_handle_class_interface(usbp);

        case USB_RTYPE_RECIPIENT_ENDPOINT:
            return audio_request_handle_class_endpoint(usbp);

        default:
            return false;
    }
}

/**
 * @brief Handles setup requests.
 *
 * @param usbp A pointer to the  USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
bool audio_request_hook_cb(USBDriver *usbp) {
    g_request.request_type = usbp->setup[0];
    g_request.request      = usbp->setup[1];
    g_request.value        = (usbp->setup[3] << 8) | (usbp->setup[2]);
    g_request.index        = ((usbp->setup[5] << 8) | usbp->setup[4]);
    g_request.length       = ((usbp->setup[7] << 8) | usbp->setup[6]);

    switch (g_request.request_type & USB_RTYPE_TYPE_MASK) {
        case USB_RTYPE_TYPE_STD:
            return audio_request_handle_standard(usbp);

        case USB_RTYPE_TYPE_CLASS:
            return audio_request_handle_class(usbp);

        default:
            return false;
    }
}

/**
 * @brief Initialize the audio request module.
 */
void audio_request_init(mailbox_t *p_mailbox) {
    gp_mailbox = p_mailbox;

    g_controls.volume.channel_index = 0u;

    for (size_t channel_index = 0; channel_index < AUDIO_CHANNEL_COUNT; channel_index++) {
        g_controls.volume.b_channel_mute_states[channel_index]        = false;
        g_controls.volume.channel_volume_levels_8q8_db[channel_index] = 0;
    }

    g_controls.sample_rate_hz = AUDIO_DEFAULT_SAMPLE_RATE_HZ;
}

/**
 * @}
 */
