// Copyright 2023 elagil

/**
 * @file
 * @brief   USB descriptors for UAC v1 functionality.
 *
 * @addtogroup usb
 * @{
 */

#ifndef SOURCE_USB_USB_DESCRIPTORS_H_
#define SOURCE_USB_USB_DESCRIPTORS_H_

#include "audio.h"
#include "common.h"
#include "hal.h"

/**
 * @brief The current version of the USB specification (1.1).
 */
#define USB_DESC_SPEC_VERSION 0x0110u

/**
 * @brief The current version of the ADC specification (1.0).
 */
#define USB_DESC_ADC_VERSION 0x0100u

/**
 * @brief The current version of the USB device (1.0).
 */
#define USB_DESC_DEVICE_VERSION 0x0100u

/**
 * @brief The pidcodes vendor ID.
 */
#define USB_DESC_VID 0x1209u

/**
 * @brief The pidcodes product ID.
 */
#define USB_DESC_PID 0xAF01u

/**
 * @brief Maximum size for a packet on the in-endpoint.
 */
#define USB_DESC_MAX_IN_SIZE 4u

/**
 * @brief Packet interval for USB FS. Must be 1 ms.
 */
#define USB_DESC_FS_BINTERVAL 0x01u

#define USB_DESC_ENDPOINT_COUNT_ZERO_BANDWIDTH 0u
#define USB_DESC_ENDPOINT_COUNT_OPERATIONAL    2u

/**
 * @brief Audio endpoint assignments.
 * @note These values are not defined by the standard, but arbitrary. In some cases, there are hardware limitations with
 * regard to endpoint numbers.
 */
enum USB_DESC_ENDPOINT {
    USB_DESC_ENDPOINT_PLAYBACK = 0x01u,  ///< The endpoint for audio playback.
    USB_DESC_ENDPOINT_FEEDBACK = 0x02u,  ///< The endpoint for audio feedback.
};

/**
 * @brief Audio unit assignments.
 * @note These values are not defined by the standard, but arbitrary.
 */
enum USB_DESC_UNIT {
    USB_DESC_UNIT_INPUT    = 0x01u,  ///< The index of the input unit.
    USB_DESC_UNIT_FUNCTION = 0x02u,  ///< The index of the function unit.
    USB_DESC_UNIT_OUTPUT   = 0x03u,  ///< The index of the output unit.
};

/**
 * @brief Audio interface assignments.
 * @note These values are not defined by the standard, but arbitrary.
 */
enum USB_DESC_INTERFACE {
    USB_DESC_INTERFACE_CONTROL   = 0x00u,  ///< The index of the control interface.
    USB_DESC_INTERFACE_STREAMING = 0x01u,  ///< The index of the streaming interface.
};

/**
 * @brief USB terminal types.
 */
enum USB_DESC_TERMINAL_TYPE {
    USB_DESC_TERMINAL_TYPE_INPUT     = 0x02u,    ///< An input terminal.
    USB_DESC_TERMINAL_TYPE_OUTPUT    = 0x03u,    ///< An output terminal.
    USB_DESC_TERMINAL_TYPE_UNDEFINED = 0x0100u,  ///< An undefined terminal type.
    USB_DESC_TERMINAL_TYPE_STREAMING = 0x0101u,  ///< A Terminal dealing with a signal carried over an endpoint in an
                                                 ///< AudioStreaming interface. The AudioStreaming interface descriptor
                                                 ///< points to the associated Terminal through the bTerminalLink field.
    USB_DESC_TERMINAL_TYPE_VENDOR_SPECIFIC =
        0x01FFu,  ///< A Terminal dealing with a signal carried over a vendor-specific interface. The vendor-specific
                  ///< interface descriptor must contain a field that references the Terminal.
};

/**
 * @brief USB output terminal types.
 */
enum USB_DESC_OUTPUT_TERMINAL_TYPE {
    USB_DESC_OUTPUT_TERMINAL_TYPE_SPEAKER                    = 0x0301u,  ///< A generic speaker.
    USB_DESC_OUTPUT_TERMINAL_TYPE_HEADPHONES                 = 0x0302u,  ///< Head mounted audio output.
    USB_DESC_OUTPUT_TERMINAL_TYPE_HEAD_MOUNTED_DISPLAY_AUDIO = 0x0303u,  ///< Part of a VR head mount.
    USB_DESC_OUTPUT_TERMINAL_TYPE_DESKTOP_SPEAKER            = 0x0304u,  ///< Small desk speaker.
    USB_DESC_OUTPUT_TERMINAL_TYPE_ROOM_SPEAKER               = 0x0305u,  ///< Larger room speaker.
    USB_DESC_OUTPUT_TERMINAL_TYPE_COMMUNICATION_SPEAKER      = 0x0306u,  ///< Speaker for voice communication.
    USB_DESC_OUTPUT_TERMINAL_TYPE_LOW_FREQUENCY_SPEAKER      = 0x0307u   ///< Speaker for low frequency effects.
};

/**
 * @brief Channel configuration bit masks.
 */
enum USB_DESC_CHANNEL_CONFIG {
    USB_DESC_CHANNEL_CONFIG_NONE            = 0x0000u,
    USB_DESC_CHANNEL_CONFIG_LEFT_FRONT      = 0x0001u,
    USB_DESC_CHANNEL_CONFIG_RIGHT_FRONT     = 0x0002u,
    USB_DESC_CHANNEL_CONFIG_CENTER_FRONT    = 0x0004u,
    USB_DESC_CHANNEL_CONFIG_LFE             = 0x0008u,
    USB_DESC_CHANNEL_CONFIG_LEFT_SURROUND   = 0x0010u,
    USB_DESC_CHANNEL_CONFIG_RIGHT_SURROUND  = 0x0020u,
    USB_DESC_CHANNEL_CONFIG_LEFT_OF_CENTER  = 0x0040u,
    USB_DESC_CHANNEL_CONFIG_RIGHT_OF_CENTER = 0x0080u,
    USB_DESC_CHANNEL_CONFIG_SURROUND        = 0x0100u,
    USB_DESC_CHANNEL_CONFIG_SIDE_LEFT       = 0x0200u,
    USB_DESC_CHANNEL_CONFIG_SIDE_RIGHT      = 0x0400u,
    USB_DESC_CHANNEL_CONFIG_TOP             = 0x0800u,
};

/**
 * @brief Feature unit control bit masks.
 */
enum USB_DESC_FU_CONTROLS {
    USB_DESC_FU_CONTROLS_NONE              = 0X0000U,
    USB_DESC_FU_CONTROLS_MUTE              = 0X0001U,
    USB_DESC_FU_CONTROLS_VOLUME            = 0X0002u,
    USB_DESC_FU_CONTROLS_BASS              = 0X0004U,
    USB_DESC_FU_CONTROLS_MID               = 0X0008U,
    USB_DESC_FU_CONTROLS_TREBLE            = 0X0010u,
    USB_DESC_FU_CONTROLS_GRAPHIC_EQUALIZER = 0x0020u,
    USB_DESC_FU_CONTROLS_AUTOMATIC_GAIN    = 0x0040u,
    USB_DESC_FU_CONTROLS_DELAY             = 0x0080u,
    USB_DESC_FU_CONTROLS_BASS_BOOST        = 0x0100u,
    USB_DESC_FU_CONTROLS_LOUDNESS          = 0x0200u,
};

// USB interface definitions.
#define USB_DESC_INTERFACE_CLASS_AUDIO                    0x01u
#define USB_DESC_INTERFACE_CLASS_AUDIO_SUBCLASS_CONTROL   0x01u
#define USB_DESC_INTERFACE_CLASS_AUDIO_SUBCLASS_STREAMING 0x02u

#define USB_DESC_INTERFACE_ALT_SETTING_ZERO_BW     0x00u
#define USB_DESC_INTERFACE_ALT_SETTING_OPERATIONAL 0x01u

#define USB_DESC_INTERFACE_NONE               0x00u
#define USB_DESC_INTERFACE_PROTOCOL_UNDEFINED 0x00u

static const uint8_t audio_device_descriptor_data[18u] = {
    USB_DESC_DEVICE(USB_DESC_SPEC_VERSION,    // bcdUSB (1.1).
                    0x00u,                    // bDeviceClass (None).
                    0x00u,                    // bDeviceSubClass.
                    0x00u,                    // bDeviceProtocol.
                    0x40u,                    // bMaxPacketSize.
                    USB_DESC_VID,             // idVendor.
                    USB_DESC_PID,             // idProduct.
                    USB_DESC_DEVICE_VERSION,  // bcdDevice.
                    1u,                       // iManufacturer.
                    2u,                       // iProduct.
                    3u,                       // iSerialNumber.
                    1u)                       // bNumConfigurations.
};

// Device Descriptor wrapper.
static const USBDescriptor audio_device_descriptor = {sizeof audio_device_descriptor_data,
                                                      audio_device_descriptor_data};

#define USB_DESCRIPTORS_TOTAL_LENGTH 125u

// Configuration Descriptor tree for a UAC.
static const uint8_t audio_configuration_descriptor_data[USB_DESCRIPTORS_TOTAL_LENGTH] = {
    // Configuration Descriptor. (UAC 4.2)
    USB_DESC_CONFIGURATION(USB_DESCRIPTORS_TOTAL_LENGTH,  // wTotalLength.
                           0x02u,                         // bNumInterfaces.
                           0x01u,                         // bConfigurationValue.
                           0u,                            // iConfiguration.
                           0xC0u,                         // bmAttributes (self powered).
                           50u),                          // bMaxPower (100mA).

    // Standard Audio Control Interface Descriptor (UAC 4.3.1)
    USB_DESC_INTERFACE(USB_DESC_INTERFACE_CONTROL,                       // bInterfaceNumber.
                       0x00u,                                            // bAlternateSetting.
                       0x00u,                                            // bNumEndpoints.
                       USB_DESC_INTERFACE_CLASS_AUDIO,                   // bInterfaceClass.
                       USB_DESC_INTERFACE_CLASS_AUDIO_SUBCLASS_CONTROL,  // bInterfaceSubClass.
                       0x00u,                                            // bInterfaceProtocol (none).
                       0u),                                              // iInterface.

    // Class-specific AC Interface Descriptor (UAC 4.3.2)
    USB_DESC_BYTE(9u),                            // bLength.
    USB_DESC_BYTE(0x24u),                         // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x01u),                         // bDescriptorSubtype (Header).
    USB_DESC_BCD(USB_DESC_ADC_VERSION),           // bcdADC.
    USB_DESC_WORD(43u),                           // wTotalLength.
    USB_DESC_BYTE(0x01u),                         // bInCollection (1 streaming interface).
    USB_DESC_BYTE(USB_DESC_INTERFACE_STREAMING),  // baInterfaceNr.

    // Input Terminal Descriptor (UAC 4.3.2.1)
    USB_DESC_BYTE(12u),                               // bLength.
    USB_DESC_BYTE(0x24u),                             // bDescriptorType.
    USB_DESC_BYTE(USB_DESC_TERMINAL_TYPE_INPUT),      // bDescriptorSubtype.
    USB_DESC_BYTE(USB_DESC_UNIT_INPUT),               // bTerminalID.
    USB_DESC_WORD(USB_DESC_TERMINAL_TYPE_STREAMING),  // wTerminalType.
    USB_DESC_BYTE(0x00u),                             // bAssocTerminal (none).
    USB_DESC_BYTE(AUDIO_CHANNEL_COUNT),               // bNrChannels.
    USB_DESC_WORD(USB_DESC_CHANNEL_CONFIG_RIGHT_FRONT |
                  USB_DESC_CHANNEL_CONFIG_LEFT_FRONT),  // wChannelConfig (left, right).
    USB_DESC_BYTE(0x00u),                               // iChannelNames (none).
    USB_DESC_BYTE(0x00u),                               // iTerminal (none).

    // Feature Unit Descriptor (UAC 4.3.2.5)
    USB_DESC_BYTE(13u),                                                      // bLength.
    USB_DESC_BYTE(0x24u),                                                    // bDescriptorType.
    USB_DESC_BYTE(0x06u),                                                    // bDescriptorSubtype (Feature Unit).
    USB_DESC_BYTE(USB_DESC_UNIT_FUNCTION),                                   // bUnitID.
    USB_DESC_BYTE(USB_DESC_UNIT_INPUT),                                      // bSourceID.
    USB_DESC_BYTE(AUDIO_CHANNEL_COUNT),                                      // bControlSize.
    USB_DESC_WORD(USB_DESC_FU_CONTROLS_NONE),                                // Master controls.
    USB_DESC_WORD(USB_DESC_FU_CONTROLS_MUTE | USB_DESC_FU_CONTROLS_VOLUME),  // Channel 0 controls
    USB_DESC_WORD(USB_DESC_FU_CONTROLS_MUTE | USB_DESC_FU_CONTROLS_VOLUME),  // Channel 1 controls
    USB_DESC_BYTE(0x00u),                                                    // iFeature (none)

    // Output Terminal Descriptor (UAC 4.3.2.2)
    USB_DESC_BYTE(9u),                                     // bLength.
    USB_DESC_BYTE(0x24u),                                  // bDescriptorType.
    USB_DESC_BYTE(USB_DESC_TERMINAL_TYPE_OUTPUT),          // bDescriptorSubtype.
    USB_DESC_BYTE(USB_DESC_UNIT_OUTPUT),                   // bTerminalID.
    USB_DESC_WORD(USB_DESC_OUTPUT_TERMINAL_TYPE_SPEAKER),  // wTerminalType.
    USB_DESC_BYTE(0x00u),                                  // bAssocTerminal (none).
    USB_DESC_BYTE(USB_DESC_UNIT_FUNCTION),                 // bSourceID.
    USB_DESC_BYTE(0x00u),                                  // iTerminal (none).

    // Standard AS Interface Descriptor (zero-bandwidth) (UAC 4.5.1)
    USB_DESC_INTERFACE(USB_DESC_INTERFACE_STREAMING,                       // bInterfaceNumber.
                       USB_DESC_INTERFACE_ALT_SETTING_ZERO_BW,             // bAlternateSetting.
                       USB_DESC_ENDPOINT_COUNT_ZERO_BANDWIDTH,             // bNumEndpoints.
                       USB_DESC_INTERFACE_CLASS_AUDIO,                     // bInterfaceClass.
                       USB_DESC_INTERFACE_CLASS_AUDIO_SUBCLASS_STREAMING,  // bInterfaceSubClass.
                       USB_DESC_INTERFACE_PROTOCOL_UNDEFINED,              // bInterfaceProtocol.
                       USB_DESC_INTERFACE_NONE),                           // iInterface.

    // Standard AS Interface Descriptor (operational) (UAC 4.5.1)
    USB_DESC_INTERFACE(USB_DESC_INTERFACE_STREAMING,                       // bInterfaceNumber.
                       USB_DESC_INTERFACE_ALT_SETTING_OPERATIONAL,         // bAlternateSetting.
                       USB_DESC_ENDPOINT_COUNT_OPERATIONAL,                // bNumEndpoints.
                       USB_DESC_INTERFACE_CLASS_AUDIO,                     // bInterfaceClass.
                       USB_DESC_INTERFACE_CLASS_AUDIO_SUBCLASS_STREAMING,  // bInterfaceSubClass.
                       USB_DESC_INTERFACE_PROTOCOL_UNDEFINED,              // bInterfaceProtocol.
                       USB_DESC_INTERFACE_NONE),                           // iInterface.

    // Class-specific AS Interface Descriptor (UAC 4.5.2)
    USB_DESC_BYTE(7u),                   // bLength.
    USB_DESC_BYTE(0x24u),                // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x01u),                // bDescriptorSubtype (general).
    USB_DESC_BYTE(USB_DESC_UNIT_INPUT),  // bTerminalLink.
    USB_DESC_BYTE(0x00u),                // bDelay (none).
    USB_DESC_WORD(0x0001u),              // wFormatTag (PCM format).

    // Class-Specific AS Format Type Descriptor (UAC 4.5.3)
    USB_DESC_BYTE(14u),                                     // bLength.
    USB_DESC_BYTE(0x24u),                                   // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x02u),                                   // bDescriptorSubtype (Format).
    USB_DESC_BYTE(0x02u),                                   // bFormatType (Type I).
    USB_DESC_BYTE(AUDIO_CHANNEL_COUNT),                     // bNrChannels.
    USB_DESC_BYTE(AUDIO_SAMPLE_SIZE),                       // bSubframeSize.
    USB_DESC_BYTE(AUDIO_RESOLUTION_BIT),                    // bBitResolution.
    USB_DESC_BYTE(0x02u),                                   // bSamFreqType (Type I).
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_48_KHZ, 0u)),  // Audio sampling frequency, byte 0.
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_48_KHZ, 1u)),  // Audio sampling frequency, byte 1.
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_48_KHZ, 2u)),  // Audio sampling frequency, byte 2.
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_96_KHZ, 0u)),  // Audio sampling frequency, byte 0.
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_96_KHZ, 1u)),  // Audio sampling frequency, byte 1.
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_96_KHZ, 2u)),  // Audio sampling frequency, byte 2.

    // Standard AS Isochronous Audio Data Endpoint Descriptor (UAC 4.6.1.1)
    USB_DESC_BYTE(9u),                                  // bLength (9).
    USB_DESC_BYTE(0x05u),                               // bDescriptorType (Endpoint).
    USB_DESC_BYTE(USB_DESC_ENDPOINT_PLAYBACK),          // bEndpointAddress.
    USB_DESC_BYTE(0x05u),                               // bmAttributes (asynchronous isochronous).
    USB_DESC_WORD(AUDIO_MAX_PACKET_SIZE),               // wMaxPacketSize
    USB_DESC_BYTE(USB_DESC_FS_BINTERVAL),               // bInterval.
    USB_DESC_BYTE(0x00u),                               // bRefresh (0).
    USB_DESC_BYTE(USB_DESC_ENDPOINT_FEEDBACK | 0x80u),  // bSynchAddress.

    // C-S AS Isochronous Audio Data Endpoint Descriptor (UAC 4.6.1.2)
    USB_DESC_BYTE(7u),       // bLength.
    USB_DESC_BYTE(0x25u),    // bDescriptorType (CS_ENDPOINT).
    USB_DESC_BYTE(0x01u),    // bDescriptorSubtype (General).
    USB_DESC_BYTE(0x01u),    // bmAttributes - support sampling frequency adjustment.
    USB_DESC_BYTE(0x02u),    // bLockDelayUnits (PCM Samples).
    USB_DESC_WORD(0x0000u),  // bLockDelay (0).

    // Standard Isochronous Audio Feedback Endpoint Descriptor
    USB_DESC_BYTE(9u),                                  // bLength (9).
    USB_DESC_BYTE(0x05u),                               // bDescriptorType (Endpoint).
    USB_DESC_BYTE(USB_DESC_ENDPOINT_FEEDBACK | 0x80u),  // bEndpointAddress.
    USB_DESC_BYTE(USB_EP_MODE_TYPE_ISOC),               // bmAttributes.
    USB_DESC_WORD(USB_DESC_MAX_IN_SIZE),                // wMaxPacketSize
    USB_DESC_BYTE(USB_DESC_FS_BINTERVAL),               // bInterval (1 ms).
    USB_DESC_BYTE(AUDIO_FEEDBACK_PERIOD_EXPONENT),      // bRefresh.
    USB_DESC_BYTE(0x00u),                               // bSynchAddress (none).
};

// Configuration Descriptor wrapper.
static const USBDescriptor audio_configuration_descriptor = {sizeof audio_configuration_descriptor_data,
                                                             audio_configuration_descriptor_data};

// U.S. English language identifier.
static const uint8_t audio_string0[] = {
    USB_DESC_BYTE(4u),                     // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
    USB_DESC_WORD(0x0409u)                 // wLANGID (U.S. English).
};

// Vendor string.
static const uint8_t audio_string1[] = {USB_DESC_BYTE(22u),                    // bLength.
                                        USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
                                        'b',
                                        0u,
                                        'l',
                                        0u,
                                        'u',
                                        0u,
                                        's',
                                        0u,
                                        '-',
                                        0u,
                                        'a',
                                        0u,
                                        'u',
                                        0u,
                                        'd',
                                        0u,
                                        'i',
                                        0u,
                                        'o',
                                        0u};

// Device Description string.
static const uint8_t audio_string2[] = {USB_DESC_BYTE(30u),                    // bLength.
                                        USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
                                        'U',
                                        0u,
                                        'S',
                                        0u,
                                        'B',
                                        0u,
                                        '-',
                                        0u,
                                        'I',
                                        0u,
                                        '2',
                                        0,
                                        'S',
                                        0u,
                                        ' ',
                                        0u,
                                        'b',
                                        0u,
                                        'r',
                                        0u,
                                        'i',
                                        0u,
                                        'd',
                                        0u,
                                        'g',
                                        0u,
                                        'e',
                                        0u};

// Serial Number string.
static const uint8_t audio_string3[] = {USB_DESC_BYTE(8),                      // bLength.
                                        USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
                                        '0' + CH_KERNEL_MAJOR,
                                        0u,
                                        '0' + CH_KERNEL_MINOR,
                                        0u,
                                        '0' + CH_KERNEL_PATCH,
                                        0u};

// Strings wrappers array.
static const USBDescriptor audio_strings[] = {{sizeof audio_string0, audio_string0},
                                              {sizeof audio_string1, audio_string1},
                                              {sizeof audio_string2, audio_string2},
                                              {sizeof audio_string3, audio_string3}};

#endif  // SOURCE_USB_USB_DESCRIPTORS_H_

/**
 * @}
 */
