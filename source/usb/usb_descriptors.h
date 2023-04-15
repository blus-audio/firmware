// Copyright 2023 elagil
#ifndef SOURCE_USB_USB_DESCRIPTORS_H_
#define SOURCE_USB_USB_DESCRIPTORS_H_

#include "audio.h"
#include "common.h"
#include "hal.h"

/**
 * @brief Maximum size for a packet on the in-endpoint.
 */
#define USB_DESC_MAX_IN_SIZE 4u

/**
 * @brief Packet interval for USB FS. Must be 1 ms.
 */
#define USB_DESC_FS_BINTERVAL 0x01u

/**
 * @brief The period between feedback packets in 2^N ms.
 */
#define USB_DESC_FEEDBACK_PERIOD 0x06u  // 2^6 ms = 64 ms

/**
 * @brief Terminal type: Loudspeaker.
 */
#define USB_DESC_TERMINAL_TYPE 0x0301u

static const uint8_t audio_device_descriptor_data[18] = {
    USB_DESC_DEVICE(0x0110u,  // bcdUSB (1.1).
                    0x00u,    // bDeviceClass (None).
                    0x00u,    // bDeviceSubClass.
                    0x00u,    // bDeviceProtocol.
                    0x40u,    // bMaxPacketSize.
                    0x1209u,  // idVendor.
                    0x0001u,  // idProduct.
                    0x0001u,  // bcdDevice.
                    1u,       // iManufacturer.
                    2u,       // iProduct.
                    3u,       // iSerialNumber.
                    1u)       // bNumConfigurations.
};

// Device Descriptor wrapper.
static const USBDescriptor audio_device_descriptor = {
    sizeof audio_device_descriptor_data, audio_device_descriptor_data};

// Configuration Descriptor tree for a UAC.
static const uint8_t audio_configuration_descriptor_data[122] = {
    // Configuration Descriptor. (UAC 4.2)
    USB_DESC_CONFIGURATION(122u,   // wTotalLength.
                           0x02u,  // bNumInterfaces.
                           0x01u,  // bConfigurationValue.
                           0u,     // iConfiguration.
                           0xC0u,  // bmAttributes (self powered).
                           50u),   // bMaxPower (100mA).
    //  Standard Audio Control Interface Descriptor (UAC 4.3.1)
    USB_DESC_INTERFACE(AUDIO_CONTROL_INTERFACE,  // bInterfaceNumber.
                       0x00u,                    // bAlternateSetting.
                       0x00u,                    // bNumEndpoints.
                       0x01u,                    // bInterfaceClass (AUDIO).
                       0x01u,  // bInterfaceSubClass (AUDIO_CONTROL).
                       0x00u,  // bInterfaceProtocol (none).
                       0u),    // iInterface.
    //  Class-specific AC Interface Descriptor (UAC 4.3.2)
    USB_DESC_BYTE(9u),      // bLength.
    USB_DESC_BYTE(0x24u),   // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x01u),   // bDescriptorSubtype (Header).
    USB_DESC_BCD(0x0100u),  // bcdADC.
    USB_DESC_WORD(43u),     // wTotalLength.
    USB_DESC_BYTE(0x01u),   // bInCollection (1 streaming interfaces).
    USB_DESC_BYTE(AUDIO_STREAMING_INTERFACE),  // baInterfaceNr.
    // Input Terminal Descriptor (UAC 4.3.2.1)
    USB_DESC_BYTE(12u),                  // bLength.
    USB_DESC_BYTE(0x24u),                // bDescriptorType.
    USB_DESC_BYTE(0x02u),                // bDescriptorSubtype (Input Terminal).
    USB_DESC_BYTE(AUDIO_INPUT_UNIT_ID),  // bTerminalID.
    USB_DESC_WORD(0x0101u),              // wTerminalType (USB streaming).
    USB_DESC_BYTE(0x00u),                // bAssocTerminal (none).
    USB_DESC_BYTE(2u),                   // bNrChannels (2).
    USB_DESC_WORD(0x0003u),              // wChannelConfig (left, right).
    USB_DESC_BYTE(0x00u),                // iChannelNames (none).
    USB_DESC_BYTE(0x00u),                // iTerminal (none).
    // Feature Unit Descriptor (UAC 4.3.2.5)
    USB_DESC_BYTE(13u),    // bLength.
    USB_DESC_BYTE(0x24u),  // bDescriptorType.
    USB_DESC_BYTE(0x06u),  // bDescriptorSubtype (Feature Unit).
    USB_DESC_BYTE(AUDIO_FUNCTION_UNIT_ID),  // bUnitID.
    USB_DESC_BYTE(AUDIO_INPUT_UNIT_ID),     // bSourceID.
    USB_DESC_BYTE(2u),                      // bControlSize (2).
    USB_DESC_WORD(0x0000u),                 // Master controls.
    USB_DESC_WORD(0x0003u),                 // Channel 0 controls
    USB_DESC_WORD(0x0003u),                 // Channel 1 controls
    USB_DESC_BYTE(0x00u),                   // iFeature (none)
    // Output Terminal Descriptor (UAC 4.3.2.2)
    USB_DESC_BYTE(9u),     // bLength.
    USB_DESC_BYTE(0x24u),  // bDescriptorType.
    USB_DESC_BYTE(0x03u),  // bDescriptorSubtype (Output Terminal).
    USB_DESC_BYTE(AUDIO_OUTPUT_UNIT_ID),    // bTerminalID.
    USB_DESC_WORD(USB_DESC_TERMINAL_TYPE),  // wTerminalType.
    USB_DESC_BYTE(0x00u),                   // bAssocTerminal (none).
    USB_DESC_BYTE(AUDIO_FUNCTION_UNIT_ID),  // bSourceID.
    USB_DESC_BYTE(0x00u),                   // iTerminal (none).
    // Standard AS Interface Descriptor (zero-bandwidth) (UAC 4.5.1)
    USB_DESC_INTERFACE(AUDIO_STREAMING_INTERFACE,  // bInterfaceNumber.
                       0x00u,                      // bAlternateSetting.
                       0x00u,                      // bNumEndpoints.
                       0x01u,                      // bInterfaceClass (AUDIO).
                       0x02u,  // bInterfaceSubClass (AUDIO_STREAMING).
                       0x00u,  // bInterfaceProtocol (none).
                       0u),    // iInterface.
    // Standard AS Interface Descriptor (operational) (UAC 4.5.1)
    USB_DESC_INTERFACE(AUDIO_STREAMING_INTERFACE,  // bInterfaceNumber.
                       0x01u,                      // bAlternateSetting.
                       0x02u,                      // bNumEndpoints.
                       0x01u,                      // bInterfaceClass (AUDIO).
                       0x02u,  // bInterfaceSubClass (AUDIO_STREAMING).
                       0x00u,  // bInterfaceProtocol (none).
                       0u),    // iInterface.
    // Class-specific AS Interface Descriptor (UAC 4.5.2)
    USB_DESC_BYTE(7u),                   // bLength.
    USB_DESC_BYTE(0x24u),                // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x01u),                // bDescriptorSubtype (General).
    USB_DESC_BYTE(AUDIO_INPUT_UNIT_ID),  // bTerminalLink.
    USB_DESC_BYTE(0x00u),                // bDelay (none).
    USB_DESC_WORD(0x0001u),              // wFormatTag (PCM format).
    // Class-Specific AS Format Type Descriptor (UAC 4.5.3)
    USB_DESC_BYTE(11u),                   // bLength.
    USB_DESC_BYTE(0x24u),                 // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x02u),                 // bDescriptorSubtype (Format).
    USB_DESC_BYTE(0x01u),                 // bFormatType (Type I).
    USB_DESC_BYTE(0x02u),                 // bNrChannels (2).
    USB_DESC_BYTE(0x02u),                 // bSubFrameSize (2).
    USB_DESC_BYTE(AUDIO_RESOLUTION_BIT),  // bBitResolution.
    USB_DESC_BYTE(0x01u),                 // bSamFreqType (Type I).
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_HZ, 0u)),
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_HZ, 1u)),
    USB_DESC_BYTE(GET_BYTE(AUDIO_SAMPLE_RATE_HZ, 2u)),
    // Standard AS Isochronous Audio Data Endpoint Descriptor (UAC 4.6.1.1)
    USB_DESC_BYTE(9u),                       // bLength (9).
    USB_DESC_BYTE(0x05u),                    // bDescriptorType (Endpoint).
    USB_DESC_BYTE(AUDIO_PLAYBACK_ENDPOINT),  // bEndpointAddress.
    USB_DESC_BYTE(0x05u),  // bmAttributes (asynchronous isochronous).
    USB_DESC_WORD(AUDIO_MAX_PACKET_SIZE),            // wMaxPacketSize
    USB_DESC_BYTE(0x01u),                            // bInterval (1 ms).
    USB_DESC_BYTE(0x00u),                            // bRefresh (0).
    USB_DESC_BYTE(AUDIO_FEEDBACK_ENDPOINT | 0x80u),  // bSynchAddress.
    // C-S AS Isochronous Audio Data Endpoint Descriptor (UAC 4.6.1.2)
    USB_DESC_BYTE(7u),       // bLength.
    USB_DESC_BYTE(0x25u),    // bDescriptorType (CS_ENDPOINT).
    USB_DESC_BYTE(0x01u),    // bDescriptorSubtype (General).
    USB_DESC_BYTE(0x00u),    // bmAttributes (none).
    USB_DESC_BYTE(0x02u),    // bLockDelayUnits (PCM Samples).
    USB_DESC_WORD(0x0000u),  // bLockDelay (0).
    // Standard Isochronous Audio Feedback Endpoint Descriptor
    USB_DESC_BYTE(9u),     // bLength (9).
    USB_DESC_BYTE(0x05u),  // bDescriptorType (Endpoint).
    USB_DESC_BYTE(AUDIO_FEEDBACK_ENDPOINT | 0x80u),  // bEndpointAddress.
    USB_DESC_BYTE(USB_EP_MODE_TYPE_ISOC),            // bmAttributes.
    USB_DESC_WORD(USB_DESC_MAX_IN_SIZE),             // wMaxPacketSize
    USB_DESC_BYTE(USB_DESC_FS_BINTERVAL),            // bInterval (1 ms).
    USB_DESC_BYTE(USB_DESC_FEEDBACK_PERIOD),         // bRefresh.
    USB_DESC_BYTE(0x00u),                            // bSynchAddress (none).
};

// Configuration Descriptor wrapper.
static const USBDescriptor audio_configuration_descriptor = {
    sizeof audio_configuration_descriptor_data,
    audio_configuration_descriptor_data};

// U.S. English language identifier.
static const uint8_t audio_string0[] = {
    USB_DESC_BYTE(4u),                     // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
    USB_DESC_WORD(0x0409u)                 // wLANGID (U.S. English).
};

// Vendor string.
static const uint8_t audio_string1[] = {
    USB_DESC_BYTE(14u),                    // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
    'e',
    0u,
    'l',
    0u,
    'a',
    0u,
    'g',
    0u,
    'i',
    0u,
    'l',
    0u};

// Device Description string.
static const uint8_t audio_string2[] = {
    USB_DESC_BYTE(20u),                    // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
    'b',
    0u,
    'l',
    0u,
    'u',
    0u,
    's',
    0u,
    ' ',
    0u,
    'm',
    0,
    'i',
    0u,
    'n',
    0u,
    'i',
    0u};

// Serial Number string.
static const uint8_t audio_string3[] = {
    USB_DESC_BYTE(8),                      // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  // bDescriptorType.
    '0' + CH_KERNEL_MAJOR,
    0u,
    '0' + CH_KERNEL_MINOR,
    0u,
    '0' + CH_KERNEL_PATCH,
    0u};

// Strings wrappers array.
static const USBDescriptor audio_strings[] = {
    {sizeof audio_string0, audio_string0},
    {sizeof audio_string1, audio_string1},
    {sizeof audio_string2, audio_string2},
    {sizeof audio_string3, audio_string3}};

#endif  // SOURCE_USB_USB_DESCRIPTORS_H_
