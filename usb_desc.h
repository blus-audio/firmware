#ifndef _USB_DESC_H_
#define _USB_DESC_H_

#include "hal.h"
#include "audio.h"

/**
 * @brief Packet interval for USB FS.
 */
#define USB_DESC_FS_BINTERVAL 0x01 // 1 ms

/**
 * @brief The period between feedback packets in 2^N ms.
 */
#define USB_DESC_FEEDBACK_PERIOD 0x06 // 2^6 ms = 64 ms

/**
 * @brief An isochronous endpoint.
 */
#define USB_DESC_EP_TYPE_ISOC 0x01

/**
 * @brief Terminal type: Loudspeaker.
 */
#define USB_DESC_TERMINAL_TYPE 0x0301

// USB Device Descriptor (UAC 4.1)
static const uint8_t audio_device_descriptor_data[18] = {
    USB_DESC_DEVICE(0x0110, // bcdUSB (1.1).
                    0x00,   // bDeviceClass (None).
                    0x00,   // bDeviceSubClass.
                    0x00,   // bDeviceProtocol.
                    0x40,   // bMaxPacketSize.
                    0x0483, // idVendor (ST).
                    0x5740, // idProduct.
                    0x0001, // bcdDevice.
                    1,      // iManufacturer.
                    2,      // iProduct.
                    3,      // iSerialNumber.
                    1)      // bNumConfigurations.
};

// Device Descriptor wrapper.
static const USBDescriptor audio_device_descriptor = {
    sizeof audio_device_descriptor_data,
    audio_device_descriptor_data};

// Configuration Descriptor tree for a UAC.
static const uint8_t audio_configuration_descriptor_data[122] = {
    // Configuration Descriptor. (UAC 4.2)
    USB_DESC_CONFIGURATION(122,  // wTotalLength.
                           0x02, // bNumInterfaces.
                           0x01, // bConfigurationValue.
                           0,    // iConfiguration.
                           0xC0, // bmAttributes (self powered).
                           50),  // bMaxPower (100mA).
    //  Standard Audio Control Interface Descriptor (UAC 4.3.1)
    USB_DESC_INTERFACE(AUDIO_CONTROL_INTERFACE, // bInterfaceNumber.
                       0x00,                    // bAlternateSetting.
                       0x00,                    // bNumEndpoints.
                       0x01,                    // bInterfaceClass (AUDIO).
                       0x01,                    // bInterfaceSubClass (AUDIO_CONTROL).
                       0x00,                    // bInterfaceProtocol (none).
                       0),                      // iInterface.
    //  Class-specific AC Interface Descriptor (UAC 4.3.2)
    USB_DESC_BYTE(9),                         // bLength.
    USB_DESC_BYTE(0x24),                      // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x01),                      // bDescriptorSubtype (Header).
    USB_DESC_BCD(0x0100),                     // bcdADC.
    USB_DESC_WORD(43),                        // wTotalLength.
    USB_DESC_BYTE(0x01),                      // bInCollection (1 streaming interfaces).
    USB_DESC_BYTE(AUDIO_STREAMING_INTERFACE), // baInterfaceNr.
    // Input Terminal Descriptor (UAC 4.3.2.1)
    USB_DESC_BYTE(12),                  // bLength.
    USB_DESC_BYTE(0x24),                // bDescriptorType.
    USB_DESC_BYTE(0x02),                // bDescriptorSubtype (Input Terminal).
    USB_DESC_BYTE(AUDIO_INPUT_UNIT_ID), // bTerminalID.
    USB_DESC_WORD(0x0101),              // wTerminalType (USB streaming).
    USB_DESC_BYTE(0x00),                // bAssocTerminal (none).
    USB_DESC_BYTE(2),                   // bNrChannels (2).
    USB_DESC_WORD(0x0003),              // wChannelConfig (left, right).
    USB_DESC_BYTE(0x00),                // iChannelNames (none).
    USB_DESC_BYTE(0x00),                // iTerminal (none).
    // Feature Unit Descriptor (UAC 4.3.2.5)
    USB_DESC_BYTE(13),                     // bLength.
    USB_DESC_BYTE(0x24),                   // bDescriptorType.
    USB_DESC_BYTE(0x06),                   // bDescriptorSubtype (Feature Unit).
    USB_DESC_BYTE(AUDIO_FUNCTION_UNIT_ID), // bUnitID.
    USB_DESC_BYTE(AUDIO_INPUT_UNIT_ID),    // bSourceID.
    USB_DESC_BYTE(2),                      // bControlSize (2).
    USB_DESC_WORD(0x0000),                 // Master controls.
    USB_DESC_WORD(0x0003),                 // Channel 0 controls
    USB_DESC_WORD(0x0003),                 // Channel 1 controls
    USB_DESC_BYTE(0x00),                   // iFeature (none)
    // Output Terminal Descriptor (UAC 4.3.2.2)
    USB_DESC_BYTE(9),                      // bLength.
    USB_DESC_BYTE(0x24),                   // bDescriptorType.
    USB_DESC_BYTE(0x03),                   // bDescriptorSubtype (Output Terminal).
    USB_DESC_BYTE(AUDIO_OUTPUT_UNIT_ID),   // bTerminalID.
    USB_DESC_WORD(USB_DESC_TERMINAL_TYPE), // wTerminalType.
    USB_DESC_BYTE(0x00),                   // bAssocTerminal (none).
    USB_DESC_BYTE(AUDIO_FUNCTION_UNIT_ID), // bSourceID.
    USB_DESC_BYTE(0x00),                   // iTerminal (none).
    // Standard AS Interface Descriptor (empty) (UAC 4.5.1)
    USB_DESC_INTERFACE(AUDIO_STREAMING_INTERFACE, // bInterfaceNumber.
                       0x00,                      // bAlternateSetting.
                       0x00,                      // bNumEndpoints.
                       0x01,                      // bInterfaceClass (AUDIO).
                       0x02,                      // bInterfaceSubClass (AUDIO_STREAMING).
                       0x00,                      // bInterfaceProtocol (none).
                       0),                        // iInterface.
    // Standard AS Interface Descriptor (functional) (UAC 4.5.1)
    USB_DESC_INTERFACE(AUDIO_STREAMING_INTERFACE, // bInterfaceNumber.
                       0x01,                      // bAlternateSetting.
                       0x02,                      // bNumEndpoints.
                       0x01,                      // bInterfaceClass (AUDIO).
                       0x02,                      // bInterfaceSubClass (AUDIO_STREAMING).
                       0x00,                      // bInterfaceProtocol (none).
                       0),                        // iInterface.
    // Class-specific AS Interface Descriptor (UAC 4.5.2)
    USB_DESC_BYTE(7),                   // bLength.
    USB_DESC_BYTE(0x24),                // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x01),                // bDescriptorSubtype (General).
    USB_DESC_BYTE(AUDIO_INPUT_UNIT_ID), // bTerminalLink.
    USB_DESC_BYTE(0x00),                // bDelay (none).
    USB_DESC_WORD(0x0001),              // wFormatTag (PCM format).
    // Class-Specific AS Format Type Descriptor (UAC 4.5.3)
    USB_DESC_BYTE(11),                   // bLength.
    USB_DESC_BYTE(0x24),                 // bDescriptorType (CS_INTERFACE).
    USB_DESC_BYTE(0x02),                 // bDescriptorSubtype (Format).
    USB_DESC_BYTE(0x01),                 // bFormatType (Type I).
    USB_DESC_BYTE(0x02),                 // bNrChannels (2).
    USB_DESC_BYTE(0x02),                 // bSubFrameSize (2).
    USB_DESC_BYTE(AUDIO_RESOLUTION_BIT), // bBitResolution.
    USB_DESC_BYTE(0x01),                 // bSamFreqType (Type I).
    USB_DESC_BYTE(AUDIO_SAMPLE_RATE_HZ & 0xFF),
    USB_DESC_BYTE((AUDIO_SAMPLE_RATE_HZ >> 8) & 0xFF),
    USB_DESC_BYTE((AUDIO_SAMPLE_RATE_HZ >> 16) & 0xFF),
    // Standard AS Isochronous Audio Data Endpoint Descriptor (UAC 4.6.1.1)
    USB_DESC_BYTE(9),                              // bLength (9).
    USB_DESC_BYTE(0x05),                           // bDescriptorType (Endpoint).
    USB_DESC_BYTE(AUDIO_PLAYBACK_ENDPOINT),        // bEndpointAddress.
    USB_DESC_BYTE(0x05),                           // bmAttributes (asynchronous).
    USB_DESC_WORD(AUDIO_MAX_PACKET_SIZE),          // wMaxPacketSize
    USB_DESC_BYTE(0x01),                           // bInterval (1 ms).
    USB_DESC_BYTE(0x00),                           // bRefresh (0).
    USB_DESC_BYTE(AUDIO_FEEDBACK_ENDPOINT | 0x80), // bSynchAddress.
    // C-S AS Isochronous Audio Data Endpoint Descriptor (UAC 4.6.1.2)
    USB_DESC_BYTE(7),      // bLength.
    USB_DESC_BYTE(0x25),   // bDescriptorType (CS_ENDPOINT).
    USB_DESC_BYTE(0x01),   // bDescriptorSubtype (General).
    USB_DESC_BYTE(0x00),   // bmAttributes (none).
    USB_DESC_BYTE(0x02),   // bLockDelayUnits (PCM Samples).
    USB_DESC_WORD(0x0000), // bLockDelay (0).
    // Standard Isochronous Audio Feedback Endpoint Descriptor
    USB_DESC_BYTE(9),                              // bLength (9).
    USB_DESC_BYTE(0x05),                           // bDescriptorType (Endpoint).
    USB_DESC_BYTE(AUDIO_FEEDBACK_ENDPOINT | 0x80), // bEndpointAddress.
    USB_DESC_BYTE(USB_DESC_EP_TYPE_ISOC),          // bmAttributes.
    USB_DESC_WORD(AUDIO_FEEDBACK_BUFFER_SIZE),     // wMaxPacketSize
    USB_DESC_BYTE(USB_DESC_FS_BINTERVAL),          // bInterval (1 ms).
    USB_DESC_BYTE(USB_DESC_FEEDBACK_PERIOD),       // bRefresh.
    USB_DESC_BYTE(0x00),                           // bSynchAddress (none).
};

// Configuration Descriptor wrapper.
static const USBDescriptor audio_configuration_descriptor = {
    sizeof audio_configuration_descriptor_data,
    audio_configuration_descriptor_data};

// U.S. English language identifier.
static const uint8_t audio_string0[] = {
    USB_DESC_BYTE(4),                     // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), // bDescriptorType.
    USB_DESC_WORD(0x0409)                 // wLANGID (U.S. English).
};

// Vendor string.
static const uint8_t audio_string1[] = {
    USB_DESC_BYTE(14),                    // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), // bDescriptorType.
    'e', 0, 'l', 0, 'a', 0, 'g', 0, 'i', 0, 'l', 0};

// Device Description string.
static const uint8_t audio_string2[] = {
    USB_DESC_BYTE(10),                    // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), // bDescriptorType.
    'b',
    0,
    'l',
    0,
    'u',
    0,
    's',
    0,
};

// Serial Number string.
static const uint8_t audio_string3[] = {
    USB_DESC_BYTE(8),                     // bLength.
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), // bDescriptorType.
    '0' + CH_KERNEL_MAJOR, 0,
    '0' + CH_KERNEL_MINOR, 0,
    '0' + CH_KERNEL_PATCH, 0};

// Strings wrappers array.
static const USBDescriptor audio_strings[] = {
    {sizeof audio_string0, audio_string0},
    {sizeof audio_string1, audio_string1},
    {sizeof audio_string2, audio_string2},
    {sizeof audio_string3, audio_string3}};

#endif // _USB_DESC_H_
