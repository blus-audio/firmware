// Copyright 2023 elagil

/**
 * @file
 * @brief   USB Audio common helper macros and settings.
 *
 * @addtogroup audio
 * @{
 */

#ifndef SOURCE_AUDIO_AUDIO_COMMON_H_
#define SOURCE_AUDIO_AUDIO_COMMON_H_

#include <inttypes.h>
#include <stdbool.h>

#include "audio_settings.h"
#include "common.h"
#include "hal.h"

/**
 * @brief I2S driver shorthand
 */
#define I2S_DRIVER (I2SD3)

/**
 * @brief Commonly used audio messages.
 */
enum AUDIO_COMMON_MSG {
    AUDIO_COMMON_MSG_START_PLAYBACK,   ///< Start playback (I2S data output).
    AUDIO_COMMON_MSG_STOP_PLAYBACK,    ///< Stop playback (I2S data output).
    AUDIO_COMMON_MSG_SET_MUTE_STATE,   ///< Set new mute states.
    AUDIO_COMMON_MSG_SET_VOLUME,       ///< Set new volume levels.
    AUDIO_COMMON_MSG_SET_SAMPLE_RATE,  ///< Set a new sample rate.
    AUDIO_COMMON_MSG_RESET_VOLUME,     ///< Reset volume levels.
};

// Supported audio sample rates.
#define AUDIO_SAMPLE_RATE_48_KHZ     48000u
#define AUDIO_SAMPLE_RATE_96_KHZ     96000u
#define AUDIO_DEFAULT_SAMPLE_RATE_HZ AUDIO_SAMPLE_RATE_48_KHZ
#define AUDIO_MAX_SAMPLE_RATE_HZ     AUDIO_SAMPLE_RATE_96_KHZ

/**
 * @brief The number of audio channels.
 */
#define AUDIO_CHANNEL_COUNT 2u

/**
 * @brief The size of an audio sample in bytes.
 */
#define AUDIO_SAMPLE_SIZE AUDIO_COMMON_GET_SAMPLE_SIZE(AUDIO_RESOLUTION_BIT)

/**
 * @brief The maximum audio packet size to be received, in bytes.
 * @details Due to the feedback mechanism, a frame can be larger than a nominal packet. If the device
 * reports a too low sample rate, the host has to send a larger packet.
 * @note Reserve an extra number of full samples, e.g. eight.
 * @warning Do not choose a value that might cause the total amount of available RX FIFO buffer to be exceeded.
 */
#define AUDIO_MAX_PACKET_SIZE                                                                                          \
    AUDIO_COMMON_GET_PACKET_SIZE(AUDIO_CHANNEL_COUNT, AUDIO_MAX_SAMPLE_RATE_HZ, AUDIO_SAMPLE_SIZE) +                   \
        (8u * AUDIO_SAMPLE_SIZE)

/**
 * @brief The size of the audio buffer in bytes.
 */
#define AUDIO_MAX_BUFFER_SIZE AUDIO_COMMON_GET_BUFFER_SIZE(AUDIO_MAX_PACKET_SIZE, AUDIO_BUFFER_PACKET_COUNT)

// Endpoint numbers.
#define AUDIO_PLAYBACK_ENDPOINT 0x01u
#define AUDIO_FEEDBACK_ENDPOINT 0x02u

// Interface numbers.
#define AUDIO_CONTROL_INTERFACE   0u
#define AUDIO_STREAMING_INTERFACE 1u

// Functional unit numbers.
#define AUDIO_INPUT_UNIT_ID    1u
#define AUDIO_FUNCTION_UNIT_ID 2u
#define AUDIO_OUTPUT_UNIT_ID   3u
//
// Functional unit controls.
#define UAC_FU_MUTE_CONTROL   0x01u
#define UAC_FU_VOLUME_CONTROL 0x02u

// Endpoint controls.
#define UAC_EU_SAMPLING_FREQ_CONTROL 0x01u

// Definitions that are regularly reused.
#define AUDIO_BYTE_MASK 0xFFu

/**
 * @brief Get the sample size, from the resolution in bit.
 *
 * @param _resolution_bit The audio resolution in bit.
 */
#define AUDIO_COMMON_GET_SAMPLE_SIZE(_resolution_bit) ((_resolution_bit) / 8u)

/**
 * @brief Calculate the audio packet size.
 *
 * @param _channel_count The number of audio channels.
 * @param _sample_rate_hz The audio sample rate.
 * @param _sample_size The size of an audio sample.
 */
#define AUDIO_COMMON_GET_PACKET_SIZE(_channel_count, _sample_rate_hz, _sample_size)                                    \
    ((((_channel_count) * (_sample_rate_hz)) / 1000u) * (_sample_size))

/**
 * @brief Calculate the audio buffer size.
 *
 * @param _packet_count The number of packets that the buffer should hold.
 * @param _packet_size The size of each packet.
 */
#define AUDIO_COMMON_GET_BUFFER_SIZE(_packet_count, _packet_size) ((_packet_count) * (_packet_size))

/**
 * @brief The audio channel (left or right).
 */
enum AUDIO_COMMON_CHANNEL {
    AUDIO_COMMON_CHANNEL_LEFT   = 0x00u,  ///< The left audio channel.
    AUDIO_COMMON_CHANNEL_RIGHT  = 0x01u,  ///< The right audio channel.
    AUDIO_COMMON_CHANNEL_MASTER = 0xFFU,  ///< The audio master channel controls all channels.
};

#endif  // SOURCE_AUDIO_AUDIO_COMMON_H_

/**
 * @}
 */
