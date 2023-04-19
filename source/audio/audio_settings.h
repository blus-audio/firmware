// Copyright 2023 elagil

/**
 * @file
 * @brief   USB Audio settings.
 *
 * @addtogroup audio
 * @{
 */

#ifndef SOURCE_AUDIO_AUDIO_SETTINGS_H_
#define SOURCE_AUDIO_AUDIO_SETTINGS_H_

/**
 * @brief The audio sample rate in Hz.
 */
#define AUDIO_SAMPLE_RATE_HZ 48000u

/**
 * @brief The resolution of an audio sample in bits.
 */
#define AUDIO_RESOLUTION_BIT 32u

/**
 * @brief The number of complete audio packets to hold in the audio buffer.
 * @details Larger numbers allow more tolerance for changes in provided sample rate, but lead to more latency.
 */
#define AUDIO_BUFFER_PACKET_COUNT 5u

#endif  // SOURCE_AUDIO_AUDIO_SETTINGS_H_

/**
 * @}
 */
