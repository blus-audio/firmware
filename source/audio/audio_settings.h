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
 * @note Currently supports 48 kHz and 96 kHz.
 */
#ifndef AUDIO_SAMPLE_RATE_HZ
#define AUDIO_SAMPLE_RATE_HZ 48000u
#endif

/**
 * @brief The resolution of an audio sample in bits.
 * @note Currently supports 16 and 32 bit.
 */
#ifndef AUDIO_RESOLUTION_BIT
#define AUDIO_RESOLUTION_BIT 32u
#endif

/**
 * @brief The number of complete audio packets to hold in the audio buffer.
 * @details Larger numbers allow more tolerance for changes in provided sample rate, but lead to more latency.
 */
#ifndef AUDIO_BUFFER_PACKET_COUNT
#define AUDIO_BUFFER_PACKET_COUNT 5u
#endif

/**
 * @brief The exponent of the period between feedback packets in 2^N ms.
 */
#ifndef AUDIO_FEEDBACK_PERIOD_EXPONENT
#define AUDIO_FEEDBACK_PERIOD_EXPONENT 0x03u
#endif

#endif  // SOURCE_AUDIO_AUDIO_SETTINGS_H_

/**
 * @}
 */
