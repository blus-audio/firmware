#ifndef _AUDIO_H_
#define _AUDIO_H_

#include "hal.h"

// I2S driver shorthand.
#define I2S_DRIVER (I2SD3)

// Supported control requests from the USB Audio Class.
#define UAC_REQ_SET_CUR 0x01
#define UAC_REQ_SET_MIN 0x02
#define UAC_REQ_SET_MAX 0x03
#define UAC_REQ_SET_RES 0x04
#define UAC_REQ_GET_CUR 0x81
#define UAC_REQ_GET_MIN 0x82
#define UAC_REQ_GET_MAX 0x83
#define UAC_REQ_GET_RES 0x84

// Functional endpoints.
#define UAC_FU_MUTE_CONTROL 0x01
#define UAC_FU_VOLUME_CONTROL 0x02

// Audio-related events.
#define AUDIO_EVENT EVENT_MASK(0)
#define AUDIO_EVENT_PLAYBACK EVENT_MASK(1)
#define AUDIO_EVENT_MUTE EVENT_MASK(2)
#define AUDIO_EVENT_VOLUME EVENT_MASK(3)
#define AUDIO_EVENT_USB_STATE EVENT_MASK(4)

/**
 * @brief The audio sample rate in Hz.
 */
#define AUDIO_SAMPLE_RATE_HZ 48000u

/**
 * @brief The resolution of an audio sample in bits.
 */
#define AUDIO_RESOLUTION_BIT 16u

/**
 * @brief The number of audio channels, e.g. 2 for stereo.
 */
#define AUDIO_CHANNEL_COUNT 2u

/**
 * @brief The number of complete audio packets to hold in the audio buffer.
 * @details Larger numbers allow more tolerance for changes in provided sample rate,
 * but lead to more latency.
 */
#define AUDIO_BUFFER_PACKET_COUNT 3u

// The values below are calculated from those above, or constant. Changes should not be required.

/**
 * @brief The number of bits in a byte.
 */
#define AUDIO_BIT_PER_BYTE 8u

/**
 * @brief Number of audio samples that are transported per USB frame.
 */
#define AUDIO_SAMPLES_PER_FRAME (AUDIO_SAMPLE_RATE_HZ / 1000)

/**
 * @brief The size of each sample in bytes.
 */
#define AUDIO_SAMPLE_SIZE (AUDIO_RESOLUTION_BIT / AUDIO_BIT_PER_BYTE)

/**
 * @brief The size of an audio packet.
 * @details Calculated by means of the number of samples per frame, the channel count, and the size per audio sample.
 */
#define AUDIO_PACKET_SIZE (AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNEL_COUNT * AUDIO_SAMPLE_SIZE)

/**
 * @brief The maximum audio packet size to be received.
 * @details Due to the feedback mechanism, a frame can be larger than the regular \a AUDIO_PACKET_SIZE.
 * If the device reports a too low sample rate, the host has to send a larger packet.
 */
#define AUDIO_MAX_PACKET_SIZE (2u * AUDIO_PACKET_SIZE)

/**
 * @brief The number of samples in the audio buffer.
 */
#define AUDIO_BUFFER_SAMPLE_COUNT (AUDIO_PACKET_SIZE * AUDIO_BUFFER_PACKET_COUNT)

/**
 * @brief The size of the packets for the feedback endpoint.
 * @details These are three bytes long, in 10.14 binary format.
 * The format represents a number in kHz, so that a 1 at a bit shift of 14 is 1 kHz.
 */
#define AUDIO_FEEDBACK_BUFFER_SIZE 3u

/**
 * @brief The target buffer fill level in number of samples.
 */
#define AUDIO_BUFFER_TARGET_FILL_LEVEL (AUDIO_BUFFER_SAMPLE_COUNT / 2u)

/**
 * @brief The allowed margin for the buffer fill level in samples.
 * @details If the actual fill level is closer to zero or the end of the buffer than specified by the value,
 * this application attempts to force the host to adjust fill level by means of changing the reported feedback value.
 *
 * @note This should never happen, if the host adheres to the provided feedback, and does not
 * drop packets, or sends excessive amounts of data.
 */
#define AUDIO_BUFFER_FILL_LEVEL_MARGIN (AUDIO_PACKET_SIZE)

/**
 * @brief The lower boundary for the buffer fill level in samples.
 */
#define AUDIO_BUFFER_MIN_FILL_LEVEL (AUDIO_BUFFER_FILL_LEVEL_MARGIN)

/**
 * @brief The upper boundary for the buffer fill level in samples.
 */
#define AUDIO_BUFFER_MAX_FILL_LEVEL (AUDIO_BUFFER_SAMPLE_COUNT - AUDIO_BUFFER_FILL_LEVEL_MARGIN)

/**
 * @brief The amount by which the feedback value is adjusted, when the buffer fill level is critical.
 *
 * @details This translates to a difference in reported sample rate of
 *   \a AUDIO_FEEDBACK_CORRECTION_OFFSET * 2**14 / 1000
 *
 * For example, 16 represents a reported offset of 1.024 Hz - a mild adjustment.
 */
#define AUDIO_FEEDBACK_CORRECTION_OFFSET (16u)

// Sanity checks.

#if AUDIO_MAX_PACKET_SIZE <= AUDIO_PACKET_SIZE
#error "The maximum audio packet size should be larger than the regular packet size."
#endif

#if AUDIO_BUFFER_MIN_FILL_LEVEL >= AUDIO_BUFFER_MAX_FILL_LEVEL
#error "Inconsistent settings, sample count tolerance likely too large."
#endif

// Endpoint numbers.
#define AUDIO_PLAYBACK_ENDPOINT 0x01
#define AUDIO_FEEDBACK_ENDPOINT 0x02

// Interface numbers.
#define AUDIO_CONTROL_INTERFACE 0
#define AUDIO_STREAMING_INTERFACE 1

// Functional unit numbers.
#define AUDIO_INPUT_UNIT_ID 1
#define AUDIO_FUNCTION_UNIT_ID 2
#define AUDIO_OUTPUT_UNIT_ID 3

/**
 * @brief A structure that holds the audio configuration - driver pointers.
 */
struct audio_config
{
  const I2SConfig *p_i2s_config; ///< Pointer to the I2S configuration structure.
};

/**
 * @brief A structure that holds the state of the audio sample rate feedback.
 *
 */
struct audio_feedback
{
  volatile bool b_is_first_sof;               ///< If true, the first SOF packet is yet to be received.
  volatile bool b_is_valid;                   ///< Is true, if the feedback value is valid.
  volatile size_t sof_package_count;          ///< Counts the SOF packages since the last feedback value update.
  volatile uint32_t value;                    ///< The current feedback value.
  uint8_t buffer[AUDIO_FEEDBACK_BUFFER_SIZE]; ///< The current feedback buffer, derived from the feedback value.
  volatile uint32_t previous_counter_value;   ///< The counter value at the time of the previous SOF interrupt.
  volatile uint32_t timer_count_difference;   ///< The accumulated timer count duration over all SOF interrupts.
};

/**
 * @brief A structure that holds the state of audio playback, as well as the audio buffer.
 */
struct audio_playback
{
  uint16_t buffer[AUDIO_BUFFER_SAMPLE_COUNT + AUDIO_MAX_PACKET_SIZE / AUDIO_SAMPLE_SIZE]; ///< The audio sample buffer.
  uint16_t buffer_write_offset;                                                           ///< The current write offset (USB).
  bool b_enabled;                                                                         ///< True, if audio playback is enabled, and data is being received via USB.
  bool b_output_enabled;                                                                  ///< True, if the audio output is enabled, and data is being output via I2S.
};

/**
 * @brief A structure that holds all relevant information for audio control, such as volume and mute.
 */
struct audio_control
{
  uint8_t buffer[8];                                         ///< The provided control data.
  uint8_t channel;                                           ///< The current channel mask.
  bool b_channel_mute_states[AUDIO_CHANNEL_COUNT];           ///< Channel mute states.
  int16_t channel_volume_levels_8q8_db[AUDIO_CHANNEL_COUNT]; ///< Channel volumes in 8.8 format (in dB).
  int16_t local_volume_8q8_db;                               ///< The locally set volume (volume potentiometer) in 8.8 format (in dB).
};

/**
 * @brief Diagnostics for observing important properties of the audio module's status.
 */
struct audio_diagnostics
{
  uint16_t sample_distance; ///< The distance between read (I2S) and write (USB) memory locations, in units of audio samples.
  size_t error_count;       ///< The number of buffer over-/underflow errors.
};

/**
 * @brief The main audio context.
 * @details Holds all important audio-related structures.
 */
struct audio_context
{
  struct audio_config config;           ///< The audio configuration structure.
  struct audio_feedback feedback;       ///< The audio feedback structure.
  struct audio_playback playback;       ///< The audio playback structure.
  struct audio_control control;         ///< The audio control structure.
  struct audio_diagnostics diagnostics; ///< The audio diagnostics structure.

  event_source_t audio_events; ///< The event source for all audio-related events.
};

/**
 * @brief Toggle the playback state.
 *
 * @param p_playback The pointer to the playback structure.
 */
__STATIC_INLINE void audio_toggle_playback(struct audio_playback *p_playback)
{
  p_playback->b_enabled = !p_playback->b_enabled;
}

struct audio_context *audio_get_context(void);

void audio_init_context(struct audio_context *p_context);
void audio_init_feedback(struct audio_feedback *p_feedback);

void audio_stop_playback_cb(USBDriver *usbp);
bool audio_requests_hook_cb(USBDriver *usbp);
void audio_received_cb(USBDriver *usbp, usbep_t ep);
void audio_feedback_cb(USBDriver *usbp, usbep_t ep);
void audio_start_sof_capture(void);
void audio_stop_sof_capture(void);

#endif // _AUDIO_H_
