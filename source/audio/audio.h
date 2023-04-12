// Copyright 2023 elagil
#ifndef SOURCE_AUDIO_AUDIO_H_
#define SOURCE_AUDIO_AUDIO_H_

#include "hal.h"

/**
 * @brief I2S driver shorthand
 */
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
#define UAC_FU_MUTE_CONTROL   0x01
#define UAC_FU_VOLUME_CONTROL 0x02

/**
 * @brief Audio related events.
 *
 */
#define AUDIO_EVENT EVENT_MASK(0)

/**
 * @brief Start streaming audio data from USB.
 */
#define AUDIO_EVENT_START_STREAMING EVENT_MASK(1)

/**
 * @brief Stop streaming audio data from USB.
 */
#define AUDIO_EVENT_STOP_STREAMING EVENT_MASK(2)

/**
 * @brief Start I2S audio blayback.
 */
#define AUDIO_EVENT_START_PLAYBACK EVENT_MASK(3)

/**
 * @brief Stop I2S audio blayback.
 */
#define AUDIO_EVENT_STOP_PLAYBACK EVENT_MASK(4)

/**
 * @brief Mute state changed.
 */
#define AUDIO_EVENT_MUTE EVENT_MASK(5)

/**
 * @brief Volume setting changed.
 */
#define AUDIO_EVENT_VOLUME EVENT_MASK(6)

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
 * @details Larger numbers allow more tolerance for changes in provided sample
 * rate, but lead to more latency.
 */
#define AUDIO_BUFFER_PACKET_COUNT 3u

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
 * @details Calculated by means of the number of samples per frame, the channel
 * count, and the size per audio sample.
 */
#define AUDIO_PACKET_SIZE                                                      \
    (AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNEL_COUNT * AUDIO_SAMPLE_SIZE)

/**
 * @brief The maximum audio packet size to be received.
 * @details Due to the feedback mechanism, a frame can be larger than the
 * regular \a AUDIO_PACKET_SIZE. If the device reports a too low sample rate,
 * the host has to send a larger packet.
 */
#define AUDIO_MAX_PACKET_SIZE (2u * AUDIO_PACKET_SIZE)

/**
 * @brief The number of samples in the audio buffer.
 */
#define AUDIO_BUFFER_LENGTH (AUDIO_PACKET_SIZE * AUDIO_BUFFER_PACKET_COUNT)

/**
 * @brief The target buffer fill level in number of samples.
 * @details By adding half a packet size, the buffer level is equal to half the
 * buffer size on average. Buffer level is measured only after USB packets have
 * arrived and count towards the buffer level.
 */
#define AUDIO_BUFFER_TARGET_FILL_LEVEL                                         \
    (AUDIO_BUFFER_LENGTH / 2u + AUDIO_PACKET_SIZE / 2u)

/**
 * @brief The allowed margin for the buffer fill level in samples.
 * @details If the actual fill level is closer to zero or the end of the buffer
 * than specified by the value, this application attempts to force the host to
 * adjust fill level by means of changing the reported feedback value.
 *
 * @note This should never happen, if
 * - the host adheres to the provided feedback, and does not drop packets, and
 * - does not send excessive amounts of data.
 */
#define AUDIO_BUFFER_FILL_LEVEL_MARGIN (AUDIO_PACKET_SIZE / 4)

/**
 * @brief The lower boundary for the buffer fill level in samples.
 */
#define AUDIO_BUFFER_MIN_FILL_LEVEL                                            \
    (AUDIO_BUFFER_TARGET_FILL_LEVEL - AUDIO_BUFFER_FILL_LEVEL_MARGIN)

/**
 * @brief The upper boundary for the buffer fill level in samples.
 */
#define AUDIO_BUFFER_MAX_FILL_LEVEL                                            \
    (AUDIO_BUFFER_TARGET_FILL_LEVEL + AUDIO_BUFFER_FILL_LEVEL_MARGIN)

/**
 * @brief The amount by which the feedback value is adjusted, when the buffer
 * fill level is critical.
 *
 * @details This translates to a difference in reported sample rate of
 *   \a AUDIO_FEEDBACK_CORRECTION_OFFSET * 2**14 / 1000
 *
 * For example, 16 represents a reported offset of 1.024 Hz - a mild adjustment.
 */
#define AUDIO_FEEDBACK_CORRECTION_OFFSET (256u)

/**
 * @brief The size of the packets for the feedback endpoint.
 * @details These are three bytes long, in 10.14 binary format.
 * The format represents a number in kHz, so that a 1 at a bit shift of 14 is 1
 * kHz.
 */
#define AUDIO_FEEDBACK_BUFFER_SIZE 3u

// Sanity checks.

#if AUDIO_MAX_PACKET_SIZE < AUDIO_PACKET_SIZE
#error                                                                         \
    "The maximum audio packet size should be larger than the regular packet size."
#endif

#if AUDIO_BUFFER_MIN_FILL_LEVEL > AUDIO_BUFFER_MAX_FILL_LEVEL
#error "Inconsistent settings, sample count tolerance likely too large."
#endif

// Endpoint numbers.
#define AUDIO_PLAYBACK_ENDPOINT 0x01
#define AUDIO_FEEDBACK_ENDPOINT 0x02

// Interface numbers.
#define AUDIO_CONTROL_INTERFACE   0
#define AUDIO_STREAMING_INTERFACE 1

// Functional unit numbers.
#define AUDIO_INPUT_UNIT_ID    1
#define AUDIO_FUNCTION_UNIT_ID 2
#define AUDIO_OUTPUT_UNIT_ID   3

/**
 * @brief The state of the feedback correction.
 *
 */
enum audio_feedback_correction_state {
    AUDIO_FEEDBACK_CORRECTION_STATE_OFF,  ///< No feedback correction active.
    AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE,  ///< Decrease the feedback value
                                               ///< in case of over-filled audio
                                               ///< buffer.
    AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE   ///< Increase the feedback value
                                              ///< in case of under-filled audio
                                              ///< buffer.
};

/**
 * @brief A structure that holds the state of the audio sample rate feedback.
 */
struct audio_feedback {
    enum audio_feedback_correction_state correction;
    bool b_is_first_sof;       ///< If true, the first SOF packet is yet to be
                               ///< received.
    bool   b_is_valid;         ///< Is true, if the feedback value is valid.
    size_t sof_package_count;  ///< Counts the SOF packages since the last
                               ///< feedback value update.
    uint32_t value;            ///< The current feedback value.
    uint8_t  buffer[AUDIO_FEEDBACK_BUFFER_SIZE];  ///< The current feedback
                                                  ///< buffer, derived from the
                                                  ///< feedback value.
    uint32_t last_counter_value;  ///< The counter value at the time of the
                                  ///< previous SOF interrupt.
};

/**
 * @brief A structure that holds the state of audio playback, as well as the
 * audio buffer.
 */
struct audio_playback {
    uint16_t buffer[AUDIO_BUFFER_LENGTH +
                    AUDIO_MAX_PACKET_SIZE /
                        AUDIO_SAMPLE_SIZE];  ///< The audio sample buffer.
    uint16_t buffer_write_offset;  ///< The current write offset (USB).
    uint16_t buffer_read_offset;   ///< The current read offset (I2S).
    uint16_t fill_level;  ///< The distance between read (I2S) and write (USB)
                          ///< memory locations, in units of audio samples.
    bool b_streaming_enabled;  ///< True, if audio streaming is enabled, and
                               ///< data is being received via USB.
    bool b_output_enabled;  ///< True, if the audio output is enabled, and data
                            ///< is being output via I2S.
};

/**
 * @brief A structure that holds all relevant information for audio control,
 * such as volume and mute.
 */
struct audio_control {
    uint8_t buffer[8];  ///< The provided control data.
    uint8_t channel;    ///< The current channel mask.
    bool b_channel_mute_states[AUDIO_CHANNEL_COUNT];  ///< Channel mute states.
    int16_t
        channel_volume_levels_8q8_db[AUDIO_CHANNEL_COUNT];  ///< Channel volumes
                                                            ///< in 8.8 format
                                                            ///< (in dB).
    int16_t local_volume_8q8_db;  ///< The locally set volume (volume
                                  ///< potentiometer) in 8.8 format (in dB).
};

/**
 * @brief Diagnostics for observing important properties of the audio module's
 * status.
 */
struct audio_diagnostics {
    size_t error_count;  ///< The number of buffer over-/underflow errors.
};

/**
 * @brief The main audio context.
 * @details Holds all important audio-related structures.
 */
struct audio_context {
    struct audio_feedback    feedback;     ///< The audio feedback structure.
    struct audio_playback    playback;     ///< The audio playback structure.
    struct audio_control     control;      ///< The audio control structure.
    struct audio_diagnostics diagnostics;  ///< The audio diagnostics structure.
};

event_source_t                *audio_get_event_source(void);
volatile struct audio_context *audio_get_context(void);

void                           audio_setup(void);
void                           audio_stop_playback_cb(USBDriver *usbp);
bool                           audio_requests_hook_cb(USBDriver *usbp);
void                           audio_received_cb(USBDriver *usbp, usbep_t ep);
void                           audio_feedback_cb(USBDriver *usbp, usbep_t ep);

#endif  // SOURCE_AUDIO_AUDIO_H_
