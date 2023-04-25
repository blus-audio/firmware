// Copyright 2023 elagil

/**
 * @file
 * @brief   USB Audio (UAC 1.0) module.
 * @details Contains functionality for handling UAC 1.0 audio streaming.
 *
 * @addtogroup audio
 * @{
 */

#include "audio.h"

#include <string.h>

#include "common.h"

/**
 * @brief The event source for all audio-related events.
 */
static event_source_t g_audio_event_source;

/**
 * @brief The global audio context.
 */
static volatile struct audio_context g_audio_context;

/**
 * @brief Settings structure for the I2S driver.
 * @details Enables the master clock output for timer capture.
 */
static const I2SConfig g_i2s_config = {.tx_buffer = (const uint8_t *)g_audio_context.playback.buffer,
                                       .rx_buffer = NULL,
                                       .size      = AUDIO_BUFFER_SIZE / AUDIO_SAMPLE_SIZE,  // Number of samples.
                                       .end_cb    = NULL,
#if AUDIO_RESOLUTION_BIT == 16u
                                       .i2scfgr = 0u,
#elif AUDIO_RESOLUTION_BIT == 32u
                                       .i2scfgr = SPI_I2SCFGR_DATLEN_1,
#endif
                                       .i2spr = SPI_I2SPR_MCKOE | (SPI_I2SPR_I2SDIV & 6)};

/**
 * @brief Get the global audio event source pointer.
 *
 * @return event_source_t* The pointer to the event source.
 */
inline event_source_t *audio_get_event_source(void) { return &g_audio_event_source; }

/**
 * @brief Get the audio buffer fill level.
 *
 * @return uint16_t The fill level.
 */
inline uint16_t audio_get_fill_level(void) { return g_audio_context.playback.buffer_fill_level_bytes; }

/**
 * @brief Check the mute state of an audio channel.
 *
 * @param audio_channel The audio channel to check.
 * @return true if the channel is muted.
 * @return false if the channel is not muted.
 */
inline bool audio_channel_is_muted(enum audio_channel audio_channel) {
    return g_audio_context.control.b_channel_mute_states[audio_channel];
}

/**
 * @brief Get the volume of an audio channel.
 *
 * @param audio_channel The audio channel, for which to get the volume.
 * @return int16_t The volume level in 8.8 fractional dB.
 */
inline int16_t audio_channel_get_volume(enum audio_channel audio_channel) {
    return g_audio_context.control.channel_volume_levels_8q8_db[audio_channel];
}

/**
 * @brief Check if audio playback via I2S is enabled.
 *
 * @return true if audio playback is enabled.
 * @return false if audio playback is disabled.
 */
inline bool audio_playback_is_enabled(void) { return g_audio_context.playback.b_playback_enabled; }

/**
 * @brief Initialize the audio diagnostics structure.
 *
 * @param p_diagnostics The pointer to the structure to initialize.
 */
static void audio_init_diagnostics(volatile struct audio_diagnostics *p_diagnostics) {
    p_diagnostics->error_count = 0u;
}

/**
 * @brief Initialize the audio feedback structure.
 *
 * @param p_feedback The pointer to the structure to initialize.
 */
static void audio_init_feedback(volatile struct audio_feedback *p_feedback) {
    p_feedback->correction_state   = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
    p_feedback->b_is_first_sof     = true;
    p_feedback->b_is_valid         = false;
    p_feedback->sof_package_count  = 0u;
    p_feedback->last_counter_value = 0u;
    p_feedback->value              = 0u;
}

/**
 * @brief Initialize the audio playback structure.
 *
 * @param p_playback The pointer to the structure to initialize.
 */
static void audio_init_playback(volatile struct audio_playback *p_playback) {
    p_playback->buffer_write_offset     = 0u;
    p_playback->buffer_read_offset      = 0u;
    p_playback->buffer_fill_level_bytes = 0u;
    p_playback->b_playback_enabled      = false;
    p_playback->b_streaming_enabled     = false;
}

/**
 * @brief Initialize the audio control structure.
 *
 * @param p_control The pointer to the structure to initialize.
 */
static void audio_init_control(volatile struct audio_control *p_control) {
    p_control->channel             = 0u;
    p_control->local_volume_8q8_db = 0;

    for (size_t channel_index = 0; channel_index < AUDIO_CHANNEL_COUNT; channel_index++) {
        p_control->b_channel_mute_states[channel_index]        = false;
        p_control->channel_volume_levels_8q8_db[channel_index] = 0;
    }
}

/**
 * @brief Initialize an audio context, and all its contained structures.
 *
 * @param p_context The pointer to the context to initialize.
 */
static void audio_init_context(volatile struct audio_context *p_context) {
    chEvtObjectInit(&g_audio_event_source);
    audio_init_feedback(&p_context->feedback);
    audio_init_playback(&p_context->playback);
    audio_init_control(&p_context->control);
    audio_init_diagnostics(&p_context->diagnostics);
}

/**
 * @brief Perform a manual correction on the feedback value, for steering the host's sample rate.
 * @details The general USB 2.0 specification states (5.12.4.2, p. 75): "It is possible that the source will deliver one
 * too many or one too few samples over a long period due to errors or accumulated inaccuracies in measuring Ff. The
 * sink must have sufficient buffer capability to accommodate this. When the sink recognizes this condition, it should
 * adjust the reported Ff value to correct it. This may also be necessary to compensate for relative clock drifts."
 *
 * FIXME: On Windows, pausing audio for a while does not immediately switch alternate modes to zero-bandwidth, but leads
 * to the generation of zero-length audio packets. This application can detect zero packets, and stops playback, and the
 * transmission of feedback values. When the audio is started again, there seems to be host-side confusion about the
 * size of audio packets that need to be transmitted - they are usually too small in the beginning. Thus, the audio
 * buffer may underrun, which needs to be compensated manually. This audio feedback correction function is a workaround
 * for that symptom.
 */
static void audio_feedback_correct(void) {
    volatile struct audio_feedback    *p_feedback    = &g_audio_context.feedback;
    volatile struct audio_playback    *p_playback    = &g_audio_context.playback;
    volatile struct audio_diagnostics *p_diagnostics = &g_audio_context.diagnostics;

    if (p_feedback->correction_state == AUDIO_FEEDBACK_CORRECTION_STATE_OFF) {
        if (p_playback->buffer_fill_level_bytes > AUDIO_BUFFER_MAX_FILL_LEVEL_BYTES) {
            // The fill level is too high, compensate by means of lower feedback value.
            p_feedback->correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE;
            p_diagnostics->error_count++;
        } else if (p_playback->buffer_fill_level_bytes < AUDIO_BUFFER_MIN_FILL_LEVEL_BYTES) {
            // The fill level is too low, compensate by means of higher feedback value.
            p_feedback->correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE;
            p_diagnostics->error_count++;
        }
    }

    switch (p_feedback->correction_state) {
        case AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE:
            if (p_playback->buffer_fill_level_bytes <= AUDIO_BUFFER_TARGET_FILL_LEVEL_BYTES) {
                // Switch off correction, when reaching target fill level.
                p_feedback->correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
            } else {
                p_feedback->value -= AUDIO_FEEDBACK_CORRECTION_OFFSET;
            }
            break;

        case AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE:
            if (p_playback->buffer_fill_level_bytes >= AUDIO_BUFFER_TARGET_FILL_LEVEL_BYTES) {
                // Switch off correction, when reaching target fill level.
                p_feedback->correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
            }
            { p_feedback->value += AUDIO_FEEDBACK_CORRECTION_OFFSET; }
            break;

        default:
            break;
    }
}

/**
 * @brief The interrupt handler for timer TIM2.
 * @details Called upon reception of a USB start of frame (SOF) signal. The timer is used for counting the interval
 * between SOF signals, which arrive roughly with a frequency of 1 kHz. Since the timer is clocked by the I2S master
 * clock, it counts the SOF period in terms of the I2S frequency. In other words, it relates the SOF period to the
 * output sample rate, which is required for feedback calculation.
 *
 * USB audio feedback calculation requires the device to provide its sampling rate in relation to the SOF period.
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER) {
    volatile struct audio_feedback *p_feedback = &g_audio_context.feedback;

    OSAL_IRQ_PROLOGUE();

    chDbgAssert(I2S_DRIVER.state == I2S_ACTIVE, "SOF period capture not possible with I2S inactive.");

    uint32_t counter_value         = TIM2->CNT;
    uint32_t timer_status_register = TIM2->SR;

    // Reset any timer interrupt flags.
    TIM2->SR = 0;

    if (!(timer_status_register & TIM_SR_TIF)) {
        // Trigger interrupt flag was not set.
        OSAL_IRQ_EPILOGUE();
        return;
    }

    if (p_feedback->b_is_first_sof) {
        // On the first SOF signal, the feedback cannot be calculated yet.
        // Only record the timer state.
        p_feedback->last_counter_value = counter_value;
        p_feedback->b_is_first_sof     = false;
        OSAL_IRQ_EPILOGUE();
        return;
    }

    // Normal playback operation below.

    // Feedback value is calculated every 64 SOF interrupts => every 64 ms.
    p_feedback->sof_package_count++;
    if (p_feedback->sof_package_count == 64u) {
        // Conveniently, the timer count difference at 64 ms count periods matches the required feedback format. The
        // feedback endpoint requires the device sample rate in kHz in a 10.14 binary (fixpoint) format.
        // - The master clock runs 256 times as fast as the reference clock.
        // - The conversion of fs / kHz to fs / Hz adds a factor of 1000
        // - The counting period is 64 ms long.
        //
        // Thus
        //   fs / kHz * 2**14 = fs * 256 * 64 ms
        //
        // where the left side is the expected format, and the right side the result of this counting function.
        p_feedback->value = subtract_circular_unsigned(counter_value, p_feedback->last_counter_value, UINT32_MAX);

        p_feedback->last_counter_value = counter_value;

        // If there is too much discrepancy between the target sample buffer fill level, and the actual fill level, this
        // must be compensated manually.
        audio_feedback_correct();

        p_feedback->sof_package_count = 0u;
        p_feedback->b_is_valid        = true;
    }

    OSAL_IRQ_EPILOGUE();
}

/**
 * @brief Set up the timer peripheral for counting USB start of frame (SOF) periods.
 * @note Only start after the I2S peripheral is running. Its MCLK output clocks this timer.
 */
static inline void audio_start_sof_capture(void) {
    chSysLock();
    chDbgAssert(I2S_DRIVER.state == I2S_ACTIVE, "Only start SOF capture after the I2S driver.");

    // Reset TIM2 instance.
    rccResetTIM2();
    nvicEnableVector(STM32_TIM2_NUMBER, STM32_IRQ_TIM2_PRIORITY);

    // Enable TIM2 counter.
    TIM2->CR1 = TIM_CR1_CEN;
    // - The timer clock source is the ETR pin.
    // - Enable slave mode.
    // - Trigger on ITR1 (the SOF signal).
    TIM2->SMCR = TIM_SMCR_ECE | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
    // Enable TIM2 interrupt.
    TIM2->DIER = TIM_DIER_TIE;
    // Remap ITR1 to the USB_FS SOF signal.
    TIM2->OR = TIM_OR_ITR1_RMP_1;

    audio_init_feedback(&g_audio_context.feedback);
    chSysUnlock();
}

/**
 * @brief Stop the timer peripheral for counting USB start of frame (SOF) periods.
 */
static inline void audio_stop_sof_capture(void) {
    chSysLock();
    nvicDisableVector(STM32_TIM2_NUMBER);
    TIM2->CR1 = 0;
    audio_init_feedback(&g_audio_context.feedback);
    chSysUnlock();
}

/**
 * @brief Joint callback for when feedback was transmitted, or failed, in the current frame.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param ep The endpoint, for which the feedback was called.
 */
void audio_feedback_cb(USBDriver *usbp, usbep_t ep) {
    volatile struct audio_feedback *p_feedback = &g_audio_context.feedback;
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_streaming_enabled) {
        // Feedback is only active while streaming.
        return;
    }

    chSysLockFromISR();

    if (p_feedback->b_is_valid) {
        static uint8_t feedback_buffer[AUDIO_FEEDBACK_BUFFER_SIZE];
        value_to_byte_array(feedback_buffer, p_feedback->value, AUDIO_FEEDBACK_BUFFER_SIZE);

        usbStartTransmitI(usbp, ep, feedback_buffer, AUDIO_FEEDBACK_BUFFER_SIZE);
    } else {
        // Transmit an empty packet.
        usbStartTransmitI(usbp, ep, NULL, 0);
    }

    chSysUnlockFromISR();
}

/**
 * @brief Update the audio buffer write offset, taking into account wrap-around of the circular buffer.
 * @details If the nominal buffer size was exceeded by the last packet, the excess is copied to the beginning of the
 * buffer. The audio buffer is large enough to handle excess data of size \a AUDIO_MAX_PACKET_SIZE.
 * @param transaction_size The received audio byte count.
 */
static inline void audio_update_write_offset(size_t transaction_size) {
    volatile struct audio_playback *p_playback              = &g_audio_context.playback;
    size_t                          new_buffer_write_offset = p_playback->buffer_write_offset + transaction_size;

    chDbgAssert(new_buffer_write_offset < ARRAY_LENGTH(p_playback->buffer), "Transaction size exceeds audio buffer.");

#if AUDIO_RESOLUTION_BIT == 32u
    // Swap upper and lower 16 bit of received audio samples to comply with the expected I2S DMA data format.
    for (size_t sample_index = 0; sample_index < transaction_size / AUDIO_SAMPLE_SIZE; sample_index++) {
        size_t    sample_offset       = p_playback->buffer_write_offset + AUDIO_SAMPLE_SIZE * sample_index;

        uint16_t *p_lower_sample_half = (uint16_t *)(&p_playback->buffer[sample_offset]);
        uint16_t *p_upper_sample_half = (uint16_t *)(&p_playback->buffer[sample_offset + sizeof(uint16_t)]);

        uint16_t  lower_sample_half   = *p_lower_sample_half;
        uint16_t  upper_sample_half   = *p_upper_sample_half;

        *p_upper_sample_half          = lower_sample_half;
        *p_lower_sample_half          = upper_sample_half;
    }
#endif

    // Copy excessive data back to the start of the audio buffer.
    if (new_buffer_write_offset > AUDIO_BUFFER_SIZE) {
        size_t excess_byte_count = new_buffer_write_offset - AUDIO_BUFFER_SIZE;
        memcpy((void *)p_playback->buffer, (void *)&p_playback->buffer[AUDIO_BUFFER_SIZE], excess_byte_count);
    }

    p_playback->buffer_write_offset = wrap_unsigned(new_buffer_write_offset, AUDIO_BUFFER_SIZE);
}

/**
 * @brief Determine the I2S DMA's current read offset from the audio buffer start.
 * @details The information is stored in the \a audio_playback structure.
 */
static inline void audio_update_read_offset(void) {
    volatile struct audio_playback *p_playback              = &g_audio_context.playback;
    size_t                          number_of_data_register = (size_t)(I2S_DRIVER.dmatx->stream->NDTR);

    // For 16 bit audio, the number of data register (NDTR) holds the number of remaining audio samples.
    size_t transferrable_sample_count = number_of_data_register;

#if AUDIO_RESOLUTION_BIT == 32u
    // For 32 bit audio, the number of data register still counts 16 bit wide samples.
    transferrable_sample_count /= 2u;
#endif

    if (I2S_DRIVER.state == I2S_ACTIVE) {
        p_playback->buffer_read_offset =
            (size_t)AUDIO_BUFFER_SIZE - (size_t)AUDIO_SAMPLE_SIZE * transferrable_sample_count;
    } else {
        p_playback->buffer_read_offset = 0u;
    }
}

/**
 * @brief Calculate the audio buffer fill level.
 * @details This is the difference in bytes between the write offset (USB) and read offset (I2S DMA) - the number of
 * bytes that can still be written via I2S, before the buffer runs out.
 */
static inline void audio_update_fill_level(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    // Calculate the distance between the DMA read offset, and the USB driver's write offset in the playback buffer.
    p_playback->buffer_fill_level_bytes =
        subtract_circular_unsigned(p_playback->buffer_write_offset, p_playback->buffer_read_offset, AUDIO_BUFFER_SIZE);
}

/**
 * @brief Start playback, when the target audio buffer fill level is reached.
 * @details I2S transfers are started by emitting the \a AUDIO_EVENT_START_PLAYBACK event.
 */
static inline void audio_start_playback(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (p_playback->b_playback_enabled) {
        // Playback already enabled.
        return;
    }

    if (p_playback->buffer_fill_level_bytes >= AUDIO_BUFFER_TARGET_FILL_LEVEL_BYTES) {
        // Signal that the playback buffer is at or above the target fill level. This starts audio playback via I2S.
        p_playback->b_playback_enabled = true;
        chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_START_PLAYBACK);
    }
}

/**
 * @brief Disables audio playback.
 * @details Emits an \a AUDIO_EVENT_STOP_PLAYBACK event.
 * @note This internally uses I-class functions.
 */
static inline void audio_stop_playback(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_playback_enabled) {
        // Playback already disabled.
        return;
    }

    p_playback->b_playback_enabled  = false;
    p_playback->buffer_write_offset = 0;

    chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_STOP_PLAYBACK);
}

/**
 * @brief Joint callback for when audio data was received from the host, or the reception failed in the current frame.
 * @note This internally uses I-class functions.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param ep The endpoint, for which the feedback was called.
 */
void audio_received_cb(USBDriver *usbp, usbep_t ep) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_streaming_enabled) {
        // Disregard packets, when streaming is disabled.
        return;
    }

    size_t transaction_size = usbGetReceiveTransactionSizeX(usbp, ep);

    chSysLockFromISR();

    if (transaction_size == 0u) {
        audio_stop_playback();
    } else {
        // Samples have been written by the USB DMA.
        audio_update_write_offset(transaction_size);
    }

    usbStartReceiveI(usbp, ep, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset], AUDIO_MAX_PACKET_SIZE);

    audio_update_read_offset();
    audio_update_fill_level();
    audio_start_playback();

    chSysUnlockFromISR();
}

/**
 * @brief Volume levels were changed. Update the audio context with new values.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_update_volumes(USBDriver *usbp) {
    (void)usbp;
    volatile struct audio_control *p_control = &g_audio_context.control;

    chSysLockFromISR();
    if (p_control->channel == 0xFFu) {
        memcpy((int16_t *)p_control->channel_volume_levels_8q8_db, (int16_t *)p_control->buffer + sizeof(int16_t),
               2u * sizeof(int16_t));
    } else {
        size_t audio_channel_index = p_control->channel - 1u;
        chDbgAssert(audio_channel_index < ARRAY_LENGTH(p_control->channel_volume_levels_8q8_db),
                    "Invalid volume channel index.");

        memcpy((int16_t *)&p_control->channel_volume_levels_8q8_db[audio_channel_index], (int16_t *)p_control->buffer,
               sizeof(int16_t));
    }
    chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_SET_VOLUME);
    chSysUnlockFromISR();
}

/**
 * @brief Channel mute information was changed. Updates the audio context.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_update_mute_states(USBDriver *usbp) {
    (void)usbp;
    volatile struct audio_control *p_control = &g_audio_context.control;

    if (p_control->channel == AUDIO_MASTER_CHANNEL) {
        p_control->b_channel_mute_states[AUDIO_CHANNEL_LEFT]  = p_control->buffer[1u];
        p_control->b_channel_mute_states[AUDIO_CHANNEL_RIGHT] = p_control->buffer[2u];
    } else {
        size_t audio_channel_index = p_control->channel - 1u;
        chDbgAssert(audio_channel_index < ARRAY_LENGTH(p_control->b_channel_mute_states),
                    "Invalid mute channel index.");

        p_control->b_channel_mute_states[audio_channel_index] = p_control->buffer[0u];
    }
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_SET_MUTE_STATE);
    chSysUnlockFromISR();
}

/**
 * @brief Handle requests for the audio function unit (mute and volume control)
 *
 * @param usbp A pointer to the USB driver structure.
 * @param req The basic request to handle.
 * @param ctrl The control unit, for which the request applies.
 * @param channel The channel to control.
 * @param length The length of the requested control data.
 * @return true in case of a successful request handling.
 * @return false in case of an unknown request, or an inconsistent request.
 */
static bool audio_handle_function_unit_request(USBDriver *usbp, uint8_t req, uint8_t ctrl, uint8_t channel,
                                               uint16_t length) {
    volatile struct audio_control *p_control = &g_audio_context.control;
    uint8_t                       *p_buffer  = (uint8_t *)p_control->buffer;

    switch (req) {
        case UAC_REQ_SET_MAX:
        case UAC_REQ_SET_MIN:
        case UAC_REQ_SET_RES:
            if (ctrl == UAC_FU_VOLUME_CONTROL) {
                usbSetupTransfer(usbp, p_buffer, length, NULL);
                return true;
            }
            break;

        case UAC_REQ_GET_MAX:
            if (ctrl == UAC_FU_VOLUME_CONTROL) {
                for (size_t i = 0; i < length; i++) ((int16_t *)p_buffer)[i] = 0;
                usbSetupTransfer(usbp, p_buffer, length, NULL);
                return true;
            }
            break;

        case UAC_REQ_GET_MIN:
            if (ctrl == UAC_FU_VOLUME_CONTROL) {
                for (size_t i = 0; i < length; i++) ((int16_t *)p_buffer)[i] = -100 * 256;
                usbSetupTransfer(usbp, p_buffer, length, NULL);
                return true;
            }
            break;

        case UAC_REQ_GET_RES:
            if (ctrl == UAC_FU_VOLUME_CONTROL) {
                for (size_t i = 0; i < length; i++) ((int16_t *)p_buffer)[i] = 128;
                usbSetupTransfer(usbp, p_buffer, length, NULL);
                return true;
            }
            break;

        case UAC_REQ_GET_CUR:
            if (ctrl == UAC_FU_MUTE_CONTROL) {
                if (channel == 0xff) {
                    uint8_t value[3] = {0, p_control->b_channel_mute_states[AUDIO_CHANNEL_LEFT],
                                        p_control->b_channel_mute_states[AUDIO_CHANNEL_RIGHT]};
                    memcpy(p_buffer, value, sizeof(value));
                    usbSetupTransfer(usbp, p_buffer, length, NULL);
                } else {
                    *p_buffer = p_control->b_channel_mute_states[channel - 1];
                    usbSetupTransfer(usbp, p_buffer, length, NULL);
                }
                return true;
            } else if (ctrl == UAC_FU_VOLUME_CONTROL) {
                if (channel == 0xff) {
                    int16_t value[3] = {0, p_control->channel_volume_levels_8q8_db[AUDIO_CHANNEL_LEFT],
                                        p_control->channel_volume_levels_8q8_db[AUDIO_CHANNEL_RIGHT]};
                    memcpy(p_buffer, value, sizeof(value));
                    usbSetupTransfer(usbp, p_buffer, length, NULL);
                } else {
                    memcpy(p_buffer, (uint8_t *)&p_control->channel_volume_levels_8q8_db[channel - 1], sizeof(int16_t));
                    usbSetupTransfer(usbp, p_buffer, length, NULL);
                }
                return true;
            }
            break;

        case UAC_REQ_SET_CUR:
            if (ctrl == UAC_FU_MUTE_CONTROL) {
                p_control->channel = channel;
                usbSetupTransfer(usbp, p_buffer, length, audio_update_mute_states);
                return true;
            } else if (ctrl == UAC_FU_VOLUME_CONTROL) {
                p_control->channel = channel;
                usbSetupTransfer(usbp, p_buffer, length, audio_update_volumes);
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
 * @param iface The target interface of the control message.
 * @param entity The target functional unit.
 * @param req The request code to handle.
 * @param wValue The request payload.
 * @param length The length of the requested data.
 * @return true if a request could be answered successfully.
 * @return false if a request could not be answered successfully.
 */
bool audio_control_cb(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req, uint16_t wValue, uint16_t length) {
    if (iface != AUDIO_CONTROL_INTERFACE) {
        // Only requests to audio control iface are supported.
        return false;
    }

    if (entity == AUDIO_FUNCTION_UNIT_ID) {
        // Handle requests to the audio function unit
        return audio_handle_function_unit_request(usbp, req, (wValue >> AUDIO_BIT_PER_BYTE) & AUDIO_BYTE_MASK,
                                                  wValue & AUDIO_BYTE_MASK, length);
    }

    // No control message handling took place.
    return false;
}

/**
 * @brief Start streaming audio via USB.
 * @details Is called, when the audio endpoint goes into its operational alternate mode (actual music playback begins).
 *
 * @param usbp The pointer to the USB driver structure.
 */
static void audio_start_streaming(USBDriver *usbp) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (p_playback->b_streaming_enabled) {
        // Streaming is already enabled.
        return;
    }

    audio_init_playback(p_playback);
    p_playback->b_streaming_enabled = true;

    chSysLockFromISR();

    // Feedback yet unknown, transmit empty packet.
    usbStartTransmitI(usbp, AUDIO_FEEDBACK_ENDPOINT, NULL, 0);

    // Initial audio data reception.
    usbStartReceiveI(usbp, AUDIO_PLAYBACK_ENDPOINT, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset],
                     AUDIO_MAX_PACKET_SIZE);

    chSysUnlockFromISR();
}

/**
 * @brief Disable audio streaming and output.
 * @details Is called on USB reset, or when the audio endpoint goes into its zero bandwidth alternate mode.
 *
 * @param usbp The pointer to the USB driver structure.
 */
void audio_stop_streaming(USBDriver *usbp) {
    (void)usbp;
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_streaming_enabled) {
        // Streaming is already disabled.
        return;
    }

    p_playback->b_streaming_enabled = false;

    chSysLockFromISR();
    audio_stop_playback();
    chSysUnlockFromISR();
}

/**
 * @brief Handles setup requests.
 *
 * @param usbp A pointer to the USB driver structure.
 * @return true if a setup request could be handled.
 * @return false if a setup request could not be handled.
 */
bool audio_requests_hook_cb(USBDriver *usbp) {
    if ((usbp->setup[0] & (USB_RTYPE_TYPE_MASK | USB_RTYPE_RECIPIENT_MASK)) ==
        (USB_RTYPE_TYPE_STD | USB_RTYPE_RECIPIENT_INTERFACE)) {
        if (usbp->setup[1] == USB_REQ_SET_INTERFACE) {
            /* Switch between empty interface and normal one. */
            if (((usbp->setup[5] << 8) | usbp->setup[4]) == AUDIO_STREAMING_INTERFACE) {
                if (((usbp->setup[3] << 8) | usbp->setup[2]) == 1) {
                    audio_start_streaming(usbp);
                } else {
                    audio_stop_streaming(usbp);
                }
                usbSetupTransfer(usbp, NULL, 0, NULL);
                return true;
            }
        }
        return false;
    } else if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) {
        switch (usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) {
            case USB_RTYPE_RECIPIENT_INTERFACE:
                return audio_control_cb(usbp, usbp->setup[4], usbp->setup[5], usbp->setup[1],
                                        (usbp->setup[3] << 8) | (usbp->setup[2]),
                                        (usbp->setup[7] << 8) | (usbp->setup[6]));
            case USB_RTYPE_RECIPIENT_ENDPOINT:
            default:
                return false;
        }
    }
    return false;
}

static THD_WORKING_AREA(wa_audio_thread, 128);

/**
 * @brief A thread that handles audio-related tasks.
 */
static THD_FUNCTION(audio_thread, arg) {
    (void)arg;
    chRegSetThreadName("audio");

    // Registers this thread for audio events.
    static event_listener_t audio_event_listener;
    chEvtRegisterMaskWithFlags(&g_audio_event_source, &audio_event_listener, AUDIO_EVENT,
                               AUDIO_EVENT_START_PLAYBACK | AUDIO_EVENT_STOP_PLAYBACK);

    // Enable the feedback counter timer TIM2 peripheral clock (no low-power mode).
    rccEnableTIM2(false);

    while (true) {
        chEvtWaitOne(AUDIO_EVENT);
        eventflags_t event_flags = chEvtGetAndClearFlags(&audio_event_listener);

        // Generally, the SOF capture (for feedback calculation) must be started after/stopped before the I2S peripheral
        // - the I2S master clock output is required for counting the SOF interval.

        if (event_flags & AUDIO_EVENT_START_PLAYBACK) {
            // Set volumes to the values configured via USB audio.
            chEvtBroadcastFlags(&g_audio_event_source, AUDIO_EVENT_SET_VOLUME);

            i2sStart(&I2S_DRIVER, &g_i2s_config);
            i2sStartExchange(&I2S_DRIVER);
            audio_start_sof_capture();
        }

        if (event_flags & AUDIO_EVENT_STOP_PLAYBACK) {
            audio_stop_sof_capture();
            i2sStopExchange(&I2S_DRIVER);
            i2sStop(&I2S_DRIVER);

            // Reset volumes to default.
            chEvtBroadcastFlags(&g_audio_event_source, AUDIO_EVENT_RESET_VOLUME);
        }
    }
}

/**
 * @brief Set up all components of the audio module.
 */
void audio_setup(void) {
    audio_init_context(&g_audio_context);
    chThdCreateStatic(wa_audio_thread, sizeof(wa_audio_thread), NORMALPRIO, audio_thread, NULL);
}

/**
 * @}
 */
