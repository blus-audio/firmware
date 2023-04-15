// Copyright 2023 elagil
#include "audio.h"

#include <string.h>

#include "common.h"

/**
 * @brief The event source for all audio-related events.
 */
static event_source_t g_audio_event_source;

/**
 * @brief The global audio context.
 * @details Can be shared with other modules by means of \a audio_get_context().
 */
static volatile struct audio_context g_audio_context;

/**
 * @brief Settings structure for the I2S driver.
 * @details Enables the master clock output for timer capture.
 */
static const I2SConfig g_i2s_config = {.tx_buffer = (const uint8_t *)g_audio_context.playback.buffer,
                                       .rx_buffer = NULL,
                                       .size      = AUDIO_BUFFER_LENGTH,
                                       .end_cb    = NULL,
                                       .i2spr     = SPI_I2SPR_MCKOE | (SPI_I2SPR_I2SDIV & 6)};

/**
 * @brief Get the global audio event source.
 *
 * @return event_source_t* The pointer to the event source.
 */
event_source_t *audio_get_event_source(void) { return &g_audio_event_source; }

/**
 * @brief Get the global audio context.
 *
 * @return volatile struct* The pointer to the audio context.
 */
volatile struct audio_context *audio_get_context(void) { return &g_audio_context; }

/**
 * @brief Get the audio buffer fill level.
 *
 * @return uint16_t The fill level.
 */
uint16_t audio_get_fill_level(void) { return g_audio_context.playback.fill_level; }

/**
 * @brief Check the mute state of an audio channel.
 *
 * @param audio_channel The audio channel to check.
 * @return true if the channel is muted.
 * @return false if the channel is not muted.
 */
bool audio_channel_is_muted(enum audio_channel audio_channel) {
    return g_audio_context.control.b_channel_mute_states[audio_channel];
}

/**
 * @brief Get the volume of an audio channel.
 *
 * @param audio_channel The audio channel, for which to get the volume.
 * @return int16_t The volume level in 8.8 fractional dB.
 */
int16_t audio_channel_get_volume(enum audio_channel audio_channel) {
    return g_audio_context.control.channel_volume_levels_8q8_db[audio_channel];
}

/**
 * @brief Check if audio streaming via USB is enabled.
 *
 * @return true if streaming is enabled.
 * @return false if streaming is disabled.
 */
bool audio_is_streaming(void) { return g_audio_context.playback.b_streaming_enabled; }

/**
 * @brief Initialize the audio diagnostics structure.
 *
 * @param p_diagnostics The pointer to the structure to initialize.
 */
void audio_init_diagnostics(volatile struct audio_diagnostics *p_diagnostics) { p_diagnostics->error_count = 0u; }

/**
 * @brief Initialize the audio feedback structure.
 *
 * @param p_feedback The pointer to the structure to initialize.
 */
void audio_init_feedback(volatile struct audio_feedback *p_feedback) {
    p_feedback->correction         = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
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
void audio_init_playback(volatile struct audio_playback *p_playback) {
    p_playback->buffer_write_offset = 0u;
    p_playback->buffer_read_offset  = 0u;
    p_playback->fill_level          = 0u;
    p_playback->b_output_enabled    = false;
    p_playback->b_streaming_enabled = false;
}

/**
 * @brief Initialize the audio control structure.
 *
 * @param p_control The pointer to the structure to initialize.
 */
void audio_init_control(volatile struct audio_control *p_control) {
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
void audio_init_context(volatile struct audio_context *p_context) {
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
void audio_feedback_correct(void) {
    volatile struct audio_feedback    *p_feedback    = &g_audio_context.feedback;
    volatile struct audio_playback    *p_playback    = &g_audio_context.playback;
    volatile struct audio_diagnostics *p_diagnostics = &g_audio_context.diagnostics;

    if (p_feedback->correction == AUDIO_FEEDBACK_CORRECTION_STATE_OFF) {
        if (p_playback->fill_level > AUDIO_BUFFER_MAX_FILL_LEVEL) {
            // The fill level is too high, compensate by means of lower feedback
            // value.
            p_feedback->correction = AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE;
            p_diagnostics->error_count++;
        } else if (p_playback->fill_level < AUDIO_BUFFER_MIN_FILL_LEVEL) {
            // The fill level is too low, compensate by means of higher feedback
            // value.
            p_feedback->correction = AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE;
            p_diagnostics->error_count++;
        }
    }

    switch (p_feedback->correction) {
        case AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE:
            if (p_playback->fill_level <= AUDIO_BUFFER_TARGET_FILL_LEVEL) {
                // Switch off correction, when reaching target fill level.
                p_feedback->correction = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
            } else {
                p_feedback->value -= AUDIO_FEEDBACK_CORRECTION_OFFSET;
            }
            break;

        case AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE:
            if (p_playback->fill_level >= AUDIO_BUFFER_TARGET_FILL_LEVEL) {
                // Switch off correction, when reaching target fill level.
                p_feedback->correction = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
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
    if (p_feedback->sof_package_count == 64) {
        // Conveniently, the timer count difference at 64 ms count periods matches the required feedback format. The
        // feedback endpoint requires the device sample rate in kHz in a 10.14 binary (fixpoint) format.
        // - The master clock runs 256 times as fast as the reference clock.
        // - The conversion of fs / kHz to fs / Hz adds a factor of 1000
        // - The counting period is 64 ms long.
        //
        // Thus
        //   fs / kHz * 2**14 = fs * 256 * 64e-3 s
        //
        // where the left side is the expected format, and the right side the result of this counting function.
        p_feedback->value = subtract_circular_unsigned(counter_value, p_feedback->last_counter_value, UINT32_MAX);

        p_feedback->last_counter_value = counter_value;

        // If there is too much discrepancy between the target sample buffer fill level, and the actual fill level, this
        // must be compensated manually.
        audio_feedback_correct();

        // Translate the feedback value to a buffer of three bytes.
        value_to_byte_array((uint8_t *)p_feedback->buffer, p_feedback->value, AUDIO_FEEDBACK_BUFFER_SIZE);

        p_feedback->sof_package_count = 0u;
        p_feedback->b_is_valid        = true;
    }

    OSAL_IRQ_EPILOGUE();
}

/**
 * @brief Set up the timer peripheral for counting USB start of frame (SOF) periods.
 * @note Only start after the I2S peripheral is running. Its MCLK output clocks this timer.
 */
void audio_start_sof_capture(void) {
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
void audio_stop_sof_capture(void) {
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
        return;
    }

    chSysLockFromISR();

    if (p_feedback->b_is_valid) {
        // Local copy that cannot be overwritten by an ISR.
        static uint8_t feedback_buffer[AUDIO_FEEDBACK_BUFFER_SIZE];
        memcpy(feedback_buffer, (uint8_t *)p_feedback->buffer, AUDIO_FEEDBACK_BUFFER_SIZE);

        usbStartTransmitI(usbp, ep, feedback_buffer, AUDIO_FEEDBACK_BUFFER_SIZE);
    } else {
        // Transmit an empty packet.
        usbStartTransmitI(usbp, ep, NULL, 0);
    }

    chSysUnlockFromISR();
}

/**
 * @brief Update the audio buffer write pointer, taking into account wrap-around of the circular buffer.
 * @details If the nominal buffer size was exceeded by the last packet, the excess is copied to the beginning of the
 * buffer. The audio buffer is large enough to handle excess data of size \a AUDIO_MAX_PACKET_SIZE.
 * @param received_sample_count The number of newly received samples.
 */
void audio_update_buffer(uint16_t received_sample_count) {
    volatile struct audio_playback *p_playback              = &g_audio_context.playback;
    uint16_t                        new_buffer_write_offset = p_playback->buffer_write_offset + received_sample_count;

    // Copy excessive data back to the start of the audio buffer.
    if (new_buffer_write_offset > AUDIO_BUFFER_LENGTH) {
        for (size_t sample_index = AUDIO_BUFFER_LENGTH; sample_index < new_buffer_write_offset; sample_index++) {
            p_playback->buffer[sample_index - AUDIO_BUFFER_LENGTH] = p_playback->buffer[sample_index];
        }
    }

    p_playback->buffer_write_offset = wrap_unsigned(new_buffer_write_offset, AUDIO_BUFFER_LENGTH);
}

/**
 * @brief Determine the I2S DMA's current read offset from the audio buffer start.
 * @details The information is stored in the \a audio_playback structure.
 */
void audio_update_read_offset(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (I2S_DRIVER.state == I2S_ACTIVE) {
        p_playback->buffer_read_offset = (uint16_t)AUDIO_BUFFER_LENGTH - (uint16_t)(I2S_DRIVER.dmatx->stream->NDTR);
    } else {
        p_playback->buffer_read_offset = 0u;
    }
}

/**
 * @brief Calculate the audio buffer fill level.
 * @details This is the difference in samples between the write pointer (USB) and read pointer (I2S DMA) - the number of
 * samples that can still be written via I2S, before the buffer runs out..
 */
void audio_update_fill_level(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    // Calculate the distance between the DMA read pointer, and the USB driver's write pointer in the playback buffer.
    p_playback->fill_level = subtract_circular_unsigned(p_playback->buffer_write_offset, p_playback->buffer_read_offset,
                                                        AUDIO_BUFFER_LENGTH);
}

/**
 * @brief Handle non-zero audio packets (regular playback).
 * @details Start I2S transfers by emitting \a AUDIO_EVENT_START_PLAYBACK event, when the target audio buffer fill level
 * is reached.
 */
void audio_handle_valid_packet(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (p_playback->b_output_enabled) {
        // Do nothing, when playback is already enabled.
        return;
    }

    if (p_playback->fill_level >= AUDIO_BUFFER_TARGET_FILL_LEVEL) {
        // Signal that the playback buffer is at or above the target fill level. This starts audio playback via I2S.
        p_playback->b_output_enabled = true;
        chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_START_PLAYBACK);
    }
}

/**
 * @brief Disables playback, if an empty packet was received.
 * @details Emits an \a AUDIO_EVENT_STOP_PLAYBACK event.
 * @note This internally uses I-class functions.
 */
void audio_handle_empty_packet(void) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (p_playback->b_output_enabled) {
        p_playback->b_output_enabled    = false;
        p_playback->buffer_write_offset = 0;
        chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_STOP_PLAYBACK);
    }
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

    uint16_t received_sample_count = usbGetReceiveTransactionSizeX(usbp, ep) / AUDIO_SAMPLE_SIZE;

    audio_update_buffer(received_sample_count);
    audio_update_read_offset();
    audio_update_fill_level();

    chSysLockFromISR();

    if (received_sample_count == 0) {
        audio_handle_empty_packet();
    } else {
        audio_handle_valid_packet();
    }

    usbStartReceiveI(usbp, ep, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset], AUDIO_MAX_PACKET_SIZE);

    chSysUnlockFromISR();
}

/**
 * @brief Volume levels were changed. Update the audio context with new values.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_notify_volume_cb(USBDriver *usbp) {
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
    chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_VOLUME);
    chSysUnlockFromISR();
}

/**
 * @brief Channel mute information was changed. Updates the audio context.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_notify_mute_cb(USBDriver *usbp) {
    (void)usbp;
    volatile struct audio_control *p_control = &g_audio_context.control;

    if (p_control->channel == 0xFFu) {
        p_control->b_channel_mute_states[AUDIO_CHANNEL_LEFT]  = p_control->buffer[1u];
        p_control->b_channel_mute_states[AUDIO_CHANNEL_RIGHT] = p_control->buffer[2u];
    } else {
        size_t audio_channel_index = p_control->channel - 1u;
        chDbgAssert(audio_channel_index < ARRAY_LENGTH(p_control->b_channel_mute_states),
                    "Invalid mute channel index.");

        p_control->b_channel_mute_states[audio_channel_index] = p_control->buffer[0u];
    }
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_MUTE);
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
bool audio_handle_request_cb(USBDriver *usbp, uint8_t req, uint8_t ctrl, uint8_t channel, uint16_t length) {
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
                usbSetupTransfer(usbp, p_buffer, length, audio_notify_mute_cb);
                return true;
            } else if (ctrl == UAC_FU_VOLUME_CONTROL) {
                p_control->channel = channel;
                usbSetupTransfer(usbp, p_buffer, length, audio_notify_volume_cb);
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
    // Only requests to audio control iface are supported.
    if (iface == AUDIO_CONTROL_INTERFACE) {
        /* Feature unit */
        if (entity == AUDIO_FUNCTION_UNIT_ID) {
            return audio_handle_request_cb(usbp, req, (wValue >> 8) & 0xFF, wValue & 0xFF, length);
        }
    }
    return false;
}

/**
 * @brief The start-playback callback.
 * @details Is called, when the audio endpoint goes into its operational alternate mode (actual music playback begins).
 * It broadcasts the \a AUDIO_EVENT_START_STREAMING event.
 *
 * @param usbp The pointer to the USB driver structure.
 */
void start_playback_cb(USBDriver *usbp) {
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_streaming_enabled) {
        audio_init_playback(p_playback);
        p_playback->b_streaming_enabled = true;

        // Distribute event, and prepare USB audio data reception, and feedback endpoint transmission.
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_START_STREAMING);

        // Feedback yet unknown, transmit empty packet.
        usbStartTransmitI(usbp, AUDIO_FEEDBACK_ENDPOINT, NULL, 0);

        // Initial audio data reception.
        usbStartReceiveI(usbp, AUDIO_PLAYBACK_ENDPOINT, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset],
                         AUDIO_MAX_PACKET_SIZE);

        chSysUnlockFromISR();
    }
}

/**
 * @brief The stop-playback callback.
 * @details Is called on USB reset, or when the audio endpoint goes into its zero bandwidth alternate mode. It
 * broadcasts the \a AUDIO_EVENT_STOP_STREAMING and \a AUDIO_EVENT_STOP_PLAYBACK events.
 *
 * @param usbp The pointer to the USB driver structure.
 */
void audio_stop_playback_cb(USBDriver *usbp) {
    (void)usbp;
    volatile struct audio_playback *p_playback = &g_audio_context.playback;

    if (p_playback->b_streaming_enabled) {
        p_playback->b_output_enabled    = false;
        p_playback->b_streaming_enabled = false;

        // Distribute events.
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_STOP_PLAYBACK);
        chEvtBroadcastFlagsI(&g_audio_event_source, AUDIO_EVENT_STOP_STREAMING);
        chSysUnlockFromISR();
    }
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
                    start_playback_cb(usbp);
                } else {
                    audio_stop_playback_cb(usbp);
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
    chEvtRegisterMask(&g_audio_event_source, &audio_event_listener, AUDIO_EVENT);

    // Enable the feedback counter timer TIM2 peripheral clock (no low-power mode).
    rccEnableTIM2(false);

    while (true) {
        chEvtWaitOne(AUDIO_EVENT);
        eventflags_t event_flags = chEvtGetAndClearFlags(&audio_event_listener);

        // The SOF capture (for feedback calculation) must be started after/stopped before the I2S peripheral - its
        // master clock output is required for counting the SOF interval.

        if (event_flags & AUDIO_EVENT_START_PLAYBACK) {
            i2sStartExchange(&I2S_DRIVER);
            audio_start_sof_capture();
        }

        if (event_flags & AUDIO_EVENT_STOP_PLAYBACK) {
            audio_stop_sof_capture();
            i2sStopExchange(&I2S_DRIVER);
        }

        if (event_flags & AUDIO_EVENT_START_STREAMING) {
            i2sStart(&I2S_DRIVER, &g_i2s_config);

            // Fire a volume event, for setting the correct DAC volumes on playback.
            chEvtBroadcastFlags(&g_audio_event_source, AUDIO_EVENT_VOLUME);
        }

        if (event_flags & AUDIO_EVENT_STOP_STREAMING) {
            i2sStop(&I2S_DRIVER);
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
