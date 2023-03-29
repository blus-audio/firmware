#include <string.h>
#include "audio.h"

/**
 * @brief The global audio context.
 * @details Can be shared with other modules by means of \a audio_get_context().
 */
static struct audio_context g_audio_context;

/**
 * @brief Settings structure for the I2S driver.
 * @details Enables the master clock output for timer capture.
 */
static const I2SConfig g_i2s_config = {
    .tx_buffer = (const uint8_t *)g_audio_context.playback.buffer,
    .rx_buffer = NULL,
    .size = AUDIO_BUFFER_SAMPLE_COUNT,
    .end_cb = NULL,
    .i2spr = SPI_I2SPR_MCKOE | (SPI_I2SPR_I2SDIV & 6)};

struct audio_context *audio_get_context(void)
{
    return &g_audio_context;
}

/**
 * @brief Initialize the audio config structure.
 *
 * @param p_config The pointer to the structure to initialize.
 */
void audio_init_config(struct audio_config *p_config)
{
    p_config->p_i2s_config = &g_i2s_config;
}

/**
 * @brief Initialize the audio diagnostics structure.
 *
 * @param p_diagnostics The pointer to the structure to initialize.
 */
void audio_init_diagnostics(struct audio_diagnostics *p_diagnostics)
{
    p_diagnostics->sample_distance = 0u;
    p_diagnostics->error_count = 0u;
}

/**
 * @brief Initialize the audio feedback structure.
 *
 * @param p_feedback The pointer to the structure to initialize.
 */
void audio_init_feedback(struct audio_feedback *p_feedback)
{
    p_feedback->b_is_first_sof = true;
    p_feedback->b_is_valid = false;
    p_feedback->sof_package_count = 0u;
    p_feedback->value = 0u;
    p_feedback->previous_counter_value = 0;
    p_feedback->timer_count_difference = 0;
}

/**
 * @brief Initialize the audio playback structure.
 *
 * @param p_playback The pointer to the structure to initialize.
 */
void audio_init_playback(struct audio_playback *p_playback)
{
    p_playback->buffer_write_offset = 0;
    p_playback->b_output_enabled = false;
    p_playback->b_enabled = false;
}

/**
 * @brief Initialize the audio control structure.
 *
 * @param p_control The pointer to the structure to initialize.
 */
void audio_init_control(struct audio_control *p_control)
{
    p_control->channel = 0u;
    p_control->local_volume_8q8_db = 0;

    for (size_t channel_index = 0; channel_index < AUDIO_CHANNEL_COUNT; channel_index++)
    {
        p_control->b_channel_mute_states[channel_index] = false;
        p_control->channel_volume_levels_8q8_db[channel_index] = 0;
    }
}

/**
 * @brief Initialize an audio context, and all its contained structures.
 *
 * @param p_context The pointer to the context to initialize.
 * @param p_usb_driver The USB driver to attach.
 * @param p_i2s_driver The I2S driver to attach.
 */
void audio_init_context(struct audio_context *p_context)
{

    chEvtObjectInit(&p_context->audio_events);
    audio_init_feedback(&p_context->feedback);
    audio_init_playback(&p_context->playback);
    audio_init_control(&p_context->control);
    audio_init_diagnostics(&p_context->diagnostics);
    audio_init_config(&p_context->config);
}

/**
 * @brief The interrupt handler for timer TIM2. Called upon reception of a USB start of frame (SOF) signal.
 * @details The timer is used for counting the interval between SOF signals, which arrive roughly with a frequency
 * of 1 kHz. Since the timer is clocked by the I2S master clock, it counts the SOF period in terms of the I2S frequency.
 * In other words, it relates the SOF period to the output sample rate, which is required for feedback calculation.
 *
 * USB audio feedback calculation requires the device to provide its sampling rate in relation to the SOF period.
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    struct audio_feedback *p_feedback = &g_audio_context.feedback;
    struct audio_diagnostics *p_diagnostics = &g_audio_context.diagnostics;
    struct audio_playback *p_playback = &g_audio_context.playback;
    uint32_t counter_value = TIM2->CNT;

    // Reset any timer interrupt flags.
    uint32_t timer_status_register = TIM2->SR;
    TIM2->SR = 0;

    if (!(timer_status_register & TIM_SR_TIF))
    {
        // Trigger interrupt flag was not set.
        OSAL_IRQ_EPILOGUE();
        return;
    }

    if (!p_playback->b_output_enabled)
    {
        // Start measurement and reporting after having reached the target buffer fill level.
        // This ensures that the target buffer fill level is reached exactly -
        // until this point, the USB packets should have nominal size.
        OSAL_IRQ_EPILOGUE();
        return;
    }

    if (p_feedback->b_is_first_sof)
    {
        // On the first SOF signal, the feedback cannot be calculated yet.
        // Only record the timer state.
        p_feedback->b_is_first_sof = false;
        p_feedback->previous_counter_value = counter_value;
        OSAL_IRQ_EPILOGUE();
        return;
    }

    // Normal playback operation.

    // Feedback value is calculated every 64 SOF interrupts => every 64 ms.
    p_feedback->sof_package_count++;
    if (p_feedback->sof_package_count == 64)
    {
        // Handle timer wrap-around.
        if (p_feedback->previous_counter_value < counter_value)
        {
            p_feedback->timer_count_difference += counter_value - p_feedback->previous_counter_value;
        }
        else
        {
            p_feedback->timer_count_difference += UINT32_MAX - p_feedback->previous_counter_value + counter_value;
        }
        p_feedback->previous_counter_value = counter_value;

        // Conveniently, the timer count difference at 64 ms count periods matches the required feedback format.
        // The feedback endpoint requires the device sample rate in kHz in a 10.14 binary (fixpoint) format.

        // - The master clock runs 256 times as fast as the reference clock.
        // - The conversion of fs / kHz to fs / Hz adds a factor of 1000
        // - The counting period is 64 ms long.
        // Thus
        //   fs / kHz * 2**14 = fs * 256 * 64e-3 s
        //
        // where the left side is the expected format, and the right side the result of this counting function.
        p_feedback->value = p_feedback->timer_count_difference;

        // If there is too much discrepancy between the target sample buffer fill level, and the
        // actual fill level, this must be compensated manually.
        //
        // The general USB 2.0 specification states (5.12.4.2, p. 75):
        // "It is possible that the source will deliver one too many or one too few samples over a long period due to errors
        // or accumulated inaccuracies in measuring Ff. The sink must have sufficient buffer capability to accommodate this.
        // When the sink recognizes this condition, it should adjust the reported Ff value to correct it.
        // This may also be necessary to compensate for relative clock drifts.""
        if (p_diagnostics->sample_distance > AUDIO_BUFFER_MAX_FILL_LEVEL)
        {
            p_feedback->value -= AUDIO_FEEDBACK_CORRECTION_OFFSET;
            p_diagnostics->error_count++;
        }
        else if (p_diagnostics->sample_distance < AUDIO_BUFFER_MIN_FILL_LEVEL)
        {
            p_feedback->value += AUDIO_FEEDBACK_CORRECTION_OFFSET;
            p_diagnostics->error_count++;
        }

        // Translate the feedback value to a buffer of three bytes.
        p_feedback->buffer[0] = p_feedback->value & 0xFF;
        p_feedback->buffer[1] = (p_feedback->value >> 8) & 0xFF;
        p_feedback->buffer[2] = (p_feedback->value >> 16) & 0xFF;

        p_feedback->timer_count_difference = 0;
        p_feedback->sof_package_count = 0;
        p_feedback->b_is_valid = true;
    }

    OSAL_IRQ_EPILOGUE();
}
/**
 * @brief Set up the timer peripheral for counting USB start of frame (SOF) periods.
 */
void audio_start_sof_capture(void)
{
    // Reset TIM2 instance.
    rccResetTIM2();
    nvicEnableVector(STM32_TIM2_NUMBER, STM32_IRQ_TIM2_PRIORITY);

    chSysLock();
    // Enable TIM2 counter.
    TIM2->CR1 = TIM_CR1_CEN;
    // - The timer clock source is the ETR pin .
    // - Enable slave mode.
    // - Trigger on ITR1 (the SOF signal).
    TIM2->SMCR = TIM_SMCR_ECE | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
    // Enable TIM2 interrupt.
    TIM2->DIER = TIM_DIER_TIE;
    // Remap ITR1 to the USB_FS SOF signal.
    TIM2->OR = TIM_OR_ITR1_RMP_1;
    chSysUnlock();
}

/**
 * @brief Stop the timer peripheral for counting USB start of frame (SOF) periods.
 */
void audio_stop_sof_capture(void)
{
    chSysLock();
    audio_init_feedback(&g_audio_context.feedback);

    nvicDisableVector(STM32_TIM2_NUMBER);
    TIM2->CR1 = 0;
    chSysUnlock();
}

/**
 * @brief Joint callback for when feedback was transmitted, or failed, in the current frame.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param ep The endpoint, for which the feedback was called.
 */
void audio_feedback_cb(USBDriver *usbp, usbep_t ep)
{
    if (!g_audio_context.playback.b_enabled)
    {
        return;
    }

    chSysLockFromISR();

    if (g_audio_context.feedback.b_is_valid)
    {
        usbStartTransmitI(usbp, ep, g_audio_context.feedback.buffer, AUDIO_FEEDBACK_BUFFER_SIZE);
    }
    else
    {
        // Transmit an empty packet.
        usbStartTransmitI(usbp, ep, NULL, 0);
    }

    chSysUnlockFromISR();
}

/**
 * @brief Update the write pointer, taking into account wrap-around of the circular buffer.
 *
 * @param received_sample_count The number of newly received samples.
 */
void audio_update_write_offset(uint16_t received_sample_count)
{
    struct audio_playback *p_playback = &g_audio_context.playback;
    uint16_t new_buffer_write_offset = p_playback->buffer_write_offset + received_sample_count;

    if (new_buffer_write_offset >= AUDIO_BUFFER_SAMPLE_COUNT)
    {
        for (size_t sample_index = AUDIO_BUFFER_SAMPLE_COUNT; sample_index < new_buffer_write_offset; sample_index++)
        {
            p_playback->buffer[sample_index - AUDIO_BUFFER_SAMPLE_COUNT] = p_playback->buffer[sample_index];
        }
        new_buffer_write_offset -= AUDIO_BUFFER_SAMPLE_COUNT;
    }
    p_playback->buffer_write_offset = new_buffer_write_offset;
}

/**
 * @brief Joint callback for when audio data was received from the host, or the reception failed, in the current frame.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param ep The endpoint, for which the feedback was called.
 */
void audio_received_cb(USBDriver *usbp, usbep_t ep)
{
    struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_enabled)
    {
        // Disregard packets, when playback is disabled.
        return;
    }

    uint16_t received_sample_count = usbGetReceiveTransactionSizeX(usbp, ep) / AUDIO_SAMPLE_SIZE;
    audio_update_write_offset(received_sample_count);

    uint16_t read_offset = (uint16_t)AUDIO_BUFFER_SAMPLE_COUNT - (uint16_t)(I2S_DRIVER.dmatx->stream->NDTR);
    uint16_t write_offset = g_audio_context.playback.buffer_write_offset;

    // Calculate the distance between the DMA read pointer, and the USB driver's write pointer in the playback buffer.
    uint16_t sample_distance = 0;
    if (read_offset > write_offset)
    {
        sample_distance += AUDIO_BUFFER_SAMPLE_COUNT;
    }
    sample_distance += (write_offset - read_offset);
    g_audio_context.diagnostics.sample_distance = sample_distance;

    chSysLockFromISR();

    // Start playback on the I2S device, when the playback buffer is at least at the target fill level.
    if (!p_playback->b_output_enabled && (p_playback->buffer_write_offset >= AUDIO_BUFFER_TARGET_FILL_LEVEL))
    {
        p_playback->b_output_enabled = true;
        i2sStartExchangeI(&I2S_DRIVER);
    }
    usbStartReceiveI(usbp, ep, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset], AUDIO_MAX_PACKET_SIZE);

    chSysUnlockFromISR();
}

/**
 * @brief Volume levels where changed. Updates the audio context.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_notify_volume_cb(USBDriver *usbp)
{
    (void)usbp;
    struct audio_control *p_control = &g_audio_context.control;

    if (p_control->channel == 0xFF)
    {
        memcpy(p_control->channel_volume_levels_8q8_db, p_control->buffer + sizeof(int16_t), 2 * sizeof(int16_t));
    }
    else
    {
        memcpy(&p_control->channel_volume_levels_8q8_db[p_control->channel - 1], p_control->buffer, sizeof(int16_t));
    }
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&g_audio_context.audio_events, AUDIO_EVENT_VOLUME);
    chSysUnlockFromISR();
}

/**
 * @brief Channel mute information was changed. Updates the audio context.
 *
 * @param usbp A pointer to the USB driver structure.
 */
static void audio_notify_mute_cb(USBDriver *usbp)
{
    (void)usbp;
    struct audio_control *p_control = &g_audio_context.control;

    if (p_control->channel == 0xff)
    {
        p_control->b_channel_mute_states[0] = p_control->buffer[1];
        p_control->b_channel_mute_states[1] = p_control->buffer[2];
    }
    else
    {
        p_control->b_channel_mute_states[p_control->channel - 1] = p_control->buffer[0];
    }
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&g_audio_context.audio_events, AUDIO_EVENT_MUTE);
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
bool audio_handle_request_cb(USBDriver *usbp, uint8_t req, uint8_t ctrl,
                             uint8_t channel, uint16_t length)
{
    struct audio_control *p_control = &g_audio_context.control;

    switch (req)
    {
    case UAC_REQ_SET_MAX:
    case UAC_REQ_SET_MIN:
    case UAC_REQ_SET_RES:
        if (ctrl == UAC_FU_VOLUME_CONTROL)
        {
            usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            return true;
        }
        break;

    case UAC_REQ_GET_MAX:
        if (ctrl == UAC_FU_VOLUME_CONTROL)
        {
            for (size_t i = 0; i < length; i++)
                ((int16_t *)p_control->buffer)[i] = 0;
            usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            return true;
        }
        break;

    case UAC_REQ_GET_MIN:
        if (ctrl == UAC_FU_VOLUME_CONTROL)
        {
            for (size_t i = 0; i < length; i++)
                ((int16_t *)p_control->buffer)[i] = -100 * 256;
            usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            return true;
        }
        break;

    case UAC_REQ_GET_RES:
        if (ctrl == UAC_FU_VOLUME_CONTROL)
        {
            for (size_t i = 0; i < length; i++)
                ((int16_t *)p_control->buffer)[i] = 128;
            usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            return true;
        }
        break;

    case UAC_REQ_GET_CUR:
        if (ctrl == UAC_FU_MUTE_CONTROL)
        {
            if (channel == 0xff)
            {
                uint8_t value[3] = {0, p_control->b_channel_mute_states[0], p_control->b_channel_mute_states[1]};
                memcpy(p_control->buffer, value, sizeof(value));
                usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            }
            else
            {
                memcpy(p_control->buffer, &p_control->b_channel_mute_states[channel - 1], sizeof(uint8_t));
                usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            }
            return true;
        }
        else if (ctrl == UAC_FU_VOLUME_CONTROL)
        {
            if (channel == 0xff)
            {
                int16_t value[3] = {0, p_control->channel_volume_levels_8q8_db[0], p_control->channel_volume_levels_8q8_db[1]};
                memcpy(p_control->buffer, value, sizeof(value));
                usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            }
            else
            {
                memcpy(p_control->buffer, &p_control->channel_volume_levels_8q8_db[channel - 1], sizeof(int16_t));
                usbSetupTransfer(usbp, p_control->buffer, length, NULL);
            }
            return true;
        }
        break;

    case UAC_REQ_SET_CUR:
        if (ctrl == UAC_FU_MUTE_CONTROL)
        {
            p_control->channel = channel;
            usbSetupTransfer(usbp, p_control->buffer, length, audio_notify_mute_cb);
            return true;
        }
        else if (ctrl == UAC_FU_VOLUME_CONTROL)
        {
            p_control->channel = channel;
            usbSetupTransfer(usbp, p_control->buffer, length, audio_notify_volume_cb);
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
bool audio_control_cb(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req,
                      uint16_t wValue, uint16_t length)
{
    // Only requests to audio control iface are supported.
    if (iface == AUDIO_CONTROL_INTERFACE)
    {
        /* Feature unit */
        if (entity == AUDIO_FUNCTION_UNIT_ID)
        {
            return audio_handle_request_cb(usbp, req, (wValue >> 8) & 0xFF,
                                           wValue & 0xFF, length);
        }
    }
    return false;
}

/**
 * @brief The start-playback callback.
 * @details Is called, when the audio endpoint goes into its operational alternate mode (actual music playback begins).
 * It broadcasts the \a AUDIO_EVENT_PLAYBACK event.
 *
 * @param usbp The pointer to the USB driver structure.
 */
void start_playback_cb(USBDriver *usbp)
{
    struct audio_playback *p_playback = &g_audio_context.playback;

    if (!p_playback->b_enabled)
    {
        audio_init_playback(p_playback);
        audio_toggle_playback(p_playback);

        // Distribute event, and prepare USB audio data reception, and feedback endpoint transmission.
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&g_audio_context.audio_events, AUDIO_EVENT_PLAYBACK);

        // Feedback yet unknown, transmit empty packet.
        usbStartTransmitI(usbp, AUDIO_FEEDBACK_ENDPOINT, NULL, 0);

        // Initial audio data reception.
        usbStartReceiveI(usbp, AUDIO_PLAYBACK_ENDPOINT, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset], AUDIO_MAX_PACKET_SIZE);

        chSysUnlockFromISR();
    }
}

/**
 * @brief The stop-playback callback.
 * @details Is called on USB reset, or when the audio endpoint goes into its zero bandwidth alternate mode.
 * It broadcasts the \a AUDIO_EVENT_PLAYBACK event.
 *
 * @param usbp The pointer to the USB driver structure.
 */
void audio_stop_playback_cb(USBDriver *usbp)
{
    (void)usbp;
    struct audio_playback *p_playback = &g_audio_context.playback;

    if (p_playback->b_enabled)
    {
        audio_toggle_playback(p_playback);

        // Distribute event.
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&g_audio_context.audio_events, AUDIO_EVENT_PLAYBACK);
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
bool audio_requests_hook_cb(USBDriver *usbp)
{
    if ((usbp->setup[0] & (USB_RTYPE_TYPE_MASK | USB_RTYPE_RECIPIENT_MASK)) ==
        (USB_RTYPE_TYPE_STD | USB_RTYPE_RECIPIENT_INTERFACE))
    {
        if (usbp->setup[1] == USB_REQ_SET_INTERFACE)
        {
            /* Switch between empty interface and normal one. */
            if (((usbp->setup[5] << 8) | usbp->setup[4]) == AUDIO_STREAMING_INTERFACE)
            {
                if (((usbp->setup[3] << 8) | usbp->setup[2]) == 1)
                {
                    start_playback_cb(usbp);
                }
                else
                {
                    audio_stop_playback_cb(usbp);
                }
                usbSetupTransfer(usbp, NULL, 0, NULL);
                return true;
            }
        }
        return false;
    }
    else if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS)
    {
        switch (usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK)
        {
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
