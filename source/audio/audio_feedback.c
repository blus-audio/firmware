// Copyright 2023 elagil

/**
 * @file
 * @brief   Audio feedback module.
 * @details Contains feedback calculation and reporting functionality.
 *
 * @addtogroup audio
 * @{
 */

#include "audio_feedback.h"

#include <string.h>

#include "audio_playback.h"

/**
 * @brief The amount by which the feedback value is adjusted, when the buffer fill size is critical.
 *
 * @details This translates to a difference in reported sample rate of
 * \a AUDIO_FEEDBACK_CORRECTION_OFFSET * 2**14 / 1000
 *
 * For example, 16 represents a reported offset of 1.024 Hz - a mild adjustment.
 */
#define AUDIO_FEEDBACK_CORRECTION_OFFSET (64u)

/**
 * @brief The minimum supported exponent of the period between feedback packets.
 */
#define AUDIO_FEEDBACK_MIN_PERIOD_EXPONENT 0x01u

/**
 * @brief The maximum supported exponent of the period between feedback packets.
 */
#define AUDIO_FEEDBACK_MAX_PERIOD_EXPONENT 0x06u

/**
 * @brief The bit-shift that needs to be applied to the timer counter values for calculating the feedback value.
 */
#define AUDIO_FEEDBACK_SHIFT (AUDIO_FEEDBACK_MAX_PERIOD_EXPONENT - AUDIO_FEEDBACK_PERIOD_EXPONENT)

/**
 * @brief The audio feedback period duration in ms.
 */
#define AUDIO_FEEDBACK_PERIOD_MS (1 << AUDIO_FEEDBACK_PERIOD_EXPONENT)

/**
 * @brief The size of the packets for the feedback endpoint.
 * @details These are three bytes long, in 10.14 binary format. The format represents a number in kHz, so that a 1 at a
 * bit shift of 14 is 1 kHz.
 */
#define AUDIO_FEEDBACK_BUFFER_SIZE 3u

/**
 * @brief Maximum deviation from ideal buffer size in packets.
 * @details The number of audio packets that the buffer fill size is allowed to deviate from its ideal fill size, before
 * the forceful feedback correction is applied.
 */
#define AUDIO_FEEDBACK_MAX_PACKET_DEVIATION_COUNT 1u

// Sanity checks.
#if AUDIO_FEEDBACK_PERIOD_EXPONENT < AUDIO_FEEDBACK_MIN_PERIOD_EXPONENT
#error "Unsupported feedback period exponent - too small."
#endif

#if AUDIO_FEEDBACK_PERIOD_EXPONENT > AUDIO_FEEDBACK_MAX_PERIOD_EXPONENT
#error "Unsupported feedback period exponent - too large."
#endif

// The buffer holds N packets. The target buffer level in packets is: N/2 + 1/2.
// With a packet deviation count of M, there is a space of
//   N - (N/2 + 1/2 + M)
// between the upper allowed fill level, and the end of the buffer. Thus,
//   -2M + N - 1 > 0 => 1 + 2M < N
// must be fulfilled.
#if ((2u * AUDIO_FEEDBACK_MAX_PACKET_DEVIATION_COUNT) + 1u) >= AUDIO_BUFFER_PACKET_COUNT
#error "Feedback deviation correction packet count too large - decrease it, or increase audio buffer packet count."
#endif

/**
 * @brief The state of the feedback correction.
 */
enum AUDIO_FEEDBACK_CORRECTION_STATE {
    AUDIO_FEEDBACK_CORRECTION_STATE_OFF,       ///< No feedback correction active.
    AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE,  ///< Decrease the feedback value in case of over-filled audio
                                               ///< buffer.
    AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE   ///< Increase the feedback value in case of under-filled audio
                                               ///< buffer.
};

/**
 * @brief A structure that holds the state of the audio sample rate feedback.
 */
struct audio_feedback {
    enum AUDIO_FEEDBACK_CORRECTION_STATE correction_state;   ///< The state of forced feedback value correction.
    bool                                 b_is_first_sof;     ///< If true, the first SOF packet is yet to be received.
    bool                                 b_is_valid;         ///< Is true, if the feedback value is valid.
    size_t                               sof_package_count;  ///< Counts the SOF packages since the last
                                                             ///< feedback value update.
    uint32_t value;                                          ///< The current feedback value.
    uint32_t last_counter_value;                             ///< The counter value at the time of the
                                                             ///< previous SOF interrupt.
} g_feedback;

/**
 * @brief Get the current feedback value.
 *
 * @return uint32_t The feedback value.
 */
uint32_t audio_feedback_get_value(void) { return g_feedback.value; }

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
    // Calculate thresholds for the buffer fill size.
    const size_t AUDIO_BUFFER_MAX_FILL_SIZE =
        audio_playback_get_buffer_target_fill_size() +
        (AUDIO_FEEDBACK_MAX_PACKET_DEVIATION_COUNT * audio_playback_get_packet_size());
    const size_t AUDIO_BUFFER_MIN_FILL_SIZE =
        audio_playback_get_buffer_target_fill_size() -
        (AUDIO_FEEDBACK_MAX_PACKET_DEVIATION_COUNT * audio_playback_get_packet_size());

    switch (g_feedback.correction_state) {
        case AUDIO_FEEDBACK_CORRECTION_STATE_OFF:
            if (audio_playback_get_buffer_fill_size() > AUDIO_BUFFER_MAX_FILL_SIZE) {
                // The fill size is too high, compensate by means of lower feedback value.
                g_feedback.correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE;
            } else if (audio_playback_get_buffer_fill_size() < AUDIO_BUFFER_MIN_FILL_SIZE) {
                // The fill size is too low, compensate by means of higher feedback value.
                g_feedback.correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE;
            }
            /* fall through */

        case AUDIO_FEEDBACK_CORRECTION_STATE_DECREASE:
            if (audio_playback_get_buffer_fill_size() <= audio_playback_get_buffer_target_fill_size()) {
                // Switch off correction, when reaching target fill size.
                g_feedback.correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
            } else {
                g_feedback.value -= AUDIO_FEEDBACK_CORRECTION_OFFSET;
            }
            break;

        case AUDIO_FEEDBACK_CORRECTION_STATE_INCREASE:
            if (audio_playback_get_buffer_fill_size() >= audio_playback_get_buffer_target_fill_size()) {
                // Switch off correction, when reaching target fill size.
                g_feedback.correction_state = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
            }
            { g_feedback.value += AUDIO_FEEDBACK_CORRECTION_OFFSET; }
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
 * USB audio feedback calculation requires the device to provide its sample rate in relation to the SOF period.
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER) {
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

    if (g_feedback.b_is_first_sof) {
        // On the first SOF signal, the feedback cannot be calculated yet. Only record the timer state.
        g_feedback.last_counter_value = counter_value;
        g_feedback.b_is_first_sof     = false;
        OSAL_IRQ_EPILOGUE();
        return;
    }

    // Normal playback operation below.

    g_feedback.sof_package_count++;
    if (g_feedback.sof_package_count == AUDIO_FEEDBACK_PERIOD_MS) {
        // The feedback value is measured and reported every \a AUDIO_FEEDBACK_PERIOD_MS .
        // The timer that counts its cycles during that period is clocked by the I2S master clock, which (on this
        // hardware) runs at 256 times the audio sample rate. Considering an audio sample rate of 48 kHz, this results
        // in a timer clock of 12.288 MHz.
        //
        // Therefore, the timer counts a total amount of N clock cycles
        //   N = fs * 256 * \a AUDIO_FEEDBACK_PERIOD_MS ,
        //
        // and thus the measured sample rate is
        //   fs = N / 256 / \a AUDIO_FEEDBACK_PERIOD_MS .
        //
        // The feedback endpoint shall report the device sample rate in units of kHz in a 10.14 binary (fixpoint)
        // format. As an example, a sample rate of 48 kHz would be represented as 48 << 14 = 786432. In practice, the
        // measured sample rate will likely deviate slightly from the nominal value.
        //
        // In mathematical terms, the reported number M must be
        //   M = 2^14 * fs / 1000 .
        //
        // Calculating m from n (inserting for fs) yields
        //   M = 2^14 * n / 256 / \a AUDIO_FEEDBACK_PERIOD_MS / 1000 .
        //
        // As a numerical example, consider \a AUDIO_FEEDBACK_PERIOD_MS = 64 ms. In this special case,
        //   2^14 / 256 / 64e-3 / 1000 = 1.0 ,
        //
        // and the timer value can directly be used as the feedback value. If, for example, \a
        // AUDIO_FEEDBACK_PERIOD_MS was halved to 32 ms, the counter value would have to be doubled, in order
        // to achieve the same feedback value.
        //
        // In this function, this is accomplished with a bitshift operation by \a AUDIO_FEEDBACK_SHIFT . The
        // value of \a AUDIO_FEEDBACK_SHIFT is zero for a feedback period of 64 ms, and increases by one for
        // every halving of the feedback period. Feedback periods longer than 64 ms are not supported.
        //
        // See the general USB 2.0 specification for more details (5.12.4.2, p. 75) on the format and calculation of the
        // feedback value.
        g_feedback.value = subtract_circular_unsigned(counter_value, g_feedback.last_counter_value, UINT32_MAX)
                           << AUDIO_FEEDBACK_SHIFT;

        g_feedback.last_counter_value = counter_value;

        // If there is too much discrepancy between the target sample buffer fill size, and the actual fill size, this
        // must be compensated manually.
        audio_feedback_correct();

        g_feedback.sof_package_count = 0u;
        g_feedback.b_is_valid        = true;
    }

    OSAL_IRQ_EPILOGUE();
}

/**
 * @brief Set up the timer peripheral for counting USB start of frame (SOF) periods.
 * @note Only start after the I2S peripheral is running. Its MCLK output clocks this timer.
 */
void audio_feedback_start_sof_capture(void) {
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

    audio_feedback_init();
    chSysUnlock();
}

/**
 * @brief Stop the timer peripheral for counting USB start of frame (SOF) periods.
 */
void audio_feedback_stop_sof_capture(void) {
    chSysLock();
    nvicDisableVector(STM32_TIM2_NUMBER);
    TIM2->CR1 = 0;

    audio_feedback_init();
    chSysUnlock();
}

/**
 * @brief Joint callback for when feedback was transmitted, or failed, in the current frame.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param ep The endpoint, for which the feedback was called.
 */
void audio_feedback_cb(USBDriver *usbp, usbep_t ep) {
    if (!audio_playback_is_streaming_enabled()) {
        // Feedback is only active while streaming.
        return;
    }

    chSysLockFromISR();

    if (g_feedback.b_is_valid) {
        static uint8_t feedback_buffer[AUDIO_FEEDBACK_BUFFER_SIZE];
        value_to_byte_array(feedback_buffer, g_feedback.value, AUDIO_FEEDBACK_BUFFER_SIZE);

        usbStartTransmitI(usbp, ep, feedback_buffer, AUDIO_FEEDBACK_BUFFER_SIZE);
    } else {
        // Transmit an empty packet.
        usbStartTransmitI(usbp, ep, NULL, 0);
    }

    chSysUnlockFromISR();
}

/**
 * @brief Initialize the audio feedback structure.
 */
void audio_feedback_init(void) {
    g_feedback.correction_state   = AUDIO_FEEDBACK_CORRECTION_STATE_OFF;
    g_feedback.b_is_first_sof     = true;
    g_feedback.b_is_valid         = false;
    g_feedback.sof_package_count  = 0u;
    g_feedback.last_counter_value = 0u;
    g_feedback.value              = 0u;
}

/**
 * @}
 */
