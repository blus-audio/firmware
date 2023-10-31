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

// Sanity checks.
#if AUDIO_FEEDBACK_PERIOD_EXPONENT < AUDIO_FEEDBACK_MIN_PERIOD_EXPONENT
#error "Unsupported feedback period exponent - too small."
#endif

#if AUDIO_FEEDBACK_PERIOD_EXPONENT > AUDIO_FEEDBACK_MAX_PERIOD_EXPONENT
#error "Unsupported feedback period exponent - too large."
#endif

/**
 * @brief The general state of audio feedback reporting.
 */
enum audio_feedback_state {
    AUDIO_FEEDBACK_STATE_IDLE,         ///< The feedback measurement is idle, and awaiting the first SOF packet.
    AUDIO_FEEDBACK_STATE_INITIALIZED,  ///< The first SOF packet was acquired, but the feedback value is not yet valid.
    AUDIO_FEEDBACK_STATE_ACTIVE        ///< The feedback value is valid, and feedback is provided to the host.
};

/**
 * @brief A structure that holds the state of the audio sample rate feedback.
 */
struct audio_feedback {
    size_t                    sof_package_count;   ///< Counts the SOF packages since the last feedback value update.
    uint32_t                  value;               ///< The current feedback value.
    uint32_t                  last_counter_value;  ///< The counter value at the time of the previous SOF interrupt.
    enum audio_feedback_state state;               ///< The general state of audio feedback reporting.
} g_feedback;

/**
 * @brief Get the current feedback value.
 *
 * @return uint32_t The feedback value.
 */
uint32_t audio_feedback_get_value(void) { return g_feedback.value; }

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

    if (g_feedback.state == AUDIO_FEEDBACK_STATE_IDLE) {
        // On the first SOF signal, the feedback cannot be calculated yet. Only record the timer state.
        g_feedback.last_counter_value = counter_value;
        g_feedback.state              = AUDIO_FEEDBACK_STATE_INITIALIZED;
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
        g_feedback.sof_package_count  = 0u;
        g_feedback.state              = AUDIO_FEEDBACK_STATE_ACTIVE;
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
 * @brief Joint callback for when feedback was transmitted, or its transmission failed.
 *
 * @param p_usb A pointer to the USB driver structure.
 * @param endpoint_identifier The endpoint, for which the feedback was called.
 */
void audio_feedback_cb(USBDriver *p_usb, usbep_t endpoint_identifier) {
    chSysLockFromISR();
    bool b_playback_idle = audio_playback_get_state() == AUDIO_PLAYBACK_STATE_IDLE;
    chSysUnlockFromISR();

    if (b_playback_idle) {
        // Feedback values can only be reported, when audio playback is not idle.
        return;
    }

    chSysLockFromISR();

    if (g_feedback.state == AUDIO_FEEDBACK_STATE_ACTIVE) {
        static uint8_t feedback_buffer[AUDIO_FEEDBACK_BUFFER_SIZE];
        value_to_byte_array(feedback_buffer, g_feedback.value, AUDIO_FEEDBACK_BUFFER_SIZE);

        usbStartTransmitI(p_usb, endpoint_identifier, feedback_buffer, AUDIO_FEEDBACK_BUFFER_SIZE);
    } else {
        // Transmit an empty packet.
        usbStartTransmitI(p_usb, endpoint_identifier, NULL, 0);
    }

    chSysUnlockFromISR();
}

/**
 * @brief Initialize the audio feedback structure.
 */
void audio_feedback_init(void) {
    chDbgCheckClassI();
    g_feedback.state              = AUDIO_FEEDBACK_STATE_IDLE;
    g_feedback.sof_package_count  = 0u;
    g_feedback.last_counter_value = 0u;
    g_feedback.value              = 0u;
}

/**
 * @}
 */
