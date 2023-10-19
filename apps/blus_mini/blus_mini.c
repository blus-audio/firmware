// Copyright 2023 elagil

/**
 * @file
 * @brief   The user application module for the blus mini hardware.
 * @details Contains the user application functions that provide
 * - volume potentiometer ADC readout,
 * - amplifier/DAC controls, and
 * - a reporting thread.
 *
 * @addtogroup main
 * @{
 */
#include "audio.h"
#include "ch.h"
#include "chprintf.h"
#include "print.h"
#include "tas2780.h"

/**
 * @brief Settings structure for the TAS2780 I2C driver.
 */
static const I2CConfig g_tas2780_i2c_config = {
    .op_mode = OPMODE_I2C, .clock_speed = 100000u, .duty_cycle = STD_DUTY_CYCLE};

/**
 * @brief The volume potentiometer ADC sample (12 bit long).
 */
static adcsample_t g_adc_sample;

/**
 * @brief Starts continuous sampling of the volume potentiometer ADC.
 * @note The result is currently unused.
 */
static void app_start_volume_adc(void) {
    // ADC conversion group:
    // - continuous conversion
    // - 480 samples conversion time
    // - Channel 9
    static const ADCConversionGroup adc_conversion_group = {.circular     = TRUE,
                                                            .num_channels = 1,
                                                            .end_cb       = NULL,
                                                            .error_cb     = NULL,
                                                            .cr1          = 0u,
                                                            .cr2          = ADC_CR2_SWSTART,
                                                            .smpr1        = 0u,
                                                            .smpr2        = ADC_SMPR2_SMP_AN9(ADC_SAMPLE_480),
                                                            .htr          = 0u,
                                                            .ltr          = 0u,
                                                            .sqr1         = 0u,
                                                            .sqr2         = 0u,
                                                            .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN9)};

    // Start continuous conversion.
    adcStart(&ADCD1, NULL);
    adcStartConversion(&ADCD1, &adc_conversion_group, &g_adc_sample, 1u);
}

static THD_WORKING_AREA(wa_housekeeping_thread, 128);

/**
 * @brief A housekeeping thread that checks amplifier states, and outputs status information via UART (if enabled).
 */
static THD_FUNCTION(housekeeping_thread, arg) {
    (void)arg;
    chRegSetThreadName("reporting");

    while (true) {
        tas2780_acquire_lock();
        tas2780_ensure_active_all();
        tas2780_release_lock();

        chSysLock();
        size_t buffer_fill_size = audio_playback_get_buffer_fill_size();
        size_t feedback_value   = audio_feedback_get_value();
        size_t playback_state   = audio_playback_get_state();
        chSysUnlock();

// Disable extended reporting statistics.
#if 0
        uint8_t noise_gate_mask = tas2780_get_noise_gate_mask_all();
        PRINTF("Noise gate: %u\n", noise_gate_mask);

        PRINTF("Potentiometer: %u\n",
                 g_adc_sample >> 4);  // Convert to an 8 bit number.

        PRINTF("Volume: %li / %li dB\n",
                 (audio_request_get_channel_volume(AUDIO_COMMON_CHANNEL_LEFT) >> 8),
                 (audio_request_get_channel_volume(AUDIO_COMMON_CHANNEL_RIGHT) >> 8));
#endif

        PRINTF("Buffer: %u / %u (fb %u) @ state %u\n", buffer_fill_size, AUDIO_MAX_BUFFER_SIZE, feedback_value,
               playback_state);

        chThdSleepMilliseconds(500);
    }
}

/**
 * @brief Perform user application setup.
 */
void app_setup(void) {
    // Setup amplifiers.
    i2cStart(&I2CD1, &g_tas2780_i2c_config);

    tas2780_init();

    tas2780_acquire_lock();
    tas2780_setup_all();
    tas2780_release_lock();

    // Begin reading volume potentiometer ADC.
    app_start_volume_adc();

    // Create housekeeping thread for regular tasks.
    chThdCreateStatic(wa_housekeeping_thread, sizeof(wa_housekeeping_thread), NORMALPRIO, housekeeping_thread, NULL);
}

/**
 * @brief Joint handling of volume and mute controls.
 */
static void app_set_volume_and_mute_state(void) {
    chSysLock();
    bool b_left_channel_is_muted  = audio_request_is_channel_muted(AUDIO_COMMON_CHANNEL_LEFT);
    bool b_right_channel_is_muted = audio_request_is_channel_muted(AUDIO_COMMON_CHANNEL_RIGHT);

    int16_t left_channel_volume  = audio_request_get_channel_volume(AUDIO_COMMON_CHANNEL_LEFT);
    int16_t right_channel_volume = audio_request_get_channel_volume(AUDIO_COMMON_CHANNEL_RIGHT);
    chSysUnlock();

    tas2780_acquire_lock();
    if (b_left_channel_is_muted) {
        tas2780_set_volume_all(TAS2780_VOLUME_MUTE, TAS2780_CHANNEL_LEFT);
    } else {
        tas2780_set_volume_all(left_channel_volume, TAS2780_CHANNEL_LEFT);
    }

    if (b_right_channel_is_muted) {
        tas2780_set_volume_all(TAS2780_VOLUME_MUTE, TAS2780_CHANNEL_RIGHT);
    } else {
        tas2780_set_volume_all(right_channel_volume, TAS2780_CHANNEL_RIGHT);
    }
    tas2780_release_lock();
}

/**
 * @brief Reset the TAS2780 volume levels to maximum.
 */
void app_reset_volume(void) { tas2780_set_volume_all(TAS2780_VOLUME_MAX, TAS2780_CHANNEL_BOTH); }

/**
 * @brief Set a new volume on the TAS2780 amplifiers. Calls \a app_set_volume_and_mute_state() internally.
 */
void app_set_volume(void) { app_set_volume_and_mute_state(); }

/**
 * @brief Set new mute states on the TAS2780 amplifiers. Calls \a app_set_volume_and_mute_state() internally.
 */
void app_set_mute_state(void) { app_set_volume_and_mute_state(); }

/**
 * @}
 */
