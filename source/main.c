// Copyright 2023 elagil
#include <string.h>

#include "audio.h"
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "tas2780.h"
#include "usb.h"

/**
 * @brief Settings structure for the TAS2780 I2C driver.
 */
static const I2CConfig g_tas2780_i2c_config = {.op_mode     = OPMODE_I2C,
                                               .clock_speed = 100000u,
                                               .duty_cycle  = STD_DUTY_CYCLE};

static THD_WORKING_AREA(wa_reporting_thread, 128);

/**
 * @brief A reporting thread that outputs status information via UART.
 */
static THD_FUNCTION(reporting_thread, arg) {
    (void)arg;
    static BaseSequentialStream   *p_stream = (BaseSequentialStream *)&SD2;
    volatile struct audio_context *p_audio_context = audio_get_context();

    sdStart(&SD2, NULL);
    chRegSetThreadName("reporting");

    // ADC conversion group:
    // - continuous conversion
    // - 480 samples conversion time
    // - Channel 9
    static const ADCConversionGroup adc_conversion_group = {
        .circular     = TRUE,
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
    adcsample_t adc_sample;
    adcStart(&ADCD1, NULL);
    adcStartConversion(&ADCD1, &adc_conversion_group, &adc_sample, 1u);

    while (true) {
        tas2780_ensure_active_all();

        uint8_t noise_gate_mask = tas2780_noise_gate_mask_all();
        chprintf(p_stream, "Noise gate: %u\n", noise_gate_mask);

        chprintf(p_stream, "Potentiometer: %u\n",
                 adc_sample >> 4);  // Convert to an 8 bit number.
        chprintf(
            p_stream, "Volume: %li / %li dB\n",
            (p_audio_context->control.channel_volume_levels_8q8_db[0] >> 8),
            (p_audio_context->control.channel_volume_levels_8q8_db[1] >> 8));
        chprintf(p_stream, "Feedback value: %lu (%lu errors)\n",
                 p_audio_context->feedback.value,
                 p_audio_context->diagnostics.error_count);
        chprintf(p_stream, "Feedback correction mode: %u\n",
                 p_audio_context->feedback.correction);
        chprintf(p_stream,
                 "Audio buffer fill level: %lu / %lu (margins %lu / %lu)\n",
                 p_audio_context->playback.fill_level, AUDIO_BUFFER_LENGTH,
                 AUDIO_BUFFER_MIN_FILL_LEVEL, AUDIO_BUFFER_MAX_FILL_LEVEL);

        chThdSleepMilliseconds(1000);
    }
}

/**
 * @brief Application entry point.
 *
 * @return int The return code.
 */
int main(void) {
    halInit();
    chSysInit();

    event_source_t *p_audio_event_source           = audio_get_event_source();
    volatile struct audio_context *p_audio_context = audio_get_context();

    // Initialize audio module.
    audio_setup();

    // Initialize the USB module.
    usb_setup();

    // Setup amplifiers.
    i2cStart(&I2CD1, &g_tas2780_i2c_config);
    tas2780_setup_all();

    // Registers this thread for audio events.
    static event_listener_t audio_event_listener;
    chEvtRegisterMask(p_audio_event_source, &audio_event_listener, AUDIO_EVENT);

    // Create reporting thread.
    chThdCreateStatic(wa_reporting_thread, sizeof(wa_reporting_thread),
                      NORMALPRIO, reporting_thread, NULL);

    // Wait for an audio event.
    while (true) {
        chEvtWaitOne(AUDIO_EVENT);
        eventflags_t event_flags = chEvtGetAndClearFlags(&audio_event_listener);

        if (event_flags & AUDIO_EVENT_STOP_STREAMING) {
            // Restore volume levels to maximum, when playback ends.
            tas2780_set_volume_all(TAS2780_VOLUME_MAX, TAS2780_CHANNEL_BOTH);
        }

        // Joint handling of volume and mute controls.
        if ((event_flags & AUDIO_EVENT_MUTE) ||
            (event_flags & AUDIO_EVENT_VOLUME)) {
            if (p_audio_context->control.b_channel_mute_states[0]) {
                tas2780_set_volume_all(TAS2780_VOLUME_MUTE,
                                       TAS2780_CHANNEL_LEFT);
            } else {
                tas2780_set_volume_all(
                    p_audio_context->control.channel_volume_levels_8q8_db[0],
                    TAS2780_CHANNEL_LEFT);
            }

            if (p_audio_context->control.b_channel_mute_states[1]) {
                tas2780_set_volume_all(TAS2780_VOLUME_MUTE,
                                       TAS2780_CHANNEL_RIGHT);
            } else {
                tas2780_set_volume_all(
                    p_audio_context->control.channel_volume_levels_8q8_db[1],
                    TAS2780_CHANNEL_RIGHT);
            }
        }
    }
}
