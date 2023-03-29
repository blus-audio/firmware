#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "audio.h"
#include "usb.h"
#include "tas2780.h"

/**
 * @brief Settings structure for the TAS2780 I2C driver.
 */
static const I2CConfig tas2780_i2c_config = {
    .op_mode = OPMODE_I2C,
    .clock_speed = 100000u,
    .duty_cycle = STD_DUTY_CYCLE};

static BaseSequentialStream *stream = (BaseSequentialStream *)&SD2;
static THD_WORKING_AREA(wa_reporting_thread, 128);

/**
 * @brief Settings structure for the USB driver.
 */
static const USBConfig usbcfg = {
    usb_event_cb,
    usb_get_descriptor_cb,
    audio_requests_hook_cb,
    NULL,
};

/**
 * @brief A reporting thread that outputs status information via UART.
 */
static THD_FUNCTION(reporting_thread, arg)
{
  (void)arg;
  struct audio_context *p_audio_context = audio_get_context();

  sdStart(&SD2, NULL);
  chRegSetThreadName("reporting");

  static const ADCConversionGroup adc_conversion_group = {
      FALSE,
      1, // ADC_GRP1_NUM_CHANNELS,
      NULL,
      NULL,            // adcerrorcallback,
      0,               /* CR1 */
      ADC_CR2_SWSTART, /* CR2 */
      ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
      0, /* SMPR2 */
      0, /* HTR */
      0, /* LTR */
      0, /* SQR1 */
      0, /* SQR2 */
      ADC_SQR3_SQ1_N(ADC_CHANNEL_IN9)};

  adcStart(&ADCD1, NULL);

  while (true)
  {
    adcsample_t adc_sample;
    adcConvert(&ADCD1, &adc_conversion_group, &adc_sample, 1);
    chThdSleepMilliseconds(1000);

    chprintf(stream, "Pot: %lu\n", adc_sample);
    chprintf(stream, "Volume: %li / %li dB\n", (p_audio_context->control.channel_volume_levels_8q8_db[0] >> 8), (p_audio_context->control.channel_volume_levels_8q8_db[1] >> 8));
    chprintf(stream, "Feedback value: %lu (%lu errors)\n", p_audio_context->feedback.value, p_audio_context->diagnostics.error_count);
    chprintf(stream, "Audio buffer use: %lu / %lu (margins %lu / %lu)\n", AUDIO_BUFFER_SAMPLE_COUNT - p_audio_context->diagnostics.sample_distance, AUDIO_BUFFER_SAMPLE_COUNT, AUDIO_BUFFER_MIN_FILL_LEVEL, AUDIO_BUFFER_MAX_FILL_LEVEL);
  }
}

/**
 * @brief Application entry point.
 *
 * @return int The return code.
 */
int main(void)
{
  halInit();
  chSysInit();

  struct audio_context *p_audio_context = audio_get_context();

  // Initialize audio module.
  audio_init_context(p_audio_context);

  // Create reporting thread.
  chThdCreateStatic(wa_reporting_thread, sizeof(wa_reporting_thread), NORMALPRIO, reporting_thread, NULL);

  // Setup amplifiers.
  i2cStart(&I2CD1, &tas2780_i2c_config);
  tas2780_setup_all();

  // Registers this thread for audio events.
  static event_listener_t audio_event_listener;
  chEvtRegisterMask(&p_audio_context->audio_events, &audio_event_listener, AUDIO_EVENT);

  // Enable the feedback counter timer TIM2 peripheral clock (no low-power mode).
  rccEnableTIM2(false);

  // Activate USB connectivity.
  usbDisconnectBus(&USB_DRIVER);
  usbStart(&USB_DRIVER, &usbcfg);
  usbConnectBus(&USB_DRIVER);

  // Wait for an audio event.
  while (true)
  {
    chEvtWaitOne(AUDIO_EVENT);
    eventflags_t event_flags = chEvtGetAndClearFlags(&audio_event_listener);

    if (event_flags & AUDIO_EVENT_USB_STATE)
    {
    }

    if (event_flags & AUDIO_EVENT_PLAYBACK)
    {
      if (p_audio_context->playback.b_enabled)
      {
        i2sStart(&I2S_DRIVER, p_audio_context->config.p_i2s_config);
        audio_init_feedback(&p_audio_context->feedback);
        audio_start_sof_capture();

        // Fire a volume event, for setting the correct amplifier volumes on playback.
        chEvtBroadcastFlags(&p_audio_context->audio_events, AUDIO_EVENT_VOLUME);
      }
      else
      {
        audio_stop_sof_capture();
        i2sStopExchange(&I2S_DRIVER);
        i2sStop(&I2S_DRIVER);

        // Restore volume levels to maximum, when playback ends.
        tas2780_set_volume_all(TAS2780_VOLUME_MAX, TAS2780_CHANNEL_BOTH);
      }
    }

    // Joint handling of volume and mute controls.
    if ((event_flags & AUDIO_EVENT_MUTE) || (event_flags & AUDIO_EVENT_VOLUME))
    {
      if (p_audio_context->control.b_channel_mute_states[0])
      {
        tas2780_set_volume_all(TAS2780_VOLUME_MUTE, TAS2780_CHANNEL_LEFT);
      }
      else
      {
        tas2780_set_volume_all(p_audio_context->control.channel_volume_levels_8q8_db[0], TAS2780_CHANNEL_LEFT);
      }

      if (p_audio_context->control.b_channel_mute_states[1])
      {
        tas2780_set_volume_all(TAS2780_VOLUME_MUTE, TAS2780_CHANNEL_RIGHT);
      }
      else
      {
        tas2780_set_volume_all(p_audio_context->control.channel_volume_levels_8q8_db[1], TAS2780_CHANNEL_RIGHT);
      }
    }
  }
}
