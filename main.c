/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include <string.h>
#include "hal.h"
#include "audio.h"
#include "usb_desc.h"
#include "tas2780.h"
#include "chprintf.h"

#define I2S_DRIVER (I2SD3)
#define USB_DRIVER (USBD1)

static const USBDescriptor *
get_descriptor_cb(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang);
bool audio_requests_hook(USBDriver *usbp);
void audio_received_cb(USBDriver *usbp, usbep_t ep);
void audio_feedback_cb(USBDriver *usbp, usbep_t ep);
static void usb_event(USBDriver *usbp, usbevent_t event);

static struct audio_context g_audio_context;
static BaseSequentialStream *stream = (BaseSequentialStream *)&SD2;

/**
 * @brief Settings structure for the USB driver.
 */
static const USBConfig usbcfg = {
    usb_event,
    get_descriptor_cb,
    audio_requests_hook,
    NULL,
};

/**
 * @brief A structure that holds the state of endpoint 1.
 */
static USBOutEndpointState endpoint1_state;

/**
 * @brief The configuration structure for endpoint 1.
 */
static const USBEndpointConfig endpoint1_config = {
    USB_EP_MODE_TYPE_ISOC,
    NULL,
    NULL,
    audio_received_cb,
    0x0000,
    AUDIO_MAX_PACKET_SIZE,
    NULL,
    &endpoint1_state,
    1,
    NULL};

/**
 * @brief A structure that holds the state of endpoint 2.
 */
static USBInEndpointState endpoint2_state;

/**
 * @brief The configuration structure for endpoint 2.
 */
static const USBEndpointConfig endpoint2_config = {
    USB_EP_MODE_TYPE_ISOC,
    NULL,
    audio_feedback_cb,
    NULL,
    0x0004,
    0x0000,
    &endpoint2_state,
    NULL,
    1,
    NULL};

/**
 * @brief Settings structure for the TAS2780 I2C driver.
 */
static const I2CConfig tas2780_i2c_config = {
    .op_mode = OPMODE_I2C,
    .clock_speed = 100000u,
    .duty_cycle = STD_DUTY_CYCLE};

/**
 * @brief Settings structure for the I2S driver.
 * @details Enables the master clock output for timer capture.
 */
static const I2SConfig i2s_config = {
    .tx_buffer = (const uint8_t *)g_audio_context.playback.buffer,
    .rx_buffer = NULL,
    .size = AUDIO_BUFFER_SAMPLE_COUNT,
    .end_cb = NULL,
    .i2spr = SPI_I2SPR_MCKOE | (SPI_I2SPR_I2SDIV & 6)};

volatile size_t g_sample_distance = 0;

/**
 * @brief Callback function that returns the requested USB descriptor.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param dtype The descriptor type that is requested.
 * @param dindex The index of data within the descriptor to fetch.
 * @param lang Currently unused - language specifier.
 * @return const USBDescriptor* The requested descriptor.
 */
static const USBDescriptor *
get_descriptor_cb(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang)
{

  (void)usbp;
  (void)lang;
  switch (dtype)
  {
  case USB_DESCRIPTOR_DEVICE:
    return &audio_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &audio_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &audio_strings[dindex];
  }
  return NULL;
}

static THD_WORKING_AREA(wa_reporting_thread, 128);
static THD_FUNCTION(reporting_thread, arg)
{
  (void)arg;

  sdStart(&SD2, NULL);
  chRegSetThreadName("reporting");

  while (true)
  {
    chprintf(stream, "Feedback value: %lu\n", g_audio_context.feedback.value);
    chprintf(stream, "Audio buffer use: %lu / %lu\n", AUDIO_BUFFER_SAMPLE_COUNT - g_sample_distance, AUDIO_BUFFER_SAMPLE_COUNT);
    chThdSleepMilliseconds(500);
  }
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
  uint32_t counter_value = TIM2->CNT;

  // Reset any timer interrupt flags.
  uint32_t timer_status_register = TIM2->SR;
  TIM2->SR = 0;

  if (timer_status_register & TIM_SR_TIF)
  {
    // Trigger interrupt flag was set.

    chSysLockFromISR();
    if (!(p_feedback->b_is_first_sof))
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

      // Feedback value is calculated every 64 SOF interrupts => every 64 ms.
      p_feedback->sof_package_count++;
      if (p_feedback->sof_package_count == 64)
      {
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

        // Translate feedback value to a buffer of three bytes.
        p_feedback->buffer[0] = p_feedback->value & 0xFF;
        p_feedback->buffer[1] = (p_feedback->value >> 8) & 0xFF;
        p_feedback->buffer[2] = (p_feedback->value >> 16) & 0xFF;

        p_feedback->timer_count_difference = 0;
        p_feedback->sof_package_count = 0;
        p_feedback->b_is_valid = true;
      }
    }

    p_feedback->b_is_first_sof = false;
    p_feedback->previous_counter_value = counter_value;
    chSysUnlockFromISR();
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief Set up the timer peripheral for counting USB start of frame (SOF) periods.
 */
void audio_start_sof_capture(void)
{
  /* Reset TIM2 */
  rccResetTIM2();
  nvicEnableVector(STM32_TIM2_NUMBER, STM32_IRQ_TIM2_PRIORITY);

  chSysLock();

  /* Enable TIM2 counting */
  TIM2->CR1 = TIM_CR1_CEN;
  /* Timer clock = ETR pin, slave mode, trigger on ITR1 */
  TIM2->SMCR = TIM_SMCR_ECE | TIM_SMCR_TS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
  /* TIM2 enable interrupt */
  TIM2->DIER = TIM_DIER_TIE;
  /* Remap ITR1 to USB_FS SOF signal */
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
  if (g_audio_context.playback.b_enabled)
  {
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

  if (p_playback->b_enabled)
  {
    uint16_t new_write_offset = p_playback->buffer_write_offset + usbGetReceiveTransactionSizeX(usbp, ep) / AUDIO_SAMPLE_SIZE;

    // Handle buffer wrap-around.
    if (new_write_offset >= AUDIO_BUFFER_SAMPLE_COUNT)
    {
      for (size_t sample_index = AUDIO_BUFFER_SAMPLE_COUNT; sample_index < new_write_offset; sample_index++)
      {
        p_playback->buffer[sample_index - AUDIO_BUFFER_SAMPLE_COUNT] = p_playback->buffer[sample_index];
      }
      new_write_offset -= AUDIO_BUFFER_SAMPLE_COUNT;
    }

    p_playback->buffer_write_offset = new_write_offset;

    chSysLockFromISR();

    // Start playback on the I2S device, when the playback buffer is at least half full.
    if (!p_playback->b_output_enabled && (p_playback->buffer_write_offset >= (AUDIO_BUFFER_SAMPLE_COUNT / 2)))
    {
      p_playback->b_output_enabled = true;
      i2sStartExchangeI(&I2S_DRIVER);
    }
    usbStartReceiveI(usbp, ep, (uint8_t *)&p_playback->buffer[p_playback->buffer_write_offset], AUDIO_MAX_PACKET_SIZE);

    // The current read index of the I2S DMA.
    size_t read_offset = AUDIO_BUFFER_SAMPLE_COUNT - I2S_DRIVER.dmatx->stream->NDTR;
    g_sample_distance = 0;

    // Calculate the distance between the DMA read pointer, and the USB driver's write pointer in the playback buffer.
    if (read_offset > p_playback->buffer_write_offset)
    {
      g_sample_distance += AUDIO_BUFFER_SAMPLE_COUNT;
    }
    g_sample_distance += (p_playback->buffer_write_offset - read_offset);

    chSysUnlockFromISR();
  }
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
    memcpy(p_control->channel_volume_levels, p_control->buffer + sizeof(int16_t), 2 * sizeof(int16_t));
  }
  else
  {
    memcpy(&p_control->channel_volume_levels[p_control->channel - 1], p_control->buffer, sizeof(int16_t));
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
      for (int i = 0; i < length; i++)
        ((int16_t *)p_control->buffer)[i] = 0;
      usbSetupTransfer(usbp, p_control->buffer, length, NULL);
      return true;
    }
    break;

  case UAC_REQ_GET_MIN:
    if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      for (int i = 0; i < length; i++)
        ((int16_t *)p_control->buffer)[i] = -96 * 256;
      usbSetupTransfer(usbp, p_control->buffer, length, NULL);
      return true;
    }
    break;

  case UAC_REQ_GET_RES:
    if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      for (int i = 0; i < length; i++)
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
        int16_t value[3] = {0, p_control->channel_volume_levels[0], p_control->channel_volume_levels[1]};
        memcpy(p_control->buffer, value, sizeof(value));
        usbSetupTransfer(usbp, p_control->buffer, length, NULL);
      }
      else
      {
        memcpy(p_control->buffer, &p_control->channel_volume_levels[channel - 1], sizeof(int16_t));
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
void stop_playback_cb(USBDriver *usbp)
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
bool audio_requests_hook(USBDriver *usbp)
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
          stop_playback_cb(usbp);
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

/**
 * @brief Handles global events that the USB driver triggers.
 *
 * @param usbp A pointer to the USB driver structure.
 * @param event The event that was triggered.
 */
static void usb_event(USBDriver *usbp, usbevent_t event)
{
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&g_audio_context.audio_events, AUDIO_EVENT_USB_STATE);
  chSysUnlockFromISR();

  switch (event)
  {
  case USB_EVENT_RESET:
    stop_playback_cb(usbp);
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, AUDIO_PLAYBACK_ENDPOINT, &endpoint1_config);
    usbInitEndpointI(usbp, AUDIO_FEEDBACK_ENDPOINT, &endpoint2_config);
    chSysUnlockFromISR();
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  case USB_EVENT_UNCONFIGURED:
    return;
  }
  return;
}

/**
 * @brief Application entry point.
 *
 * @return int The return code.
 */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  chThdCreateStatic(wa_reporting_thread, sizeof(wa_reporting_thread), NORMALPRIO, reporting_thread, NULL);

  i2cStart(&I2CD1, &tas2780_i2c_config);

  struct tas2780_context tas2780_a_context =
      {
          .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT,
          .device_address = TAS2780_DEVICE_ADDRESS_A,
      };

  struct tas2780_context tas2780_b_context =
      {
          .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT,
          .device_address = TAS2780_DEVICE_ADDRESS_B,
      };
  struct tas2780_context tas2780_c_context =
      {
          .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT,
          .device_address = TAS2780_DEVICE_ADDRESS_C,
      };
  struct tas2780_context tas2780_d_context =
      {
          .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT,
          .device_address = TAS2780_DEVICE_ADDRESS_D,
      };

  tas_2780_setup(&tas2780_a_context);
  tas_2780_setup(&tas2780_b_context);
  tas_2780_setup(&tas2780_c_context);
  tas_2780_setup(&tas2780_d_context);

  i2cStop(&I2CD1);

  audio_init_context(&g_audio_context, &USB_DRIVER, &I2S_DRIVER);

  /*
   * Registers this thread for audio events.
   */
  static event_listener_t listener;
  chEvtRegisterMask(&g_audio_context.audio_events, &listener, AUDIO_EVENT);

  /*
   * Enables TIM2
   */
  rccEnableTIM2(FALSE);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USB_DRIVER);
  chThdSleepMilliseconds(1500);
  usbStart(&USB_DRIVER, &usbcfg);
  usbConnectBus(&USB_DRIVER);

  /*
   * Normal main() thread activity, in this demo it controls external DAC
   */
  for (;;)
  {
    /*
     * Wait for audio event.
     */
    chEvtWaitOne(AUDIO_EVENT);
    eventflags_t evt = chEvtGetAndClearFlags(&listener);

    /*
     * USB state cahanged, switch LED3.
     */
    if (evt & AUDIO_EVENT_USB_STATE)
    {
    }

    /*
     * Audio playback started (stopped).
     * Enable (Disable) external DAC and I2S bus.
     * Enable (Disable) SOF capture.
     */
    if (evt & AUDIO_EVENT_PLAYBACK)
    {
      if (g_audio_context.playback.b_enabled)
      {
        audio_init_feedback(&g_audio_context.feedback);
        i2sStart(&I2S_DRIVER, &i2s_config);
        audio_start_sof_capture();
      }
      else
      {
        audio_stop_sof_capture();
        i2sStopExchange(&I2S_DRIVER);
        i2sStop(&I2S_DRIVER);
      }
    }

    /*
     * Set mute request received.
     */
    if (evt & AUDIO_EVENT_MUTE)
    {
    }

    /*
     * Set volume request received.
     */
    if (evt & AUDIO_EVENT_VOLUME)
    {
      // audio_dac_update_volume(&audio);
    }
  }
}
