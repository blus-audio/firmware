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

BaseSequentialStream *stream = (BaseSequentialStream *)&SD2;

I2CConfig tas2780_i2c_config = {
    .op_mode = OPMODE_I2C,
    .clock_speed = 100000u,
    .duty_cycle = STD_DUTY_CYCLE};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *
get_descriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang)
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

/*
 * Device states.
 */
static audio_state_t audio;

/* I2S buffer */
static uint16_t dac_buffer[AUDIO_BUFFER_SIZE + AUDIO_MAX_PACKET_SIZE];
/* I2S buffer write address */
static uint16_t dac_buffer_wr_addr = 0;

/*
 * Framerate feedback stuff.
 */
static volatile uint32_t feedback_value = 0;
static volatile uint32_t sof_last_counter = 0;
static volatile uint32_t sof_delta = 0;
static volatile bool sof_first = true;
static volatile int sof_delta_count = 0;
static uint8_t sof_feedback_data[3];

/*
 * TIM2 interrupt handler
 * TIM2 clocked by I2S master clock (PC7 (I2S_MCK) connected to PA5 (TIM2_ETR)).
 * TIM2 triggers on USB start of frame.
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  uint32_t value = TIM2->CNT;
  uint32_t sr = TIM2->SR;
  TIM2->SR = 0;

  if (sr & TIM_SR_TIF)
  {
    chSysLockFromISR();
    if (!sof_first)
    {
      if (sof_last_counter < value)
        sof_delta += value - sof_last_counter;
      else
        sof_delta += UINT32_MAX - sof_last_counter + value;

      /* Feedback value calculated every 32 SOFs = 32 ms */
      if (sof_delta_count == 31)
      {
        /* 10.14 format F = 256fs (8 bit), 32 frames (5 bits) = 19.13 */
        feedback_value = ((sof_delta << 9) / 1000) & 0xFFFFFFUL;
        sof_feedback_data[0] = feedback_value & 0xFF;
        sof_feedback_data[1] = (feedback_value >> 8) & 0xFF;
        sof_feedback_data[2] = (feedback_value >> 16) & 0xFF;
        sof_delta = 0;
        sof_delta_count = 0;
        audio.sof_feedback_valid = true;
      }
      else
      {
        sof_delta_count++;
      }
    }
    sof_first = false;
    sof_last_counter = value;
    chSysUnlockFromISR();
  }

  OSAL_IRQ_EPILOGUE();
}

/*
 * Start frame interval measure.
 */
void start_sof_capture(void)
{
  /* Reset TIM2 */
  rccResetTIM2();
  nvicEnableVector(STM32_TIM2_NUMBER, STM32_IRQ_TIM2_PRIORITY);

  chSysLock();
  sof_last_counter = 0;
  sof_delta = 0;
  sof_first = true;
  sof_delta_count = 0;
  audio.sof_feedback_valid = false;

  /* Enable TIM2 counting */
  TIM2->CR1 = TIM_CR1_CEN;
  /* Timer clock = ETR pin, slave mode, trigger on ITR1 */
  TIM2->SMCR = TIM_SMCR_TS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1; // TIM_SMCR_ECE | TIM_SMCR_TS_0 ;
  /* TIM2 enable interrupt */
  TIM2->DIER = TIM_DIER_TIE;
  /* Remap ITR1 to USB_FS SOF signal */
  TIM2->OR = TIM_OR_ITR1_RMP_1;
  chSysUnlock();
}

/*
 * Stop frame interval measure.
 */
void stop_sof_capture(void)
{
  chSysLock();
  nvicDisableVector(STM32_TIM2_NUMBER);
  TIM2->CR1 = 0;
  audio.sof_feedback_valid = false;
  chSysUnlock();
}

/*
 * Feedback transmitted (or dropped) in current frame.
 */
void audio_feedback(USBDriver *usbp, usbep_t ep)
{
  if (audio.playback)
  {
    /* Transmit feedback data */
    chSysLockFromISR();
    if (audio.sof_feedback_valid)
      usbStartTransmitI(usbp, ep, sof_feedback_data, 3);
    else
      usbStartTransmitI(usbp, ep, NULL, 0);
    chSysUnlockFromISR();
  }
}

/*
 * Data received (or not) in current frame.
 */
void audio_received(USBDriver *usbp, usbep_t ep)
{
  if (audio.playback)
  {
    uint16_t new_addr = dac_buffer_wr_addr + usbGetReceiveTransactionSizeX(usbp, ep) / 2;

    /* Handle buffer wrap */
    if (new_addr >= AUDIO_BUFFER_SIZE)
    {
      for (int i = AUDIO_BUFFER_SIZE; i < new_addr; i++)
        dac_buffer[i - AUDIO_BUFFER_SIZE] = dac_buffer[i];
      new_addr -= AUDIO_BUFFER_SIZE;
    }
    dac_buffer_wr_addr = new_addr;

    chSysLockFromISR();
    usbStartReceiveI(usbp, ep, (uint8_t *)&dac_buffer[dac_buffer_wr_addr], AUDIO_MAX_PACKET_SIZE);
    chSysUnlockFromISR();
  }
}

/*
 * Part (half) of I2S buffer transmitted.
 */
static void i2s_callback(I2SDriver *i2sp)
{
  (void)i2sp;

  // if (audio.playback)
  // {
  //   /* Simple buffer overflow/underflow handling */
  //   if ((off == 0) && (dac_buffer_wr_addr < n))
  //   {
  //     audio.buffer_errors++;
  //     dac_buffer_wr_addr = n + AUDIO_PACKET_SIZE / 2;
  //   }
  //   else if ((off != 0) && (dac_buffer_wr_addr > off))
  //   {
  //     audio.buffer_errors++;
  //     dac_buffer_wr_addr = AUDIO_PACKET_SIZE / 2;
  //   }
  // }
}

/*
 * EP1 states.
 */
static USBOutEndpointState ep1outstate;

/*
 * EP1 initialization structure (IN & OUT).
 */
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_ISOC,
    NULL,
    NULL,
    audio_received,
    0x0000,
    AUDIO_MAX_PACKET_SIZE,
    NULL,
    &ep1outstate,
    1,
    NULL};

/*
 * EP2 states.
 */
static USBInEndpointState ep2instate;

/*
 * EP2 initialization structure (IN & OUT).
 */
static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_ISOC,
    NULL,
    audio_feedback,
    NULL,
    0x0004,
    0x0000,
    &ep2instate,
    NULL,
    1,
    NULL};

/*
 * Temporary buffer for control data.
 */
static uint8_t control_data[8];
static uint8_t control_channel;

/*
 * Volume data received.
 */
static void notify_volume(USBDriver *usbp)
{
  (void)usbp;

  if (control_channel == 0xff)
  {
    memcpy(audio.volume, control_data + sizeof(int16_t), 2 * sizeof(int16_t));
  }
  else
  {
    memcpy(&audio.volume[control_channel - 1], control_data, sizeof(int16_t));
  }
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&audio.audio_events, AUDIO_EVENT_VOLUME);
  chSysUnlockFromISR();
}

/*
 * Mute data received.
 */
static void notify_mute(USBDriver *usbp)
{
  (void)usbp;

  if (control_channel == 0xff)
  {
    audio.mute[0] = control_data[1];
    audio.mute[1] = control_data[2];
  }
  else
  {
    audio.mute[control_channel - 1] = control_data[0];
  }
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&audio.audio_events, AUDIO_EVENT_MUTE);
  chSysUnlockFromISR();
}

/*
 * Handles requests for audio function unit (volume & mute).
 */
bool audio_volume_control(USBDriver *usbp, uint8_t req, uint8_t ctrl,
                          uint8_t channel, uint16_t length)
{
  switch (req)
  {
  case UAC_REQ_SET_MAX:
  case UAC_REQ_SET_MIN:
  case UAC_REQ_SET_RES:
    if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      usbSetupTransfer(usbp, control_data, length, NULL);
      return true;
    }
    break;

  case UAC_REQ_GET_MAX:
    if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      for (int i = 0; i < length; i++)
        ((int16_t *)control_data)[i] = 0;
      usbSetupTransfer(usbp, control_data, length, NULL);
      return true;
    }
    break;

  case UAC_REQ_GET_MIN:
    if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      for (int i = 0; i < length; i++)
        ((int16_t *)control_data)[i] = -96 * 256;
      usbSetupTransfer(usbp, control_data, length, NULL);
      return true;
    }
    break;

  case UAC_REQ_GET_RES:
    if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      for (int i = 0; i < length; i++)
        ((int16_t *)control_data)[i] = 128;
      usbSetupTransfer(usbp, control_data, length, NULL);
      return true;
    }
    break;

  case UAC_REQ_GET_CUR:
    if (ctrl == UAC_FU_MUTE_CONTROL)
    {
      if (channel == 0xff)
      {
        uint8_t value[3] = {0, audio.mute[0], audio.mute[1]};
        memcpy(control_data, value, sizeof(value));
        usbSetupTransfer(usbp, control_data, length, NULL);
      }
      else
      {
        memcpy(control_data, &audio.mute[channel - 1], sizeof(uint8_t));
        usbSetupTransfer(usbp, control_data, length, NULL);
      }
      return true;
    }
    else if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      if (channel == 0xff)
      {
        int16_t value[3] = {0, audio.volume[0], audio.volume[1]};
        memcpy(control_data, value, sizeof(value));
        usbSetupTransfer(usbp, control_data, length, NULL);
      }
      else
      {
        memcpy(control_data, &audio.volume[channel - 1], sizeof(int16_t));
        usbSetupTransfer(usbp, control_data, length, NULL);
      }
      return true;
    }
    break;

  case UAC_REQ_SET_CUR:
    if (ctrl == UAC_FU_MUTE_CONTROL)
    {
      control_channel = channel;
      usbSetupTransfer(usbp, control_data, length, notify_mute);
      return true;
    }
    else if (ctrl == UAC_FU_VOLUME_CONTROL)
    {
      control_channel = channel;
      usbSetupTransfer(usbp, control_data, length, notify_volume);
      return true;
    }
    break;

  default:
    return false;
  }

  return false;
}

/*
 * Handles UAC-specific controls.
 */
bool audio_control(USBDriver *usbp, uint8_t iface, uint8_t entity, uint8_t req,
                   uint16_t wValue, uint16_t length)
{
  /* Only requests to audio control iface are supported */
  if (iface == AUDIO_CONTROL_INTERFACE)
  {
    /* Feature unit */
    if (entity == AUDIO_FUNCTION_UNIT_ID)
    {
      return audio_volume_control(usbp, req, (wValue >> 8) & 0xFF,
                                  wValue & 0xFF, length);
    }
  }
  return false;
}

/*
 * Starts USB transfers, and notifies control thread.
 */
void start_playback(USBDriver *usbp)
{
  if (!audio.playback)
  {
    audio.playback = true;
    audio.buffer_errors = 0;
    dac_buffer_wr_addr = AUDIO_PACKET_SIZE / 2;
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&audio.audio_events, AUDIO_EVENT_PLAYBACK);
    usbStartTransmitI(usbp, AUDIO_FEEDBACK_ENDPOINT, NULL, 0);
    usbStartReceiveI(usbp, AUDIO_PLAYBACK_ENDPOINT, (uint8_t *)&dac_buffer[dac_buffer_wr_addr], AUDIO_MAX_PACKET_SIZE);
    chSysUnlockFromISR();
  }
}

/*
 * Stops playback, and notifies control thread.
 */
void stop_playback(USBDriver *usbp)
{
  (void)usbp;

  if (audio.playback)
  {
    audio.playback = false;
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&audio.audio_events, AUDIO_EVENT_PLAYBACK);
    chSysUnlockFromISR();
  }
}

/*
 * Handles SETUP requests.
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
          start_playback(usbp);
        }
        else
        {
          stop_playback(usbp);
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
      return audio_control(usbp, usbp->setup[4], usbp->setup[5], usbp->setup[1],
                           (usbp->setup[3] << 8) | (usbp->setup[2]),
                           (usbp->setup[7] << 8) | (usbp->setup[6]));
    case USB_RTYPE_RECIPIENT_ENDPOINT:
    default:
      return false;
    }
  }
  return false;
}

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event)
{
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&audio.audio_events, AUDIO_EVENT_USB_STATE);
  chSysUnlockFromISR();

  switch (event)
  {
  case USB_EVENT_RESET:
    stop_playback(usbp);
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, AUDIO_PLAYBACK_ENDPOINT, &ep1config);
    usbInitEndpointI(usbp, AUDIO_FEEDBACK_ENDPOINT, &ep2config);
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

/*
 * USB driver configuration.
 */
static const USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    audio_requests_hook,
    NULL,
};

/*
 * I2S configuration.
 */
static const I2SConfig i2scfg = {
    .tx_buffer = (const uint8_t *)dac_buffer,
    .rx_buffer = NULL,
    .size = AUDIO_BUFFER_SIZE,
    .end_cb = i2s_callback,
    .i2spr = (SPI_I2SPR_I2SDIV & 20)};

/*
 * Audio demo config
 */
static const audio_config_t audiocfg = {
    &USBD1,
    &I2SD3};

/*
 * Initial init.
 */
void audioObjectInit(audio_state_t *ap)
{
  chEvtObjectInit(&ap->audio_events);

  ap->config = NULL;
  ap->mute[0] = 0;
  ap->mute[1] = 0;
  ap->playback = false;
  ap->sof_feedback_valid = false;
  ap->buffer_errors = 0;
  ap->volume[0] = 0;
  ap->volume[1] = 0;
}

/*
 * Startup.
 */
void audioStart(audio_state_t *ap, const audio_config_t *cp)
{
  ap->config = cp;
}

/*
 * Application entry point.
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

  sdStart(&SD2, NULL);

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

  // tas_2780_setup(&tas2780_a_context);
  // tas_2780_setup(&tas2780_b_context);
  // tas_2780_setup(&tas2780_c_context);
  // tas_2780_setup(&tas2780_d_context);

  i2cStop(&I2CD1);

  audioObjectInit(&audio);
  audioStart(&audio, &audiocfg);

  /*
   * Registers this thread for audio events.
   */
  static event_listener_t listener;
  chEvtRegisterMask(&audio.audio_events, &listener, AUDIO_EVENT);

  /*
   * Enables TIM2
   */
  rccEnableTIM2(FALSE);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1500);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

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
      volatile uint32_t s = USBD1.state;
    }

    /*
     * Audio playback started (stopped).
     * Enable (Disable) external DAC and I2S bus.
     * Enable (Disable) SOF capture.
     */
    if (evt & AUDIO_EVENT_PLAYBACK)
    {
      if (audio.playback)
      {
        i2sStart(&I2SD3, &i2scfg);
        i2sStartExchange(&I2SD3);
        start_sof_capture();
      }
      else
      {
        stop_sof_capture();
        i2sStopExchange(&I2SD3);
        i2sStop(&I2SD3);
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
