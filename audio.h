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

#ifndef _AUDIO_H_
#define _AUDIO_H_

#include "hal.h"

/*
 * Supported control requests from USB Audio Class
 */
#define UAC_REQ_SET_CUR 0x01
#define UAC_REQ_SET_MIN 0x02
#define UAC_REQ_SET_MAX 0x03
#define UAC_REQ_SET_RES 0x04
#define UAC_REQ_GET_CUR 0x81
#define UAC_REQ_GET_MIN 0x82
#define UAC_REQ_GET_MAX 0x83
#define UAC_REQ_GET_RES 0x84

#define UAC_FU_MUTE_CONTROL 0x01
#define UAC_FU_VOLUME_CONTROL 0x02

/*
 * Audio playback events
 */
#define AUDIO_EVENT EVENT_MASK(0)
#define AUDIO_EVENT_PLAYBACK EVENT_MASK(1)
#define AUDIO_EVENT_MUTE EVENT_MASK(2)
#define AUDIO_EVENT_VOLUME EVENT_MASK(3)
#define AUDIO_EVENT_USB_STATE EVENT_MASK(4)

/*
 * Audio parameters.
 */
#define AUDIO_BIT_PER_BYTE 8U
#define AUDIO_SAMPLING_FREQUENCY 48000U
#define AUDIO_RESOLUTION_BIT 16U
#define AUDIO_SAMPLE_SIZE (AUDIO_RESOLUTION_BIT / AUDIO_BIT_PER_BYTE)
#define AUDIO_CHANNEL_COUNT 2U
#define AUDIO_SAMPLES_PER_FRAME (AUDIO_SAMPLING_FREQUENCY / 1000)
#define AUDIO_PACKET_SIZE (AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNEL_COUNT * AUDIO_RESOLUTION_BIT / AUDIO_BIT_PER_BYTE)
/* Because of samplerate feedback host can send more samples per frame */
#define AUDIO_BUFFER_LENGTH 4u
#define AUDIO_MAX_PACKET_SIZE (AUDIO_PACKET_SIZE + 4)
#define AUDIO_BUFFER_SAMPLE_COUNT (AUDIO_SAMPLES_PER_FRAME * AUDIO_CHANNEL_COUNT * AUDIO_BUFFER_LENGTH)
#define AUDIO_FEEDBACK_BUFFER_SIZE 3u

/*
 * USB Audio Class parameters
 */
#define AUDIO_PLAYBACK_ENDPOINT 0x01
#define AUDIO_FEEDBACK_ENDPOINT 0x02
#define AUDIO_CONTROL_INTERFACE 0
#define AUDIO_STREAMING_INTERFACE 1
#define AUDIO_INPUT_UNIT_ID 1
#define AUDIO_FUNCTION_UNIT_ID 2
#define AUDIO_OUTPUT_UNIT_ID 3

struct audio_config
{
  USBDriver *p_usb_driver;
  I2SDriver *p_i2s_driver;
};

struct audio_feedback
{
  volatile bool b_is_first_sof;
  volatile bool b_is_valid;
  volatile size_t sof_package_count;
  volatile uint32_t value;
  uint8_t buffer[AUDIO_FEEDBACK_BUFFER_SIZE];
  volatile uint32_t previous_counter_value;
  volatile uint32_t timer_count_difference;
};

struct audio_playback
{
  uint16_t buffer[AUDIO_BUFFER_SAMPLE_COUNT + AUDIO_MAX_PACKET_SIZE];
  uint16_t buffer_write_offset;
  bool b_enabled;
  bool b_output_enabled;
};

struct audio_control
{
  uint8_t buffer[8];                                  //<<< The provided control data.
  uint8_t channel;                                    //<<< The current channel mask.
  bool b_channel_mute_states[AUDIO_CHANNEL_COUNT];    //<<< Channel mute states.
  int16_t channel_volume_levels[AUDIO_CHANNEL_COUNT]; //<<< Channel volumes in 8.8 format (in dB).
};

struct audio_context
{
  struct audio_config config;
  struct audio_feedback feedback;
  struct audio_playback playback;
  struct audio_control control;

  event_source_t audio_events;
};

void audio_init_feedback(struct audio_feedback *p_feedback)
{
  p_feedback->b_is_first_sof = true;
  p_feedback->b_is_valid = false;
  p_feedback->sof_package_count = 0u;
  p_feedback->value = 0u;
  p_feedback->previous_counter_value = 0;
  p_feedback->timer_count_difference = 0;
}

void audio_toggle_playback(struct audio_playback *p_playback)
{
  p_playback->b_enabled = !p_playback->b_enabled;
}

void audio_init_playback(struct audio_playback *p_playback)
{
  p_playback->buffer_write_offset = 0;
  p_playback->b_output_enabled = false;
  p_playback->b_enabled = false;
}

void audio_init_control(struct audio_control *p_control)
{
  p_control->channel = 0u;

  for (size_t channel_index = 0; channel_index < AUDIO_CHANNEL_COUNT; channel_index++)
  {
    p_control->b_channel_mute_states[channel_index] = false;
    p_control->channel_volume_levels[channel_index] = 0;
  }
}

void audio_init_context(struct audio_context *p_context, USBDriver *p_usb_driver, I2SDriver *p_i2s_driver)
{
  p_context->config.p_i2s_driver = p_i2s_driver;
  p_context->config.p_usb_driver = p_usb_driver;

  chEvtObjectInit(&p_context->audio_events);
  audio_init_feedback(&p_context->feedback);
  audio_init_playback(&p_context->playback);
  audio_init_control(&p_context->control);
}

#endif // _AUDIO_H_
