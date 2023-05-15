// Copyright 2023 elagil

/**
 * @file
 * @brief   Audio playback module.
 *
 * @addtogroup audio
 * @{
 */

#ifndef SOURCE_AUDIO_AUDIO_PLAYBACK_H_
#define SOURCE_AUDIO_AUDIO_PLAYBACK_H_

#include "audio_common.h"

uint8_t *audio_playback_get_buffer(void);
size_t   audio_playback_get_buffer_size(void);
size_t   audio_playback_get_buffer_fill_size(void);
size_t   audio_playback_get_buffer_target_fill_size(void);
size_t   audio_playback_get_packet_size(void);

bool audio_playback_is_enabled(void);
bool audio_playback_is_streaming_enabled(void);

void audio_playback_start_streaming(USBDriver *usbp);
void audio_playback_stop_streaming(USBDriver *usbp);

void audio_playback_received_cb(USBDriver *usbp, usbep_t ep);

void audio_playback_set_sample_rate(uint32_t sample_rate_hz);
void audio_playback_init(mailbox_t *p_mailbox, bool b_streaming_enabled);

#endif  // SOURCE_AUDIO_AUDIO_PLAYBACK_H_
