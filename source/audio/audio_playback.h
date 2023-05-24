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

/**
 * @brief The state of the audio playback.
 */
enum audio_playback_state {
    AUDIO_PLAYBACK_STATE_IDLE,       ///< The playback module is idle.
    AUDIO_PLAYBACK_STATE_STREAMING,  ///< The playback module is connected for USB audio streaming.
    AUDIO_PLAYBACK_STATE_PLAYBACK    ///< Audio is being played back via I2S.
};

uint8_t *audio_playback_get_buffer(void);
size_t   audio_playback_get_buffer_size(void);
size_t   audio_playback_get_buffer_fill_size(void);
size_t   audio_playback_get_buffer_target_fill_size(void);
size_t   audio_playback_get_packet_size(void);

void                      audio_playback_start_streaming(USBDriver *usbp);
void                      audio_playback_stop_streaming(USBDriver *usbp);
enum audio_playback_state audio_playback_get_state(void);

void audio_playback_received_cb(USBDriver *usbp, usbep_t ep);

void audio_playback_set_sample_rate(uint32_t sample_rate_hz);
void audio_playback_init(mailbox_t *p_mailbox);

#endif  // SOURCE_AUDIO_AUDIO_PLAYBACK_H_
