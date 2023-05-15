// Copyright 2023 elagil

/**
 * @file
 * @brief   Audio request module.
 * @details Contains functionality for handling UAC 1.0 audio requests.
 *
 * @addtogroup audio
 * @{
 */

#ifndef SOURCE_AUDIO_AUDIO_REQUEST_H_
#define SOURCE_AUDIO_AUDIO_REQUEST_H_

#include "audio_common.h"

bool     audio_request_is_channel_muted(enum AUDIO_COMMON_CHANNEL audio_channel);
int16_t  audio_request_get_channel_volume(enum AUDIO_COMMON_CHANNEL audio_channel);
uint32_t audio_request_get_sample_rate_hz(void);
bool     audio_request_hook_cb(USBDriver *usbp);

void audio_request_init(mailbox_t *p_mailbox);

#endif  // SOURCE_AUDIO_AUDIO_REQUEST_H_

/**
 * @}
 */
