// Copyright 2023 elagil

/**
 * @file
 * @brief   Main audio module headers.
 *
 * @addtogroup audio
 * @{
 */

#ifndef SOURCE_AUDIO_AUDIO_H_
#define SOURCE_AUDIO_AUDIO_H_

#include "audio_common.h"
#include "audio_feedback.h"
#include "audio_playback.h"
#include "audio_request.h"

void audio_setup(mailbox_t *p_mailbox);
void audio_reset(USBDriver *p_usb);

#endif  // SOURCE_AUDIO_AUDIO_H_

/**
 * @}
 */
