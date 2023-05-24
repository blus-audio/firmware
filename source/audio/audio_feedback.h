// Copyright 2023 elagil

/**
 * @file
 * @brief   Audio feedback module headers.
 *
 * @addtogroup audio
 * @{
 */

#ifndef SOURCE_AUDIO_AUDIO_FEEDBACK_H_
#define SOURCE_AUDIO_AUDIO_FEEDBACK_H_

#include "audio_common.h"
#include "audio_playback.h"

uint32_t audio_feedback_get_value(void);

void audio_feedback_start_sof_capture(void);
void audio_feedback_stop_sof_capture(void);

void audio_feedback_cb(USBDriver *p_usb, usbep_t endpoint_identifier);

void audio_feedback_init(void);

#endif  // SOURCE_AUDIO_AUDIO_FEEDBACK_H_
