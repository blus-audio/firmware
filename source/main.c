// Copyright 2023 elagil

/**
 * @file
 * @brief   The main module.
 * @details Contains the main application thread, and from there sets up
 * - general USB handling,
 * - the audio module,
 * - reporting functionality, and
 * - the user application
 *
 * @addtogroup main
 * @{
 */

#include <string.h>

#include "audio.h"
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "usb.h"

/**
 * @brief The number of messages that can be held.
 */
#define MESSAGE_BUFFER_LENGTH 10u

/**
 * @brief The main thread mailbox. Used for communicating with the audio module.
 */
static mailbox_t g_mailbox;

/**
 * @brief The mailbox buffer.
 */
static msg_t g_mailbox_buffer[MESSAGE_BUFFER_LENGTH];

#if ENABLE_REPORTING == TRUE
/**
 * @brief Global stream pointer for print messages.
 */
static BaseSequentialStream *gp_stream = (BaseSequentialStream *)&SD2;
#endif

// Weak definitions that shall be overwritten by the user application.

/**
 * @brief Set up the user application.
 */
__WEAK void app_setup(void) {}

/**
 * @brief Reset volume to maximum levels, when instructed (e.g. when stream ends).
 */
__WEAK void app_reset_volume(void) {}

/**
 * @brief Set a new volume from a USB configuration message.
 */
__WEAK void app_set_volume(void) {}

/**
 * @brief Set a new mute state from a USB configuration message.
 */
__WEAK void app_set_mute_state(void) {}

/**
 * @brief Application entry point.
 *
 * @return int The return code.
 */
int main(void) {
    halInit();
    chSysInit();

    // Initialize the main thread mailbox that receives audio messages (volume, mute).
    chMBObjectInit(&g_mailbox, g_mailbox_buffer, ARRAY_LENGTH(g_mailbox_buffer));

    // Initialize audio module, giving it access to the main thread's mailbox.
    audio_setup(&g_mailbox);

    // Initialize the USB module.
    usb_setup();

#if ENABLE_REPORTING == TRUE
    // Initialize a stream for print messages.
    sdStart(&SD2, NULL);

    chprintf(gp_stream, "Using %u byte per sample at %lu Hz, %lu byte per frame.\n", AUDIO_SAMPLE_SIZE,
             AUDIO_SAMPLE_RATE_HZ, AUDIO_PACKET_SIZE);
    chprintf(gp_stream, "Audio buffer holds %lu bytes (%lu packets).\n", AUDIO_BUFFER_SIZE, AUDIO_BUFFER_PACKET_COUNT);
    chprintf(gp_stream, "Audio feedback period is %lu ms.\n", AUDIO_FEEDBACK_PERIOD_MS);
#endif

    // Set up the user application.
    app_setup();

    // Wait for a message from the audio thread.
    while (true) {
        msg_t message;
        msg_t status = chMBFetchTimeout(&g_mailbox, &message, TIME_INFINITE);

        if (status != MSG_OK) {
            chSysHalt("Failed to receive message.");
        }

        switch (message) {
            case AUDIO_MSG_RESET_VOLUME:
                app_reset_volume();
                break;

            case AUDIO_MSG_SET_MUTE_STATE:
                app_set_mute_state();
                break;

            case AUDIO_MSG_SET_VOLUME:
                app_set_volume();
                break;

            default:
                chSysHalt("Unknown message type.");
                break;
        }
    }
}

/**
 * @}
 */
