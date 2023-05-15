// Copyright 2023 elagil

/**
 * @file
 * @brief   Main audio module.
 * @details Contains functionality for setting up and controlling lower-level audio modules, as well as an application
 * interface in the shape of a mailbox.
 *
 * @addtogroup audio
 * @{
 */

#include "audio.h"

#include <string.h>

#include "common.h"

/**
 * @brief The number of audio messages that can be held.
 */
#define AUDIO_MESSAGE_BUFFER_LENGTH 10u

/**
 * @brief The time in ticks after which an \a AUDIO_COMMON_MSG_RESET_VOLUME message is sent, following the end of
 * playback.
 * @details When playback ends, the volume is not reset immediately, but only after a certain time has passed. This is
 * done so that the volume is not reset during short playback pauses, but remains at the configured level.
 * @note A setting of \a CH_CFG_ST_FREQUENCY leads to a timeout of one second.
 */
#define AUDIO_RESET_VOLUME_TIMEOUT (CH_CFG_ST_FREQUENCY / 10u)  // 100 ms delay

/**
 * @brief Calculate the I2S register content for I2SPR.
 * @details Calculate the clock divider from the I2S PLL clock output and the selected sample rate. Also enables the
 * master clock output.
 * @note See the STM32F401 reference manual at p. 594.
 *
 * @param _sample_rate_hz The sample rate in Hz.
 */
#define AUDIO_SPI_GET_I2SPR(_sample_rate_hz)                                                                           \
    (SPI_I2SPR_MCKOE | (SPI_I2SPR_I2SDIV & (STM32_PLLI2S_R_CLKOUT / (_sample_rate_hz) / 512u)))

// Set the I2S CFGR register depending on the chosen audio resolution.
#if AUDIO_RESOLUTION_BIT == 16u
#define AUDIO_I2S_CFGR 0u
#elif AUDIO_RESOLUTION_BIT == 32u
#define AUDIO_I2S_CFGR SPI_I2SCFGR_DATLEN_1
#else
#error "Unsupported audio resolution. Must be 16, or 32 bit."
#endif

/**
 * @brief Audio mailbox for communication between ISRs and the audio thread.
 */
static mailbox_t g_audio_mailbox;

/**
 * @brief The mailbox buffer.
 */
static msg_t g_audio_mailbox_buffer[AUDIO_MESSAGE_BUFFER_LENGTH];

/**
 * @brief The main audio context.
 * @details Holds all important audio-related structures.
 */
static volatile struct audio_context {
    mailbox_t *p_mailbox;  ///< The pointer to a mailbox that receives messages from the audio thread. Can be NULL.
} g_audio_context;

/**
 * @brief Settings structure for the I2S driver.
 * @details Enables the master clock output for timer capture.
 */
static I2SConfig g_i2s_config = {
    .tx_buffer = NULL,  // To be set at runtime.
    .rx_buffer = NULL,
    .size      = 0u,  // To be set at runtime.
    .end_cb    = NULL,
    .i2scfgr   = AUDIO_I2S_CFGR,
    .i2spr     = 0u  // To be set at runtime.
};

/**
 * @brief Determine, whether a mailbox is known to the audio module.
 * @details If the mailbox pointer is NULL, no communication must be attempted.
 *
 * @return true if a valid mailbox is known.
 * @return false if no valid mailbox is known.
 */
static bool audio_mailbox_is_set(void) { return g_audio_context.p_mailbox != NULL; }

/**
 * @brief Initialize an audio context, and all its contained structures.
 *
 * @param p_context The pointer to the context to initialize.
 * @param p_mailbox The pointer to the mailbox, where audio messages are posted.
 */
static void audio_init_context(volatile struct audio_context *p_context, mailbox_t *p_mailbox) {
    p_context->p_mailbox = p_mailbox;
}

/**
 * @brief Reset the audio module.
 * @details Is called on USB reset, unconfigure, or suspend. First calls \a audio_stop_streaming() , then resets the
 * internal state of the module.
 *
 * @param usbp The pointer to the USB driver structure.
 */
void audio_reset(USBDriver *usbp) {
    audio_playback_stop_streaming(usbp);
    audio_init_context(&g_audio_context, g_audio_context.p_mailbox);
}

/**
 * @brief The volume reset timer callback function.
 * @details Sends a \a AUDIO_MSG_RESET_VOLUME message to a listening thread, if playback is still disabled after the
 * timeout period. If playback is no longer disabled, no message is sent.
 *
 * @param p_virtual_timer A pointer to the virtual timer object (unused).
 * @param p_arg A pointer to the callback argument (unused).
 */
static void audio_volume_reset_cb(virtual_timer_t *p_virtual_timer, void *p_arg) {
    (void)p_virtual_timer;
    (void)p_arg;

    chSysLockFromISR();
    if (!audio_playback_is_enabled() && audio_mailbox_is_set()) {
        chMBPostI(g_audio_context.p_mailbox, AUDIO_COMMON_MSG_RESET_VOLUME);
    }
    chSysUnlockFromISR();
}

/**
 * @brief Send a message to the application layer, if an application mailbox is configured.
 *
 * @param message The message to send.
 */
static void audio_send_app_message(msg_t message) {
    if (audio_mailbox_is_set()) {
        chMBPostTimeout(g_audio_context.p_mailbox, message, TIME_INFINITE);
    }
}

static void audio_update_sample_rate(void) {
    uint32_t sample_rate_hz = audio_request_get_sample_rate_hz();
    audio_playback_set_sample_rate(sample_rate_hz);

    if (audio_playback_is_enabled()) {
        i2sStopExchange(&I2S_DRIVER);
    }

    // The I2S size is counted in number of transactions.
    g_i2s_config.size  = audio_playback_get_buffer_size() / AUDIO_SAMPLE_SIZE;
    g_i2s_config.i2spr = AUDIO_SPI_GET_I2SPR(sample_rate_hz);

    if (audio_playback_is_enabled()) {
        i2sStartExchange(&I2S_DRIVER);
    }
}

static THD_WORKING_AREA(wa_audio_thread, 128);

/**
 * @brief A thread that handles audio-related tasks.
 */
static THD_FUNCTION(audio_thread, arg) {
    (void)arg;
    chRegSetThreadName("audio");

    // Enable the feedback counter timer TIM2 peripheral clock (no low-power mode).
    rccEnableTIM2(false);

    // Initialize a volume reset timer.
    virtual_timer_t volume_reset_timer;
    chVTObjectInit(&volume_reset_timer);

    // Wait for a message from an audio ISR.
    while (true) {
        msg_t message;
        msg_t status = chMBFetchTimeout(&g_audio_mailbox, &message, TIME_INFINITE);

        if (status != MSG_OK) {
            chSysHalt("Failed to receive message.");
        }

        switch (message) {
            case AUDIO_COMMON_MSG_START_PLAYBACK:
                // Set volumes to the values configured via USB audio.
                audio_send_app_message(AUDIO_COMMON_MSG_SET_VOLUME);

                i2sStart(&I2S_DRIVER, &g_i2s_config);
                i2sStartExchange(&I2S_DRIVER);
                audio_feedback_start_sof_capture();
                break;

            case AUDIO_COMMON_MSG_STOP_PLAYBACK:
                audio_feedback_stop_sof_capture();
                i2sStopExchange(&I2S_DRIVER);
                i2sStop(&I2S_DRIVER);

                chVTSet(&volume_reset_timer, AUDIO_RESET_VOLUME_TIMEOUT, audio_volume_reset_cb, NULL);
                break;

            case AUDIO_COMMON_MSG_SET_SAMPLE_RATE:
                audio_update_sample_rate();
                break;

            // Relay messages to the app layer.
            case AUDIO_COMMON_MSG_SET_MUTE_STATE:
            case AUDIO_COMMON_MSG_SET_VOLUME:
            case AUDIO_COMMON_MSG_RESET_VOLUME:
                audio_send_app_message(message);
                break;

            default:
                chSysHalt("Unknown message type.");
                break;
        }
    }
}

/**
 * @brief Set up all components of the audio module.
 *
 * @param p_mailbox The pointer to the mailbox, where audio messages are posted. To be provided by the user application.
 */
void audio_setup(mailbox_t *p_mailbox) {
    audio_request_init(&g_audio_mailbox);
    audio_playback_init(&g_audio_mailbox, false);
    audio_feedback_init();
    audio_update_sample_rate();

    // Connect the playback buffer with the I2S peripheral.
    g_i2s_config.tx_buffer = (const void *)audio_playback_get_buffer();

    // Initialize the mailbox connections.
    audio_init_context(&g_audio_context, p_mailbox);
    chMBObjectInit(&g_audio_mailbox, g_audio_mailbox_buffer, ARRAY_LENGTH(g_audio_mailbox_buffer));

    // Start the audio thread at the end.
    chThdCreateStatic(wa_audio_thread, sizeof(wa_audio_thread), NORMALPRIO, audio_thread, NULL);
}

/**
 * @}
 */
