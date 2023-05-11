// Copyright2023 elagil

/**
 * @file
 * @brief   TI TAS2780 amplifier control module headers.
 *
 * @addtogroup tas2780
 * @{
 */

#ifndef SOURCE_DRIVERS_TAS2780_TAS2780_H_
#define SOURCE_DRIVERS_TAS2780_TAS2780_H_

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

/**
 * @brief The volume level that the TAS2780 interprets as mute.
 */
#define TAS2780_VOLUME_MUTE (0xFFu)

/**
 * @brief The volume level that the TAS2780 interprets as maximum.
 */
#define TAS2780_VOLUME_MAX (0x00u)

/**
 * @brief Read and write buffer lengths.
 */
#define TAS2780_BUFFER_LENGTH 2u

/**
 * @brief The selected channel.
 */
enum tas2780_channel {
    TAS2780_CHANNEL_LEFT,   ///< The left channel.
    TAS2780_CHANNEL_RIGHT,  ///< The right channel.
    TAS2780_CHANNEL_BOTH,   ///< Both channels.
};
/**
 * @brief The context for holding information about a TAS2780 amplifier.
 */
struct tas2780_context {
    uint8_t              write_buffer[TAS2780_BUFFER_LENGTH];  ///< The write buffer for I2C transactions.
    uint8_t              read_buffer[TAS2780_BUFFER_LENGTH];   ///< The read buffer for I2C transactions.
    uint8_t              page_index;                           ///< The page that the device is set to.
    uint8_t              book_index;                           ///< The book that the device is set to.
    uint16_t             device_address;                       ///< The I2C device address.
    uint8_t              tdm_slot_index;                       ///< The TDM slot index on which this device plays.
    enum tas2780_channel channel;  ///< The audio channel (left/right). When TAS2780_CHANNEL_BOTH
                                   ///< is selected, sets up stereo mixing.
    uint8_t analog_gain_setting;   ///< Can be between 0x00 and 0x14, where 0x14
                                   ///< is loudest. The range of gains is 10 dB.
};

void    tas2780_setup_all(void);
void    tas2780_set_volume_all(int16_t volume_8q8_db, enum tas2780_channel channel);
void    tas2780_init(struct tas2780_context *p_context, uint16_t device_address);
void    tas2780_ensure_active_all(void);
uint8_t tas2780_get_noise_gate_mask_all(void);

#endif  // SOURCE_DRIVERS_TAS2780_TAS2780_H_

/**
 * @}
 */
