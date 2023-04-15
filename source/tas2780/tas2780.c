// Copyright 2023 elagil
#include "tas2780.h"

#include "hal.h"
#include "tas2780_settings.h"

/**
 * @brief All amplifier contexts.
 * @details Will be set up with settings from \a tas2780_settings.h .
 */
static struct tas2780_context g_tas2780_contexts[TAS2780_DEVICE_COUNT];

/**
 * @brief Initialize a TAS2780 amplifier context.
 *
 * @param p_context The pointer to the amplifier context.
 * @param channel The channel that the amplifier shall play.
 * @param device_address The amplifiers I2C device address.
 * @param tdm_slot_index The TDM slot index, on which the amplifier plays.
 * @param analog_gain_setting The analog gain setting between 0x00 and 0x14,
 * where 0x14 is 10 dB louder than 0x00.
 */
void tas2780_init_context(struct tas2780_context *p_context, enum tas2780_channel channel, uint16_t device_address,
                          uint8_t tdm_slot_index, uint8_t analog_gain_setting) {
    p_context->analog_gain_setting =
        analog_gain_setting > TAS2780_CHNL_0_AMP_LEVEL_MAX ? TAS2780_CHNL_0_AMP_LEVEL_MAX : analog_gain_setting;
    p_context->channel        = channel;
    p_context->device_address = device_address;
    p_context->tdm_slot_index = tdm_slot_index;
}

/**
 * @brief Exchange data with a TAS2780 amplifier.
 *
 * @param p_context The pointer to the amplifier context.
 * @param p_write_buffer The pointer to the buffer to transmit. Can be NULL.
 * @param write_size The size of the write buffer.
 * @param p_read_buffer The pointer to the buffer to transmit. Can be NULL.
 * @param read_size The size of the read buffer.
 */
void tas2780_exchange(struct tas2780_context *p_context, uint8_t *p_write_buffer, size_t write_size,
                      uint8_t *p_read_buffer, size_t read_size) {
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, p_context->device_address >> 1, p_write_buffer, write_size, p_read_buffer, read_size);
    i2cReleaseBus(&I2CD1);
}

/**
 * @brief Read data from a TAS2780 amplifier.
 *
 * @param p_context The pointer to the amplifier context.
 * @param p_read_buffer The pointer to the buffer to transmit. Can be NULL.
 * @param read_size The size of the read buffer.
 */
void tas2780_read(struct tas2780_context *p_context, uint8_t *p_read_buffer, size_t read_size) {
    i2cAcquireBus(&I2CD1);
    i2cMasterReceive(&I2CD1, p_context->device_address >> 1, p_read_buffer, read_size);
    i2cReleaseBus(&I2CD1);
}

/**
 * @brief Write data to a TAS2780 amplifier.
 *
 * @param p_context The pointer to the amplifier context.
 * @param p_write_buffer The pointer to the buffer to transmit.
 * @param write_size The size of the write buffer.
 */
void tas2780_write(struct tas2780_context *p_context, uint8_t *p_write_buffer, size_t write_size) {
    tas2780_exchange(p_context, p_write_buffer, write_size, NULL, 0u);
}

/**
 * @brief Set the volume on a single TAS2780 amplifier.
 *
 * @param p_context The pointer to the amplifier context.
 * @param volume_8q8_db The volume to set in 8.8 signed binary fixpoint format.
 */
void tas2780_set_volume(struct tas2780_context *p_context, int16_t volume_8q8_db) {
    uint8_t *p_write_buffer = p_context->write_buffer;
    uint8_t  volume_db      = TAS2780_VOLUME_FROM_8Q8_DB(volume_8q8_db);

    p_write_buffer[0]       = TAS2780_DVC_REG;
    p_write_buffer[1]       = volume_db;
    tas2780_write(p_context, p_write_buffer, 2);
}

/**
 * @brief Set up a single TAS2780 amplifier with default values.
 *
 * @param p_context The pointer to the amplifier context.
 */
void tas2780_setup(struct tas2780_context *p_context) {
    // FIXME: Separate into functions.
    // FIXME: Take into account the page of registers. Currently, it is assumed that the amplifier is resting at page 0
    // after setup.
    uint8_t *p_write_buffer = p_context->write_buffer;

    // Go to page 0.
    p_write_buffer[0] = TAS2780_PAGE_REG;
    p_write_buffer[1] = 0x00u;  // Page 0x00.
    tas2780_write(p_context, p_write_buffer, 2);

    // Go to book 0.
    p_write_buffer[0] = TAS2780_BOOK_REG;
    p_write_buffer[1] = 0x00u;  // Book 0x00.
    tas2780_write(p_context, p_write_buffer, 2);

    // Perform software reset.
    p_write_buffer[0] = TAS2780_SW_RESET_REG;
    p_write_buffer[1] = (0x01u << TAS2780_SW_RESET_SW_RESET_POS) & TAS2780_SW_RESET_SW_RESET_MASK;
    tas2780_write(p_context, p_write_buffer, 2);

    // Wait for the device to start up.
    osalThreadSleepMilliseconds(1);

    // Go to page 1.
    p_write_buffer[0] = TAS2780_PAGE_REG;
    p_write_buffer[1] = 0x01u;  // Page 0x01.
    tas2780_write(p_context, p_write_buffer, 2);

    // undocumented
    p_write_buffer[0] = 0x17u;
    p_write_buffer[1] = 0xC0u;
    tas2780_write(p_context, p_write_buffer, 2);

    p_write_buffer[0] = TAS2780_LSR_REG;
    p_write_buffer[1] = (0x01u << TAS2780_LSR_EN_LLSR_POS) & TAS2780_LSR_EN_LLSR_MASK;
    tas2780_write(p_context, p_write_buffer, 2);

    // undocumented
    p_write_buffer[0] = 0x21u;
    p_write_buffer[1] = 0x00u;
    tas2780_write(p_context, p_write_buffer, 2);

    // undocumented
    p_write_buffer[0] = 0x35u;
    p_write_buffer[1] = 0x74u;
    tas2780_write(p_context, p_write_buffer, 2);

    p_write_buffer[0] = TAS2780_PAGE_REG;
    p_write_buffer[1] = 0xFDu;
    tas2780_write(p_context, p_write_buffer, 2);

    // undocumented
    p_write_buffer[0] = 0x0Du;
    p_write_buffer[1] = 0x0Du;
    tas2780_write(p_context, p_write_buffer, 2);

    // undocumented
    p_write_buffer[0] = 0x3Eu;
    p_write_buffer[1] = 0x4Au;
    tas2780_write(p_context, p_write_buffer, 2);

    // undocumented
    p_write_buffer[0] = 0x0Du;
    p_write_buffer[1] = 0x00u;
    tas2780_write(p_context, p_write_buffer, 2);

    // Go to page 0.
    p_write_buffer[0] = TAS2780_PAGE_REG;
    p_write_buffer[1] = 0x00u;  // Page 0x00.
    tas2780_write(p_context, p_write_buffer, 2);

    // Set the analog gain of the amplifier.
    p_write_buffer[0] = TAS2780_CHNL_0_REG;
    p_write_buffer[1] =
        ((0x03u << TAS2780_CHNL_0_CDS_MODE_POS) & TAS2780_CHNL_0_CDS_MODE_MASK) |
        ((p_context->analog_gain_setting << TAS2780_CHNL_0_AMP_LEVEL_POS) & TAS2780_CHNL_0_AMP_LEVEL_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    // Set up "PWR_MODE2".
    // PVDD is the only supply. VBAT1S is delivered by an internal LDO and used to supply at signals close to idle
    // channel levels. When audio signal levels crosses -100dBFS (default), Class-D output switches to PVDD.
    p_write_buffer[0] = TAS2780_DC_BLK0_REG;
    p_write_buffer[1] = ((0x01u << TAS2780_DC_BLK0_VBAT1S_MODE_POS) & TAS2780_DC_BLK0_VBAT1S_MODE_MASK) |
                        ((0x01u << TAS2780_DC_BLK0_AMP_SS_POS) & TAS2780_DC_BLK0_AMP_SS_MASK) |
                        ((0x01u << TAS2780_DC_BLK0_HPF_FREQ_PB_POS) & TAS2780_DC_BLK0_HPF_FREQ_PB_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    // Under-voltage lockout, set to 6.5 V.
    p_write_buffer[0] = TAS2780_PVDD_UVLO_REG;
    p_write_buffer[1] = 0x0Eu;  // 6.5 V
    tas2780_write(p_context, p_write_buffer, 2);

    // The TDM_CFG2 register content - initially without channel information.
    uint8_t tdm_cfg2 =
        ((0x02u << TAS2780_TDM_CFG2_RX_SLEN_POS) & TAS2780_TDM_CFG2_RX_SLEN_MASK) |
        ((0x02u << TAS2780_TDM_CFG2_RX_WLEN_POS) & TAS2780_TDM_CFG2_RX_WLEN_MASK) |
        ((TAS2780_TDM_CFG2_RX_SCFG_DEFAULT << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);

    // The TDM_CFG3 register content.
    uint8_t tdm_cfg3;

    // Determine the configured audio channel (left, or right).
    switch (p_context->channel) {
        case TAS2780_CHANNEL_LEFT:
            tdm_cfg2 |=
                ((TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);

            // Set the specified TDM slot for the left channel. Leave the TDM slot at default for the right channel.
            tdm_cfg3 =
                ((p_context->tdm_slot_index << TAS2780_TDM_CFG3_RX_SLOT_L_POS) & TAS2780_TDM_CFG3_RX_SLOT_L_MASK) |
                ((TAS2780_TDM_CFG3_RX_SLOT_R_DEFAULT << TAS2780_TDM_CFG3_RX_SLOT_R_POS) &
                 TAS2780_TDM_CFG3_RX_SLOT_R_MASK);
            break;

        case TAS2780_CHANNEL_RIGHT:
            tdm_cfg2 |=
                ((TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);

            // Set the specified TDM slot for the right channel. Leave the TDM slot at default for the left channel.
            tdm_cfg3 =
                ((p_context->tdm_slot_index << TAS2780_TDM_CFG3_RX_SLOT_R_POS) & TAS2780_TDM_CFG3_RX_SLOT_R_MASK) |
                ((TAS2780_TDM_CFG3_RX_SLOT_L_DEFAULT << TAS2780_TDM_CFG3_RX_SLOT_L_POS) &
                 TAS2780_TDM_CFG3_RX_SLOT_L_MASK);
            break;

        default:
            // Default channel: defined by I2C address.
            tdm_cfg2 |=
                ((TAS2780_TDM_CFG2_RX_SCFG_DEFAULT << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);

            // Default TDM slot settings
            // - left: 0
            // - right: 1
            tdm_cfg3 = ((TAS2780_TDM_CFG3_RX_SLOT_L_DEFAULT << TAS2780_TDM_CFG3_RX_SLOT_L_POS) &
                        TAS2780_TDM_CFG3_RX_SLOT_L_MASK) |
                       ((TAS2780_TDM_CFG3_RX_SLOT_R_DEFAULT << TAS2780_TDM_CFG3_RX_SLOT_R_POS) &
                        TAS2780_TDM_CFG3_RX_SLOT_R_MASK);
            break;
    }

    // Set up the determined audio channel.
    p_write_buffer[0] = TAS2780_TDM_CFG2_REG;
    p_write_buffer[1] = tdm_cfg2;
    tas2780_write(p_context, p_write_buffer, 2);

    // Set up the matching TDM slot.
    p_write_buffer[0] = TAS2780_TDM_CFG3_REG;
    p_write_buffer[1] = tdm_cfg3;
    tas2780_write(p_context, p_write_buffer, 2);

    // Set up the noise gate.
    p_write_buffer[0] = TAS2780_NG_CFG0_REG;
    p_write_buffer[1] =
        ((TAS2780_NG_CFG0_RES_DEFAULT << TAS2780_NG_CFG0_RES_POS) & TAS2780_NG_CFG0_RES_MASK) |
        ((TAS2780_NG_CFG0_NG_EN_DEFAULT << TAS2780_NG_CFG0_NG_EN_POS) & TAS2780_NG_CFG0_NG_EN_MASK) |
        ((TAS2780_NG_CFG0_NG_LVL_DEFAULT << TAS2780_NG_CFG0_NG_LVL_POS) & TAS2780_NG_CFG0_NG_LVL_MASK) |
        ((TAS2780_NG_CFG0_NG_HYST_DEFAULT << TAS2780_NG_CFG0_NG_HYST_POS) & TAS2780_NG_CFG0_NG_HYST_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    // Enable active mode without mute.
    p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
    p_write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
    tas2780_write(p_context, p_write_buffer, 2);
}

/**
 * @brief Set up all connected TAS2780 amplifiers.
 */
void tas2780_setup_all(void) {
    // Common hardware reset for all amplifiers.
    palClearLine(LINE_NSPK_SD);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_NSPK_SD);

    for (size_t device_index = 0; device_index < TAS2780_DEVICE_COUNT; device_index++) {
        tas2780_init_context(&g_tas2780_contexts[device_index], TAS2780_DEVICE_CHANNELS[device_index],
                             TAS2780_DEVICE_ADDRESSES[device_index], TAS2780_TDM_SLOT_INDICES[device_index],
                             TAS2780_CHNL_0_AMP_LEVEL_MIN);
        tas2780_setup(&g_tas2780_contexts[device_index]);
    }
}

/**
 * @brief Sets the volume on all connected TAS2780 amplifiers, for a chosen channel.
 *
 * @param volume_8q8_db The volume to set in 8.8 signed binary fixpoint format.
 * @param channel The channel to set the volume for.
 */
void tas2780_set_volume_all(int16_t volume_8q8_db, enum tas2780_channel channel) {
    for (size_t device_index = 0; device_index < TAS2780_DEVICE_COUNT; device_index++) {
        struct tas2780_context *p_context = &g_tas2780_contexts[device_index];

        if (((p_context->channel == TAS2780_CHANNEL_LEFT) && (channel == TAS2780_CHANNEL_LEFT)) ||
            ((p_context->channel == TAS2780_CHANNEL_RIGHT) && (channel == TAS2780_CHANNEL_RIGHT)) ||
            (channel == TAS2780_CHANNEL_BOTH)) {
            tas2780_set_volume(p_context, volume_8q8_db);
        }
    }
}

/**
 * @brief Check the amplifier state to be active without mute, and enforce it.
 *
 * @param p_context The pointer to the amplifier context.
 */
void tas2780_ensure_active(struct tas2780_context *p_context) {
    uint8_t *p_write_buffer = p_context->write_buffer;
    uint8_t *p_read_buffer  = p_context->read_buffer;

    p_write_buffer[0]       = TAS2780_MODE_CTRL_REG;
    tas2780_write(p_context, p_write_buffer, 1u);
    tas2780_read(p_context, p_read_buffer, 1u);

    uint8_t state = (p_read_buffer[0] & TAS2780_MODE_CTRL_MODE_MASK) >> TAS2780_MODE_CTRL_MODE_POS;

    if (!((state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE) ||
          (state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE))) {
        p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
        p_write_buffer[1] =
            ((TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
        tas2780_write(p_context, p_write_buffer, 2);
    }
}

/**
 * @brief Ensure the active state without mute on all connected amplifiers.
 */
void tas2780_ensure_active_all(void) {
    for (size_t device_index = 0; device_index < TAS2780_DEVICE_COUNT; device_index++) {
        tas2780_ensure_active(&g_tas2780_contexts[device_index]);
    }
}

/**
 * @brief Determine, whether or not the noise gate is active.
 * @details An active noise gate means that the amplifier goes into an energy-saving state, when it does not detect a
 * significant input level. The threshold input level can be adjusted between -90 dBFS and -120 dBFS.
 *
 * @param p_context The pointer to the amplifier context.
 * @return true if the noise gate is active (the amplifier does not output a signal).
 * @return false if the noise gate is inactive.
 */
bool tas2780_noise_gate_is_enabled(struct tas2780_context *p_context) {
    uint8_t *p_write_buffer = p_context->write_buffer;
    uint8_t *p_read_buffer  = p_context->read_buffer;

    p_write_buffer[0]       = TAS2780_INT_LIVE1_REG;
    tas2780_write(p_context, p_write_buffer, 1u);
    tas2780_read(p_context, p_read_buffer, 1u);

    uint8_t state = (p_read_buffer[0] & TAS2780_INT_LIVE1_IL_NGA_MASK) >> TAS2780_INT_LIVE1_IL_NGA_POS;

    return (bool)state;
}

/**
 * @brief Get the noise gate mask for all amplifiers (up to eight)
 * @details Each bit indicates, whether the noise gate is active (1) or inactive (0) for an amplifier. The LSB
 * represents the first amplifier in the array \a g_tas2780_contexts .
 *
 * @return uint8_t The noise gate mask.
 */
uint8_t tas2780_get_noise_gate_mask_all(void) {
    uint8_t noise_gate_mask = 0u;

    for (size_t device_index = 0; device_index < TAS2780_DEVICE_COUNT; device_index++) {
        if (tas2780_noise_gate_is_enabled(&g_tas2780_contexts[device_index])) {
            noise_gate_mask |= (1u << device_index);
        }
    }

    return noise_gate_mask;
}
