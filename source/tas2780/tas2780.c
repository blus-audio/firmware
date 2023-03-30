#include "hal.h"
#include "tas2780.h"

static struct tas2780_context tas2780_a_context;
static struct tas2780_context tas2780_b_context;
static struct tas2780_context tas2780_c_context;
static struct tas2780_context tas2780_d_context;

/**
 * @brief An array of all amplifier contexts.
 */
static struct tas2780_context *tas2780_contexts[] = {
    &tas2780_a_context, &tas2780_b_context, &tas2780_c_context, &tas2780_d_context};

/**
 * @brief Initialize a TAS2780 amplifier context.
 *
 * @param p_context The pointer to the amplifier context.
 * @param channel The channel that the amplifier shall play.
 * @param device_address The amplifiers I2C device address.
 * @param analog_gain_setting The analog gain setting between 0x00 and 0x14, where 0x14 is 10 dB louder than 0x00.
 */
void tas2780_init_context(struct tas2780_context *p_context, enum tas2780_channel channel, uint16_t device_address, uint8_t analog_gain_setting)
{
    p_context->analog_gain_setting = analog_gain_setting > TAS2780_CHNL_0_AMP_LEVEL_MAX ? TAS2780_CHNL_0_AMP_LEVEL_MAX : analog_gain_setting;
    p_context->channel = channel;
    p_context->device_address = device_address;
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
void tas2780_exchange(struct tas2780_context *p_context,
                      uint8_t *p_write_buffer,
                      size_t write_size,
                      uint8_t *p_read_buffer,
                      size_t read_size)
{
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
void tas2780_read(struct tas2780_context *p_context,
                  uint8_t *p_read_buffer,
                  size_t read_size)
{
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
void tas2780_write(struct tas2780_context *p_context,
                   uint8_t *p_write_buffer,
                   size_t write_size)
{
    tas2780_exchange(p_context, p_write_buffer, write_size, NULL, 0u);
}

/**
 * @brief Set the volume on a single TAS2780 amplifier.
 *
 * @param p_context The pointer to the amplifier context.
 * @param volume_8q8_db The volume to set in 8.8 signed binary fixpoint format.
 */
void tas2780_set_volume(struct tas2780_context *p_context, int16_t volume_8q8_db)
{
    uint8_t *p_write_buffer = p_context->write_buffer;
    uint8_t volume_db = TAS2780_VOLUME_FROM_8Q8_DB(volume_8q8_db);

    p_write_buffer[0] = TAS2780_DVC_REG;
    p_write_buffer[1] = volume_db;
    tas2780_write(p_context, p_write_buffer, 2);
}

/**
 * @brief Set up a single TAS2780 amplifier with default values.
 *
 * @param p_context The pointer to the amplifier context.
 */
void tas2780_setup(struct tas2780_context *p_context)
{
    uint8_t *p_write_buffer = p_context->write_buffer;

    // Go to page 0.
    p_write_buffer[0] = TAS2780_PAGE_REG;
    p_write_buffer[1] = 0x00u; // Page 0x00.
    tas2780_write(p_context, p_write_buffer, 2);

    // Go to book 0.
    p_write_buffer[0] = TAS2780_BOOK_REG;
    p_write_buffer[1] = 0x00u; // Book 0x00.
    tas2780_write(p_context, p_write_buffer, 2);

    // Perform software reset.
    p_write_buffer[0] = TAS2780_SW_RESET_REG;
    p_write_buffer[1] = (0x01u << TAS2780_SW_RESET_SW_RESET_POS) & TAS2780_SW_RESET_SW_RESET_MASK;
    tas2780_write(p_context, p_write_buffer, 2);

    // Wait for the device to start up.
    osalThreadSleepMilliseconds(1);

    // Go to page 1.
    p_write_buffer[0] = TAS2780_PAGE_REG;
    p_write_buffer[1] = 0x01u; // Page 0x01.
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
    p_write_buffer[1] = 0x00u; // Page 0x00.
    tas2780_write(p_context, p_write_buffer, 2);

    // Set the analog gain of the amplifier.
    p_write_buffer[0] = TAS2780_CHNL_0_REG;
    p_write_buffer[1] = ((0x03u << TAS2780_CHNL_0_CDS_MODE_POS) & TAS2780_CHNL_0_CDS_MODE_MASK) |
                        ((p_context->analog_gain_setting << TAS2780_CHNL_0_AMP_LEVEL_POS) & TAS2780_CHNL_0_AMP_LEVEL_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    // Set up "PWR_MODE2".
    // PVDD is the only supply. VBAT1S is delivered by an internal LDO and used to supply at signals
    // close to idle channel levels. When audio signal levels crosses -100dBFS (default),
    // Class-D output switches to PVDD.
    p_write_buffer[0] = TAS2780_DC_BLK0_REG;
    p_write_buffer[1] = ((0x01u << TAS2780_DC_BLK0_VBAT1S_MODE_POS) & TAS2780_DC_BLK0_VBAT1S_MODE_MASK) |
                        ((0x01u << TAS2780_DC_BLK0_AMP_SS_POS) & TAS2780_DC_BLK0_AMP_SS_MASK) | ((0x01u << TAS2780_DC_BLK0_HPF_FREQ_PB_POS) & TAS2780_DC_BLK0_HPF_FREQ_PB_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    // Under-voltage lockout, set to 6.5 V.
    p_write_buffer[0] = TAS2780_PVDD_UVLO_REG;
    p_write_buffer[1] = 0x0Eu; // 6.5 V
    tas2780_write(p_context, p_write_buffer, 2);

    // Determine the configured audio channel (left, right, stereo mix).
    uint8_t tdm_cfg2_rx_scfg = TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT;
    switch (p_context->channel)
    {
    case TAS2780_CHANNEL_LEFT:
        tdm_cfg2_rx_scfg = TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT;
        break;

    case TAS2780_CHANNEL_RIGHT:
        tdm_cfg2_rx_scfg = TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT;
        break;

    case TAS2780_CHANNEL_BOTH:
        tdm_cfg2_rx_scfg = TAS2780_TDM_CFG2_RX_SCFG_MONO_STEREO_MIX;
        break;

    default:
        break;
    }

    // Set up the determined audio channel.
    p_write_buffer[0] = TAS2780_TDM_CFG2_REG;
    p_write_buffer[1] = ((0x02u << TAS2780_TDM_CFG2_RX_SLEN_POS) & TAS2780_TDM_CFG2_RX_SLEN_MASK) |
                        ((0x02u << TAS2780_TDM_CFG2_RX_WLEN_POS) & TAS2780_TDM_CFG2_RX_WLEN_MASK) |
                        ((tdm_cfg2_rx_scfg << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    // Enable active mode without mute.
    p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
    p_write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
    tas2780_write(p_context, p_write_buffer, 2);
}

/**
 * @brief Set up all connected TAS2780 amplifiers.
 */
void tas2780_setup_all(void)
{
    // Common hardware reset for all amplifiers.
    palClearLine(LINE_NSPK_SD);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_NSPK_SD);

    tas2780_init_context(&tas2780_a_context, TAS2780_CHANNEL_LEFT, TAS2780_DEVICE_ADDRESS_A, 0x00);
    tas2780_init_context(&tas2780_b_context, TAS2780_CHANNEL_RIGHT, TAS2780_DEVICE_ADDRESS_B, 0x00);
    tas2780_init_context(&tas2780_c_context, TAS2780_CHANNEL_LEFT, TAS2780_DEVICE_ADDRESS_C, 0x00);
    tas2780_init_context(&tas2780_d_context, TAS2780_CHANNEL_RIGHT, TAS2780_DEVICE_ADDRESS_D, 0x00);

    for (size_t amplifier_index = 0; amplifier_index < ARRAY_LENGTH(tas2780_contexts); amplifier_index++)
    {
        tas2780_setup(tas2780_contexts[amplifier_index]);
    }
}

/**
 * @brief Sets the volume on all connected TAS2780 amplifiers, for a chosen channel.
 *
 * @param volume_8q8_db The volume to set in 8.8 signed binary fixpoint format.
 * @param channel The channel to set the volume for.
 */
void tas2780_set_volume_all(int16_t volume_8q8_db, enum tas2780_channel channel)
{
    for (size_t amplifier_index = 0; amplifier_index < ARRAY_LENGTH(tas2780_contexts); amplifier_index++)
    {
        struct tas2780_context *p_context = tas2780_contexts[amplifier_index];

        if (((p_context->channel == TAS2780_CHANNEL_LEFT) && (channel == TAS2780_CHANNEL_LEFT)) ||
            ((p_context->channel == TAS2780_CHANNEL_RIGHT) && (channel == TAS2780_CHANNEL_RIGHT)) ||
            (channel == TAS2780_CHANNEL_BOTH))
        {
            tas2780_set_volume(p_context, volume_8q8_db);
        }
    }
}

/**
 * @brief Checks the amplifier state for being active without mute, and enforces it.
 *
 * @param p_context The pointer to the amplifier context.
 */
void tas2780_ensure_active(struct tas2780_context *p_context)
{
    uint8_t *p_write_buffer = p_context->write_buffer;
    uint8_t *p_read_buffer = p_context->read_buffer;

    p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
    p_write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
    tas2780_write(p_context, p_write_buffer, 2u);
    tas2780_read(p_context, p_read_buffer, 1u);

    uint8_t state = (p_read_buffer[0] & TAS2780_MODE_CTRL_MODE_MASK) >> TAS2780_MODE_CTRL_MODE_POS;

    if (!((state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE) || (state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE)))
    {
        p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
        p_write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
        tas2780_write(p_context, p_write_buffer, 2);
    }
}

/**
 * @brief Ensure the active state without mute on all connected amplifiers.
 */
void tas2780_ensure_active_all(void)
{
    for (size_t amplifier_index = 0; amplifier_index < ARRAY_LENGTH(tas2780_contexts); amplifier_index++)
    {
        struct tas2780_context *p_context = tas2780_contexts[amplifier_index];
        tas2780_ensure_active(p_context);
    }
}
