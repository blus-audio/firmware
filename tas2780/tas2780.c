#include "tas2780.h"
#include "hal.h"

static struct tas2780_context tas2780_a_context =
    {
        .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT,
        .device_address = TAS2780_DEVICE_ADDRESS_A,
};
static struct tas2780_context tas2780_b_context =
    {
        .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT,
        .device_address = TAS2780_DEVICE_ADDRESS_B,
};
static struct tas2780_context tas2780_c_context =
    {
        .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT,
        .device_address = TAS2780_DEVICE_ADDRESS_C,
};
static struct tas2780_context tas2780_d_context =
    {
        .channel = TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT,
        .device_address = TAS2780_DEVICE_ADDRESS_D,
};

static struct tas2780_context *tas2780_contexts[] = {
    &tas2780_a_context, &tas2780_b_context, &tas2780_c_context, &tas2780_d_context};

/**
 * @brief Write data to a TAS2780 amplifier.
 *
 * @param p_context The pointer to the amplifier context.
 * @param p_buffer The pointer to the buffer to transmit.
 * @param size The size of the buffer.
 */
void tas2780_write(struct tas2780_context *p_context, uint8_t *p_buffer, size_t size)
{
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, p_context->device_address >> 1, p_buffer, size, NULL, 0);
    i2cReleaseBus(&I2CD1);
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

    p_write_buffer[0] = TAS2780_CHNL_0_REG;
    p_write_buffer[1] = ((0x03u << TAS2780_CHNL_0_CDS_MODE_POS) & TAS2780_CHNL_0_CDS_MODE_MASK) |
                        ((TAS2780_CHNL_0_AMP_LEVEL_DEFAULT << TAS2780_CHNL_0_AMP_LEVEL_POS) & TAS2780_CHNL_0_AMP_LEVEL_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    p_write_buffer[0] = TAS2780_DC_BLK0_REG;
    p_write_buffer[1] = ((0x01u << TAS2780_DC_BLK0_VBAT1S_MODE_POS) & TAS2780_DC_BLK0_VBAT1S_MODE_MASK) |
                        ((0x01u << TAS2780_DC_BLK0_AMP_SS_POS) & TAS2780_DC_BLK0_AMP_SS_MASK) | ((0x01u << TAS2780_DC_BLK0_HPF_FREQ_PB_POS) & TAS2780_DC_BLK0_HPF_FREQ_PB_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    p_write_buffer[0] = TAS2780_PVDD_UVLO_REG;
    p_write_buffer[1] = 0x0Eu; // 6.5 V
    tas2780_write(p_context, p_write_buffer, 2);

    p_write_buffer[0] = TAS2780_TDM_CFG2_REG;
    p_write_buffer[1] = ((0x02u << TAS2780_TDM_CFG2_RX_SLEN_POS) & TAS2780_TDM_CFG2_RX_SLEN_MASK) |
                        ((0x02u << TAS2780_TDM_CFG2_RX_WLEN_POS) & TAS2780_TDM_CFG2_RX_WLEN_MASK) |
                        ((p_context->channel << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);
    tas2780_write(p_context, p_write_buffer, 2);

    p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
    p_write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
    tas2780_write(p_context, p_write_buffer, 2);
}

/**
 * @brief Set up all connected TAS2780 amplifiers.
 */
void tas2780_setup_all(void)
{
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

        if (((p_context->channel == TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT) && (channel == TAS2780_CHANNEL_LEFT)) || ((p_context->channel == TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT) && (channel == TAS2780_CHANNEL_RIGHT)) || (channel == TAS2780_CHANNEL_BOTH))
        {
            tas2780_set_volume(p_context, volume_8q8_db);
        }
    }
}

// FIXME: check active state
// bool tas2780_is_active(struct tas2780_context *p_context)
// {

//     p_write_buffer[0] = TAS2780_MODE_CTRL_REG;
//     p_write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
//     tas2780_write(p_context, p_write_buffer, 2);

//     uint8_t reg;
//     HAL_I2C_Mem_Read(&hi2c1, address, TAS2780_MODE_CTRL_REG, I2C_MEMADD_SIZE_8BIT, &reg, 1, 10000);
//     uint8_t state = (reg & TAS2780_MODE_CTRL_MODE_MASK) >> TAS2780_MODE_CTRL_MODE_POS;

//     return ((state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE) || (state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE));
// }
