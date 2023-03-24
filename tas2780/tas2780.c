#include "tas2780.h"
#include "hal.h"

void tas2780_transmit(struct tas2780_context *p_context, uint8_t *p_buffer, size_t size)
{
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, p_context->device_address >> 1, p_buffer, size, NULL, 0);
    i2cReleaseBus(&I2CD1);
}

void tas2780_init(struct tas2780_context *p_context, uint16_t device_address)
{
    p_context->device_address = device_address;
    p_context->book_index = 0x0;
    p_context->page_index = 0x0;
}

void tas2780_set_page(struct tas2780_context *p_context, uint8_t page_index)
{
    p_context->page_index = page_index;
}

void tas2780_set_book(struct tas2780_context *p_context, uint8_t book_index)
{
    p_context->book_index = book_index;
}

void tas_2780_setup(struct tas2780_context *p_context)
{
    static uint8_t write_buffer[2];

    write_buffer[0] = TAS2780_PAGE_REG;
    write_buffer[1] = 0x00u; // Page 0
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_BOOK_REG;
    write_buffer[1] = 0x00u; // Go to book 0x00
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_SW_RESET_REG;
    write_buffer[1] = (0x01u << TAS2780_SW_RESET_SW_RESET_POS) & TAS2780_SW_RESET_SW_RESET_MASK; // Perform software reset
    tas2780_transmit(p_context, write_buffer, 2);

    osalThreadSleepMilliseconds(1);

    write_buffer[0] = TAS2780_PAGE_REG;
    write_buffer[1] = 0x01u; // Page 1
    tas2780_transmit(p_context, write_buffer, 2);

    // undocumented
    write_buffer[0] = 0x17u;
    write_buffer[1] = 0xC0u;
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_LSR_REG;
    write_buffer[1] = (0x01u << TAS2780_LSR_EN_LLSR_POS) & TAS2780_LSR_EN_LLSR_MASK;
    tas2780_transmit(p_context, write_buffer, 2);

    // undocumented
    write_buffer[0] = 0x21u;
    write_buffer[1] = 0x00u;
    tas2780_transmit(p_context, write_buffer, 2);

    // undocumented
    write_buffer[0] = 0x35u;
    write_buffer[1] = 0x74u;
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_PAGE_REG;
    write_buffer[1] = 0xFDu;
    tas2780_transmit(p_context, write_buffer, 2);

    // undocumented
    write_buffer[0] = 0x0Du;
    write_buffer[1] = 0x0Du;
    tas2780_transmit(p_context, write_buffer, 2);

    // undocumented
    write_buffer[0] = 0x3Eu;
    write_buffer[1] = 0x4Au;
    tas2780_transmit(p_context, write_buffer, 2);

    // undocumented
    write_buffer[0] = 0x0Du;
    write_buffer[1] = 0x00u;
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_PAGE_REG;
    write_buffer[1] = 0x00u; // Page 0
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_CHNL_0_REG;
    write_buffer[1] = ((0x03u << TAS2780_CHNL_0_CDS_MODE_POS) & TAS2780_CHNL_0_CDS_MODE_MASK) |
                      ((TAS2780_CHNL_0_AMP_LEVEL_DEFAULT << TAS2780_CHNL_0_AMP_LEVEL_POS) & TAS2780_CHNL_0_AMP_LEVEL_MASK);
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_DC_BLK0_REG;
    write_buffer[1] = ((0x01u << TAS2780_DC_BLK0_VBAT1S_MODE_POS) & TAS2780_DC_BLK0_VBAT1S_MODE_MASK) |
                      ((0x01u << TAS2780_DC_BLK0_AMP_SS_POS) & TAS2780_DC_BLK0_AMP_SS_MASK) | ((0x01u << TAS2780_DC_BLK0_HPF_FREQ_PB_POS) & TAS2780_DC_BLK0_HPF_FREQ_PB_MASK);
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_PVDD_UVLO_REG;
    write_buffer[1] = 0x0Eu; // 6.5 V
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_TDM_CFG2_REG;
    write_buffer[1] = ((0x02u << TAS2780_TDM_CFG2_RX_SLEN_POS) & TAS2780_TDM_CFG2_RX_SLEN_MASK) |
                      ((0x02u << TAS2780_TDM_CFG2_RX_WLEN_POS) & TAS2780_TDM_CFG2_RX_WLEN_MASK) |
                      ((p_context->channel << TAS2780_TDM_CFG2_RX_SCFG_POS) & TAS2780_TDM_CFG2_RX_SCFG_MASK);
    tas2780_transmit(p_context, write_buffer, 2);

    write_buffer[0] = TAS2780_MODE_CTRL_REG;
    write_buffer[1] = ((0x00 << TAS2780_MODE_CTRL_MODE_POS) & TAS2780_MODE_CTRL_MODE_MASK);
    tas2780_transmit(p_context, write_buffer, 2);
}

// bool tas2780_is_active(uint16_t address)
// {
//     uint8_t reg;
//     HAL_I2C_Mem_Read(&hi2c1, address, TAS2780_MODE_CTRL_REG, I2C_MEMADD_SIZE_8BIT, &reg, 1, 10000);
//     uint8_t state = (reg & TAS2780_MODE_CTRL_MODE_MASK) >> TAS2780_MODE_CTRL_MODE_POS;

//     return ((state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE) || (state == TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE));
// }
