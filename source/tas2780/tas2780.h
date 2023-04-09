#ifndef _TAS2780_H_
#define _TAS2780_H_

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

/**
 * @brief Converts a volume level in 8.8 binary signed fixpoint format to the
 * TAS2780 representation.
 * @details The TAS2780 understands volume levels from 0 (0 dB) to 0xC8 (-100
 * dB), where one step is 0.5 dB.
 *
 * @param _volume_8q8_db The volume level in 8.8 binary signed fixpoint format.
 */
#define TAS2780_VOLUME_FROM_8Q8_DB(_volume_8q8_db) ((uint8_t)(-(_volume_8q8_db >> 7u)))

/**
 * @brief The volume level that the TAS2780 interprets as mute.
 */
#define TAS2780_VOLUME_MUTE (0xFFu)

/**
 * @brief The volume level that the TAS2780 interprets as maximum.
 */
#define TAS2780_VOLUME_MAX (0x00u)

// Device addresses
#define TAS2780_DEVICE_ADDRESS_A (0x72u)
#define TAS2780_DEVICE_ADDRESS_B (0x74u)
#define TAS2780_DEVICE_ADDRESS_C (0x7Au)
#define TAS2780_DEVICE_ADDRESS_D (0x7Cu)

/**
 * @brief The selected channel.
 */
enum tas2780_channel {
    TAS2780_CHANNEL_LEFT,   ///< The left channel.
    TAS2780_CHANNEL_RIGHT,  ///< The right channel.
    TAS2780_CHANNEL_BOTH    ///< Both channels.
};

/**
 * @brief The context for holding information about a TAS2780 amplifier.
 */
struct tas2780_context {
    uint8_t write_buffer[2];      ///< The write buffer for I2C transactions.
    uint8_t read_buffer[2];       ///< The read buffer for I2C transactions.
    uint16_t device_address;      ///< The I2C device address.
    enum tas2780_channel channel; /**< The audio channel (left/right).
  When \a TAS2780_CHANNEL_BOTH is selected, sets up stereo mixing. */
    uint8_t analog_gain_setting;  /**< Can be between 0x00 and 0x14, where 0x14 is
     loudest.  The range of gains is 10 dB. */
};

// PAGE 0 registers
// PAGE register
#define TAS2780_PAGE_REG (0x00u)

// SW_RESET register
#define TAS2780_SW_RESET_REG (0x01)

#define TAS2780_SW_RESET_SW_RESET_POS (0u)
#define TAS2780_SW_RESET_SW_RESET_MASK (0x01u << TAS2780_SW_RESET_SW_RESET_POS)
#define TAS2780_SW_RESET_SW_RESET_DEFAULT (0x00u)

#define TAS2780_SW_RESET_RES_POS (1u)
#define TAS2780_SW_RESET_RES_MASK (0x7Fu << TAS2780_SW_RESET_RES_POS)
#define TAS2780_SW_RESET_RES_DEFAULT (0x00u)

// MODE_CTRL register
#define TAS2780_MODE_CTRL_REG (0x02u)

#define TAS2780_MODE_CTRL_MODE_POS (0u)
#define TAS2780_MODE_CTRL_MODE_MASK (0x03u << TAS2780_MODE_CTRL_MODE_POS)
#define TAS2780_MODE_CTRL_MODE_DEFAULT (0x02u)

#define TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE (0x0u)
#define TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE (0x1u)
#define TAS2780_MODE_CTRL_MODE_SW_SHUTDOWN (0x2u)
#define TAS2780_MODE_CTRL_MODE_DIAG (0x3u)
#define TAS2780_MODE_CTRL_MODE_STANDALONE_DIAG (0x4u)
#define TAS2780_MODE_CTRL_MODE_DIAG_GEN (0x5u)

#define TAS2780_MODE_CTRL_VSNS_PD_POS (3u)
#define TAS2780_MODE_CTRL_VSNS_PD_MASK (0x01u << TAS2780_MODE_CTRL_VSNS_PD_POS)
#define TAS2780_MODE_CTRL_VSNS_PD_DEFAULT (0x01u)

#define TAS2780_MODE_CTRL_ISNS_PD_POS (4u)
#define TAS2780_MODE_CTRL_ISNS_PD_MASK (0x01u << TAS2780_MODE_CTRL_ISNS_PD_POS)
#define TAS2780_MODE_CTRL_ISNS_PD_DEFAULT (0x01u)

#define TAS2780_MODE_CTRL_BOP_SRC_POS (7u)
#define TAS2780_MODE_CTRL_BOP_SRC_MASK (0x01u << TAS2780_MODE_CTRL_BOP_SRC_POS)
#define TAS2780_MODE_CTRL_BOP_SRC_DEFAULT (0x00u)

// CHNL_0 register
#define TAS2780_CHNL_0_REG (0x03u)

#define TAS2780_CHNL_0_RES_POS (0u)
#define TAS2780_CHNL_0_RES_MASK (0x01u << TAS2780_CHNL_0_RES_POS)
#define TAS2780_CHNL_0_RES_DEFAULT (0x00u)

#define TAS2780_CHNL_0_AMP_LEVEL_POS (1u)
#define TAS2780_CHNL_0_AMP_LEVEL_MASK (0x1Fu << TAS2780_CHNL_0_AMP_LEVEL_POS)
#define TAS2780_CHNL_0_AMP_LEVEL_MAX (0x14u)
#define TAS2780_CHNL_0_AMP_LEVEL_DEFAULT TAS2780_CHNL_0_AMP_LEVEL_MAX

#define TAS2780_CHNL_0_CDS_MODE_POS (6u)
#define TAS2780_CHNL_0_CDS_MODE_MASK (0x03u << TAS2780_CHNL_0_CDS_MODE_POS)
#define TAS2780_CHNL_0_CDS_MODE_DEFAULT (0x00u)

// DC_BLK0 register
#define TAS2780_DC_BLK0_REG (0x04u)

#define TAS2780_DC_BLK0_HPF_FREQ_PB_POS (0u)
#define TAS2780_DC_BLK0_HPF_FREQ_PB_MASK (0x07u << TAS2780_DC_BLK0_HPF_FREQ_PB_POS)
#define TAS2780_DC_BLK0_HPF_FREQ_PB_DEFAULT (0x01u)

#define TAS2780_DC_BLK0_AMP_SS_POS (5u)
#define TAS2780_DC_BLK0_AMP_SS_MASK (0x01u << TAS2780_DC_BLK0_AMP_SS_POS)
#define TAS2780_DC_BLK0_AMP_SS_DEFAULT (0x01u)

#define TAS2780_DC_BLK0_IRQZ_PU_POS (6u)
#define TAS2780_DC_BLK0_IRQZ_PU_MASK (0x01u << TAS2780_DC_BLK0_IRQZ_PU_POS)
#define TAS2780_DC_BLK0_IRQZ_PU_DEFAULT (0x00u)

#define TAS2780_DC_BLK0_VBAT1S_MODE_POS (7u)
#define TAS2780_DC_BLK0_VBAT1S_MODE_MASK (0x01u << TAS2780_DC_BLK0_VBAT1S_MODE_POS)
#define TAS2780_DC_BLK0_VBAT1S_MODE_DEFAULT (0x00u)

// TDM_CFG2 register
#define TAS2780_TDM_CFG2_REG (0x0Au)

#define TAS2780_TDM_CFG2_RX_SLEN_POS (0u)
#define TAS2780_TDM_CFG2_RX_SLEN_MASK (0x03u << TAS2780_TDM_CFG2_RX_SLEN_POS)
#define TAS2780_TDM_CFG2_RX_SLEN_DEFAULT (0x02u)

#define TAS2780_TDM_CFG2_RX_WLEN_POS (2u)
#define TAS2780_TDM_CFG2_RX_WLEN_MASK (0x03u << TAS2780_TDM_CFG2_RX_WLEN_POS)
#define TAS2780_TDM_CFG2_RX_WLEN_DEFAULT (0x02u)

#define TAS2780_TDM_CFG2_RX_SCFG_POS (4u)
#define TAS2780_TDM_CFG2_RX_SCFG_MASK (0x03u << TAS2780_TDM_CFG2_RX_SCFG_POS)
#define TAS2780_TDM_CFG2_RX_SCFG_DEFAULT (0x00u)

#define TAS2780_TDM_CFG2_RX_SCFG_MONO_I2C (0x0u)
#define TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT (0x1u)
#define TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT (0x2u)
#define TAS2780_TDM_CFG2_RX_SCFG_MONO_STEREO_MIX (0x3u)

#define TAS2780_TDM_CFG2_IVMON_LEN_POS (6u)
#define TAS2780_TDM_CFG2_IVMON_LEN_MASK (0x03u << TAS2780_TDM_CFG2_IVMON_LEN_POS)
#define TAS2780_TDM_CFG2_IVMON_LEN_DEFAULT (0x00u)

// DVC (digital volume control) register
#define TAS2780_DVC_REG (0x1Au)

// PVDD_UVLO register
#define TAS2780_PVDD_UVLO_REG (0x71u)

// BOOK register
#define TAS2780_BOOK_PAGE (0x00u)
#define TAS2780_BOOK_REG (0x7Fu)

// PAGE 1 registers
// LSR register
#define TAS2780_LSR_REG (0x19u)

#define TAS2780_LSR_EN_LLSR_POS (6u)
#define TAS2780_LSR_EN_LLSR_MASK (0x01u << TAS2780_LSR_EN_LLSR_POS)
#define TAS2780_LSR_EN_LLSR_DEFAULT (0x01u)

// INT_LDO register
#define TAS2780_INT_LDO_PAGE (0x1u)
#define TAS2780_INT_LDO_REG (0x36u)

void tas2780_setup_all(void);
void tas2780_set_volume_all(int16_t volume_8q8_db, enum tas2780_channel channel);
void tas2780_init(struct tas2780_context *p_context, uint16_t device_address);
void tas2780_ensure_active_all(void);

#endif  // _TAS2780_H_
