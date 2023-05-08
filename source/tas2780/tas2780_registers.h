// Copyright2023 elagil

/**
 * @file
 * @brief   TI TAS2780 amplifier register header.
 *
 * @addtogroup tas2780
 * @{
 */

#ifndef SOURCE_TAS2780_TAS2780_REGISTERS_H_
#define SOURCE_TAS2780_TAS2780_REGISTERS_H_

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

// Definitions for masking a chosen number of bits.
#define BIT_MASK_1 (0x01)
#define BIT_MASK_2 (0x03)
#define BIT_MASK_3 (0x07)
#define BIT_MASK_4 (0x0F)
#define BIT_MASK_5 (0x1F)
#define BIT_MASK_6 (0x3F)
#define BIT_MASK_7 (0x7F)
#define BIT_MASK_8 (0xFF)

/**
 * @brief Converts a volume level in 8.8 binary signed fixpoint format to the TAS2780 representation.
 * @details The TAS2780 understands volume levels from 0 (0 dB) to 0xC8 (-100 dB), where one step is 0.5 dB.
 *
 * @param _volume_8q8_db The volume level in 8.8 binary signed fixpoint format.
 */
#define TAS2780_VOLUME_FROM_8Q8_DB(_volume_8q8_db) ((uint8_t)(-(_volume_8q8_db >> 7u)))

// PAGE register
#define TAS2780_PAGE_REG           (0x00u)
#define TAS2780_DEFAULT_PAGE_INDEX (0x00u)

// PAGE 0 registers
// SW_RESET register
#define TAS2780_SW_RESET_REG              (0x01)

#define TAS2780_SW_RESET_SW_RESET_POS     (0u)
#define TAS2780_SW_RESET_SW_RESET_MASK    (BIT_MASK_1 << TAS2780_SW_RESET_SW_RESET_POS)
#define TAS2780_SW_RESET_SW_RESET_DEFAULT (0x00u)

#define TAS2780_SW_RESET_RES_POS          (1u)
#define TAS2780_SW_RESET_RES_MASK         (BIT_MASK_7 << TAS2780_SW_RESET_RES_POS)
#define TAS2780_SW_RESET_RES_DEFAULT      (0x00u)

// MODE_CTRL register
#define TAS2780_MODE_CTRL_REG                      (0x02u)

#define TAS2780_MODE_CTRL_MODE_POS                 (0u)
#define TAS2780_MODE_CTRL_MODE_MASK                (BIT_MASK_2 << TAS2780_MODE_CTRL_MODE_POS)
#define TAS2780_MODE_CTRL_MODE_DEFAULT             (0x02u)

#define TAS2780_MODE_CTRL_MODE_ACTIVE_WITHOUT_MUTE (0x0u)
#define TAS2780_MODE_CTRL_MODE_ACTIVE_WITH_MUTE    (0x1u)
#define TAS2780_MODE_CTRL_MODE_SW_SHUTDOWN         (0x2u)
#define TAS2780_MODE_CTRL_MODE_DIAG                (0x3u)
#define TAS2780_MODE_CTRL_MODE_STANDALONE_DIAG     (0x4u)
#define TAS2780_MODE_CTRL_MODE_DIAG_GEN            (0x5u)

#define TAS2780_MODE_CTRL_VSNS_PD_POS              (3u)
#define TAS2780_MODE_CTRL_VSNS_PD_MASK             (BIT_MASK_1 << TAS2780_MODE_CTRL_VSNS_PD_POS)
#define TAS2780_MODE_CTRL_VSNS_PD_DEFAULT          (0x01u)

#define TAS2780_MODE_CTRL_ISNS_PD_POS              (4u)
#define TAS2780_MODE_CTRL_ISNS_PD_MASK             (BIT_MASK_1 << TAS2780_MODE_CTRL_ISNS_PD_POS)
#define TAS2780_MODE_CTRL_ISNS_PD_DEFAULT          (0x01u)

#define TAS2780_MODE_CTRL_BOP_SRC_POS              (7u)
#define TAS2780_MODE_CTRL_BOP_SRC_MASK             (BIT_MASK_1 << TAS2780_MODE_CTRL_BOP_SRC_POS)
#define TAS2780_MODE_CTRL_BOP_SRC_DEFAULT          (0x00u)

// CHNL_0 register
#define TAS2780_CHNL_0_REG               (0x03u)

#define TAS2780_CHNL_0_RES_POS           (0u)
#define TAS2780_CHNL_0_RES_MASK          (BIT_MASK_1 << TAS2780_CHNL_0_RES_POS)
#define TAS2780_CHNL_0_RES_DEFAULT       (0x00u)

#define TAS2780_CHNL_0_AMP_LEVEL_POS     (1u)
#define TAS2780_CHNL_0_AMP_LEVEL_MASK    (BIT_MASK_5 << TAS2780_CHNL_0_AMP_LEVEL_POS)

#define TAS2780_CHNL_0_AMP_LEVEL_MIN     (0x00u)  ///< 11 dBV gain (48 kHz), 9 dBV gain (96 kHz)
#define TAS2780_CHNL_0_AMP_LEVEL_MAX     (0x14u)  ///< 21 dBV gain (48 kHz), 19 dBV gain (96 kHz)
#define TAS2780_CHNL_0_AMP_LEVEL_DEFAULT TAS2780_CHNL_0_AMP_LEVEL_MAX

#define TAS2780_CHNL_0_CDS_MODE_POS      (6u)
#define TAS2780_CHNL_0_CDS_MODE_MASK     (BIT_MASK_2 << TAS2780_CHNL_0_CDS_MODE_POS)
#define TAS2780_CHNL_0_CDS_MODE_DEFAULT  (0x03u)  ///< Y-Bridge, low power on VBAT1S

// DC_BLK0 register
#define TAS2780_DC_BLK0_REG                 (0x04u)

#define TAS2780_DC_BLK0_HPF_FREQ_PB_POS     (0u)
#define TAS2780_DC_BLK0_HPF_FREQ_PB_MASK    (BIT_MASK_3 << TAS2780_DC_BLK0_HPF_FREQ_PB_POS)
#define TAS2780_DC_BLK0_HPF_FREQ_PB_DEFAULT (0x01u)

#define TAS2780_DC_BLK0_AMP_SS_POS          (5u)
#define TAS2780_DC_BLK0_AMP_SS_MASK         (BIT_MASK_1 << TAS2780_DC_BLK0_AMP_SS_POS)
#define TAS2780_DC_BLK0_AMP_SS_DEFAULT      (0x01u)

#define TAS2780_DC_BLK0_IRQZ_PU_POS         (6u)
#define TAS2780_DC_BLK0_IRQZ_PU_MASK        (BIT_MASK_1 << TAS2780_DC_BLK0_IRQZ_PU_POS)
#define TAS2780_DC_BLK0_IRQZ_PU_DEFAULT     (0x00u)

#define TAS2780_DC_BLK0_VBAT1S_MODE_POS     (7u)
#define TAS2780_DC_BLK0_VBAT1S_MODE_MASK    (BIT_MASK_1 << TAS2780_DC_BLK0_VBAT1S_MODE_POS)
#define TAS2780_DC_BLK0_VBAT1S_MODE_DEFAULT (0x00u)

// TDM_CFG2 register
#define TAS2780_TDM_CFG2_REG                     (0x0Au)

#define TAS2780_TDM_CFG2_RX_SLEN_POS             (0u)
#define TAS2780_TDM_CFG2_RX_SLEN_MASK            (BIT_MASK_2 << TAS2780_TDM_CFG2_RX_SLEN_POS)
#define TAS2780_TDM_CFG2_RX_SLEN_DEFAULT         (0x03u)  ///< 32 bit word length.

#define TAS2780_TDM_CFG2_RX_WLEN_POS             (2u)
#define TAS2780_TDM_CFG2_RX_WLEN_MASK            (BIT_MASK_2 << TAS2780_TDM_CFG2_RX_WLEN_POS)
#define TAS2780_TDM_CFG2_RX_WLEN_DEFAULT         (0x03u)  ///< 32 bit word length.

#define TAS2780_TDM_CFG2_RX_SCFG_MONO_I2C        (0x0u)  ///< TDM channel selection by I2C address.
#define TAS2780_TDM_CFG2_RX_SCFG_MONO_LEFT       (0x1u)  ///< TDM channel selection: left.
#define TAS2780_TDM_CFG2_RX_SCFG_MONO_RIGHT      (0x2u)  ///< TDM channel selection: right.
#define TAS2780_TDM_CFG2_RX_SCFG_MONO_STEREO_MIX (0x3u)  ///< TDM channel selection: stereo mix of left/right.

#define TAS2780_TDM_CFG2_RX_SCFG_POS             (4u)
#define TAS2780_TDM_CFG2_RX_SCFG_MASK            (BIT_MASK_2 << TAS2780_TDM_CFG2_RX_SCFG_POS)
#define TAS2780_TDM_CFG2_RX_SCFG_DEFAULT         (TAS2780_TDM_CFG2_RX_SCFG_MONO_I2C)

#define TAS2780_TDM_CFG2_IVMON_LEN_POS           (6u)
#define TAS2780_TDM_CFG2_IVMON_LEN_MASK          (BIT_MASK_2 << TAS2780_TDM_CFG2_IVMON_LEN_POS)
#define TAS2780_TDM_CFG2_IVMON_LEN_DEFAULT       (0x00u)

// TDM_CFG3 register
#define TAS2780_TDM_CFG3_REG               (0x0Cu)

#define TAS2780_TDM_CFG3_RX_SLOT_L_POS     (0u)
#define TAS2780_TDM_CFG3_RX_SLOT_L_MASK    (BIT_MASK_4 << TAS2780_TDM_CFG3_RX_SLOT_L_POS)
#define TAS2780_TDM_CFG3_RX_SLOT_L_DEFAULT (0x00u)

#define TAS2780_TDM_CFG3_RX_SLOT_R_POS     (4u)
#define TAS2780_TDM_CFG3_RX_SLOT_R_MASK    (BIT_MASK_4 << TAS2780_TDM_CFG3_RX_SLOT_R_POS)
#define TAS2780_TDM_CFG3_RX_SLOT_R_DEFAULT (0x01u)

// NG_CFG0 register (noise gate)
#define TAS2780_NG_CFG0_REG             (0x35u)

#define TAS2780_NG_CFG0_RES_POS         (0u)
#define TAS2780_NG_CFG0_RES_MASK        (BIT_MASK_2 << TAS2780_NG_CFG0_RES_POS)
#define TAS2780_NG_CFG0_RES_DEFAULT     (0x01u)

#define TAS2780_NG_CFG0_NG_EN_POS       (2u)
#define TAS2780_NG_CFG0_NG_EN_MASK      (BIT_MASK_1 << TAS2780_NG_CFG0_NG_EN_POS)
#define TAS2780_NG_CFG0_NG_EN_DEFAULT   (0x01u)  ///< Enabled.

#define TAS2780_NG_CFG0_NG_LVL_POS      (3u)
#define TAS2780_NG_CFG0_NG_LVL_MASK     (BIT_MASK_2 << TAS2780_NG_CFG0_NG_LVL_POS)
#define TAS2780_NG_CFG0_NG_LVL_DEFAULT  (0x00u)  ///< 0x00 ... 0x03 (-90 dBFS ... -120 dBFS)

#define TAS2780_NG_CFG0_NG_HYST_POS     (5u)
#define TAS2780_NG_CFG0_NG_HYST_MASK    (BIT_MASK_3 << TAS2780_NG_CFG0_NG_HYST_POS)
#define TAS2780_NG_CFG0_NG_HYST_DEFAULT (0x07u)  ///< 1000 ms.

// INT_LIVE1 register
#define TAS2780_INT_LIVE1_REG         (0x43u)

#define TAS2780_INT_LIVE1_IL_NGA_POS  (2u)
#define TAS2780_INT_LIVE1_IL_NGA_MASK (BIT_MASK_1 << TAS2780_INT_LIVE1_IL_NGA_POS)

// DVC (digital volume control) register
#define TAS2780_DVC_REG (0x1Au)

// PVDD_UVLO register
#define TAS2780_PVDD_UVLO_REG (0x71u)

// BOOK register
#define TAS2780_BOOK_PAGE          (0x00u)
#define TAS2780_BOOK_REG           (0x7Fu)
#define TAS2780_DEFAULT_BOOK_INDEX (0x00u)

// PAGE 1 registers
// LSR register
#define TAS2780_LSR_REG             (0x19u)

#define TAS2780_LSR_EN_LLSR_POS     (6u)
#define TAS2780_LSR_EN_LLSR_MASK    (BIT_MASK_1 << TAS2780_LSR_EN_LLSR_POS)
#define TAS2780_LSR_EN_LLSR_DEFAULT (0x01u)

// INT_LDO register
#define TAS2780_INT_LDO_PAGE (0x1u)
#define TAS2780_INT_LDO_REG  (0x36u)

#endif  // SOURCE_TAS2780_TAS2780_REGISTERS_H_

/**
 * @}
 */
