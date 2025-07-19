/**
  ******************************************************************************
  * @file    gap_gate_driver_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          GAP component of the Motor Control SDK that provides support 
  *          the STGAPxx galvanically isolated gate drivers family.
  * 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup GAP_GATE_DRIVER_CTRL
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GAP_GATE_DRIVER_CTR_H
#define __GAP_GATE_DRIVER_CTR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup GAP_GATE_DRIVER_CTRL
  * @{
  */

#define GPIO_NoRemap_SPI                            ((uint32_t)(0))
#define MAX_DEVICES_NUMBER                          7                    /**< @brief   Number of GAP used in the project. */

#define CFG1_REG_MASK                               (uint8_t)(0xFF) /**< @brief Data mask for CFG1 register. */
#define CFG2_REG_MASK                               (uint8_t)(0xFF) /**< @brief Data mask for CFG2 register. */
#define CFG3_REG_MASK                               (uint8_t)(0xFF) /**< @brief Data mask for CFG3 register. */
#define CFG4_REG_MASK                               (uint8_t)(0x3F) /**< @brief Data mask for CFG4 register. */
#define CFG5_REG_MASK                               (uint8_t)(0x0F) /**< @brief Data mask for CFG5 register. */
#define STATUS1_REG_MASK                            (uint8_t)(0xFF) /**< @brief Data mask for STATUS1 register. */
#define STATUS2_REG_MASK                            (uint8_t)(0x06) /**< @brief Data mask for STATUS2 register. */
#define STATUS3_REG_MASK                            (uint8_t)(0x1F) /**< @brief Data mask for STATUS3 register. */
#define TEST1_REG_MASK                              (uint8_t)(0x1F) /**< @brief Data mask for TEST1 register. */
#define DIAG1_REG_MASK                              (uint8_t)(0xFF) /**< @brief Data mask for DIAG1 register. */
#define DIAG2_REG_MASK                              (uint8_t)(0xFF) /**< @brief Data mask for DIAG2 register. */

#define GAP_ERROR_CLEAR                             (uint32_t)(0x00000000) /**< @brief No ERROR occured. */
#define GAP_ERROR_CODE_UVLOD                        (uint32_t)(0x00000001) /**< @brief Under Voltage Lockout ERROR occured. */
#define GAP_ERROR_CODE_OVLOD                        (uint32_t)(0x00000002) /**< @brief Over Voltage Lockout ERROR occured. */
#define GAP_ERROR_CODE_REGERRL                      (uint32_t)(0x00000004) /**< @brief Configuration procedure occured. */
#define GAP_ERROR_CODE_SPI_ERR                      (uint32_t)(0x00000008) /**< @brief SPI communication ERROR occured. */
#define GAP_ERROR_CODE_DT_ERR                       (uint32_t)(0x00000010) /**< @brief Deadtime ERROR occured. */
#define GAP_ERROR_CODE_CFG                          (uint32_t)(0x00000020) /**< @brief Configuration ERROR occured. */
#define GAP_ERROR_CODE_GATE                         (uint32_t)(0x00000100) /**< @brief GATE path ERROR occured. */
#define GAP_ERROR_CODE_ASC                          (uint32_t)(0x00000200) /**< @brief Asynchronous stop command ERROR occured. */
#define GAP_ERROR_CODE_REGERRR                      (uint32_t)(0x00000400) /**< @brief Configuration procedure occured. */
#define GAP_ERROR_CODE_TWN                          (uint32_t)(0x00010000) /**< @brief Temperature warning threshold ERROR occured. */
#define GAP_ERROR_CODE_TSD                          (uint32_t)(0x00020000) /**< @brief Temperature threshold ERROR occured. */
#define GAP_ERROR_CODE_UVLOL                        (uint32_t)(0x00040000) /**< @brief Under Voltage Lockout on VL ERROR occured. */
#define GAP_ERROR_CODE_UVLOH                        (uint32_t)(0x00080000) /**< @brief Under Voltage Lockout on VH ERROR occured. */
#define GAP_ERROR_CODE_SENSE                        (uint32_t)(0x00100000) /**< @brief sensor SENS voltage ERROR occured. */
#define GAP_ERROR_CODE_DESAT                        (uint32_t)(0x00200000) /**< @brief sensor Desaturation protection ERROR occured. */
#define GAP_ERROR_CODE_OVLOL                        (uint32_t)(0x00400000) /**< @brief Over Voltage Lockout on VL ERROR occured. */
#define GAP_ERROR_CODE_OVLOH                        (uint32_t)(0x00800000) /**< @brief Over Voltage Lockout on VH ERROR occured. */
#define GAP_ERROR_CODE_SPI_CRC                      (uint32_t)(0x40000000) /**< @brief CRC SPI ERROR occured. */
#define GAP_ERROR_CODE_DEVICES_NOT_PROGRAMMABLE     (uint32_t)(0x80000000) /**< @brief Device access ERROR occured. */

#define GAP_CFG1_CRC_SPI                            (uint8_t)(0x80) /**< @brief CFG1 register: SPI communication protocol CRC enable. */
#define GAP_CFG1_UVLOD                              (uint8_t)(0x40) /**< @brief CFG1 register: UVLO protection on VDD supply voltage enable. */
#define GAP_CFG1_SD_FLAG                            (uint8_t)(0x20) /**< @brief CFG1 register: SD pin functionality. */
#define GAP_CFG1_DIAG_EN                            (uint8_t)(0x10) /**< @brief CFG1 register: IN-/DIAG2 pin functionality. */
#define GAP_CFG1_DT_DISABLE                         (uint8_t)(0x00) /**< @brief CFG1 register: No deadtime value. */
#define GAP_CFG1_DT_250NS                           (uint8_t)(0x04) /**< @brief CFG1 register: 250ns deadtime value. */
#define GAP_CFG1_DT_800NS                           (uint8_t)(0x08) /**< @brief CFG1 register: 800ns deadtime value. */
#define GAP_CFG1_DT_1200NS                          (uint8_t)(0x0C) /**< @brief CFG1 register: 1200ns deadtime value. */
#define GAP_CFG1_INFILTER_DISABLE                   (uint8_t)(0x00) /**< @brief CFG1 register: No Input deglitch time value. */
#define GAP_CFG1_INFILTER_210NS                     (uint8_t)(0x01) /**< @brief CFG1 register: 210ns Input deglitch time value. */
#define GAP_CFG1_INFILTER_560NS                     (uint8_t)(0x02) /**< @brief CFG1 register: 560ns Input deglitch time value. */
#define GAP_CFG1_INFILTER_70NS                      (uint8_t)(0x03) /**< @brief CFG1 register: 70ns Input deglitch time value. */

#define GAP_CFG2_SENSETH_100MV                      (uint8_t)(0x00) /**< @brief CFG2 register: 100mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_125MV                      (uint8_t)(0x20) /**< @brief CFG2 register: 125mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_150MV                      (uint8_t)(0x40) /**< @brief CFG2 register: 150mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_175MV                      (uint8_t)(0x60) /**< @brief CFG2 register: 175mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_200MV                      (uint8_t)(0x80) /**< @brief CFG2 register: 200mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_250MV                      (uint8_t)(0xA0) /**< @brief CFG2 register: 250mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_300MV                      (uint8_t)(0xC0) /**< @brief CFG2 register: 300mV SENSE comparator threshold value. */
#define GAP_CFG2_SENSETH_400MV                      (uint8_t)(0xE0) /**< @brief CFG2 register: 400mV SENSE comparator threshold value. */
#define GAP_CFG2_DESATCURR_250UA                    (uint8_t)(0x00) /**< @brief CFG2 register: 250uA current value sourced by the DESAT. */
#define GAP_CFG2_DESATCURR_500UA                    (uint8_t)(0x08) /**< @brief CFG2 register: 500uA current value sourced by the DESAT. */
#define GAP_CFG2_DESATCURR_750UA                    (uint8_t)(0x10) /**< @brief CFG2 register: 750uA current value sourced by the DESAT. */
#define GAP_CFG2_DESATCURR_1000UA                   (uint8_t)(0x18) /**< @brief CFG2 register: 1000uA current value sourced by the DESAT. */
#define GAP_CFG2_DESATTH_3V                         (uint8_t)(0x00) /**< @brief CFG2 register: 3V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_4V                         (uint8_t)(0x01) /**< @brief CFG2 register: 4V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_5V                         (uint8_t)(0x02) /**< @brief CFG2 register: 5V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_6V                         (uint8_t)(0x03) /**< @brief CFG2 register: 6V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_7V                         (uint8_t)(0x04) /**< @brief CFG2 register: 7V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_8V                         (uint8_t)(0x05) /**< @brief CFG2 register: 8V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_9V                         (uint8_t)(0x06) /**< @brief CFG2 register: 9V DESAT comparator threshold value. */
#define GAP_CFG2_DESATTH_10V                        (uint8_t)(0x07) /**< @brief CFG2 register: 10V DESAT comparator threshold value. */

#define GAP_CFG3_2LTOTH_7_0V                        (uint8_t)(0x00) /**< @brief CFG3 register: 7V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_7_5V                        (uint8_t)(0x10) /**< @brief CFG3 register: 7.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_8_0V                        (uint8_t)(0x20) /**< @brief CFG3 register: 8V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_8_5V                        (uint8_t)(0x30) /**< @brief CFG3 register: 8.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_9_0V                        (uint8_t)(0x40) /**< @brief CFG3 register: 9V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_9_5V                        (uint8_t)(0x50) /**< @brief CFG3 register: 9.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_10_0V                       (uint8_t)(0x60) /**< @brief CFG3 register: 10V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_10_5V                       (uint8_t)(0x70) /**< @brief CFG3 register: 10.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_11_0V                       (uint8_t)(0x80) /**< @brief CFG3 register: 11V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_11_5V                       (uint8_t)(0x90) /**< @brief CFG3 register: 11.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_12_0V                       (uint8_t)(0xA0) /**< @brief CFG3 register: 12V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_12_5V                       (uint8_t)(0xB0) /**< @brief CFG3 register: 12.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_13_0V                       (uint8_t)(0xC0) /**< @brief CFG3 register: 13V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_13_5V                       (uint8_t)(0xD0) /**< @brief CFG3 register: 13.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_14_0V                       (uint8_t)(0xE0) /**< @brief CFG3 register: 14V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTH_14_5V                       (uint8_t)(0xF0) /**< @brief CFG3 register: 14.5V threshold voltage value which is actively forced during the 2-level turnoff sequence. */
#define GAP_CFG3_2LTOTIME_DISABLE                   (uint8_t)(0x00) /**< @brief CFG3 register: No duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_0_75_US                   (uint8_t)(0x01) /**< @brief CFG3 register: 0.75us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_1_00_US                   (uint8_t)(0x02) /**< @brief CFG3 register: 1us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_1_50_US                   (uint8_t)(0x03) /**< @brief CFG3 register: 1.5us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_2_00_US                   (uint8_t)(0x04) /**< @brief CFG3 register: 2us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_2_50_US                   (uint8_t)(0x05) /**< @brief CFG3 register: 2.5us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_3_00_US                   (uint8_t)(0x06) /**< @brief CFG3 register: 3us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_3_50_US                   (uint8_t)(0x07) /**< @brief CFG3 register: 3.5us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_3_75_US                   (uint8_t)(0x08) /**< @brief CFG3 register: 3.75us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_4_00_US                   (uint8_t)(0x09) /**< @brief CFG3 register: 4us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_4_25_US                   (uint8_t)(0x0A) /**< @brief CFG3 register: 4.25us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_4_50_US                   (uint8_t)(0x0B) /**< @brief CFG3 register: 4.5us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_4_75_US                   (uint8_t)(0x0C) /**< @brief CFG3 register: 4.75us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_5_00_US                   (uint8_t)(0x0D) /**< @brief CFG3 register: 5us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_5_25_US                   (uint8_t)(0x0E) /**< @brief CFG3 register: 5.25us duration time value of the 2-level turn-off sequence. */
#define GAP_CFG3_2LTOTIME_5_50_US                   (uint8_t)(0x0F) /**< @brief CFG3 register: 5.5us duration time value of the 2-level turn-off sequence. */

#define GAP_CFG4_OVLO                               (uint8_t)(0x20) /**< @brief CFG4 register: enables the OVLO protection on the VH and VL power supply. */
#define GAP_CFG4_UVLOLATCH                          (uint8_t)(0x10) /**< @brief CFG4 register: sets if the UVLO is latched or not. */
#define GAP_CFG4_UVLOTH_VH_DISABLE                  (uint8_t)(0x00) /**< @brief CFG4 register: disables the UVLO threshold on the positive power supply. */
#define GAP_CFG4_UVLOTH_VH_10V                      (uint8_t)(0x01) /**< @brief CFG4 register: sets the VH positive supply voltage UVLO threshold to 10v on the positive power supply. */
#define GAP_CFG4_UVLOTH_VH_12V                      (uint8_t)(0x02) /**< @brief CFG4 register: sets the VH positive supply voltage UVLO threshold to 12v on the positive power supply. */
#define GAP_CFG4_UVLOTH_VH_14V                      (uint8_t)(0x03) /**< @brief CFG4 register: sets the VH positive supply voltage UVLO threshold to 140v on the positive power supply. */
#define GAP_CFG4_UVLOTH_VL_DISABLE                  (uint8_t)(0x00) /**< @brief CFG4 register: disables the UVLO threshold on the negative power supply. */
#define GAP_CFG4_UVLOTH_VL_N3V                      (uint8_t)(0x04) /**< @brief CFG4 register: sets the VL negative supply voltage UVLO threshold to -3v on the negative power supply. */
#define GAP_CFG4_UVLOTH_VL_N5V                      (uint8_t)(0x08) /**< @brief CFG4 register: sets the VL negative supply voltage UVLO threshold to -5v on the negative power supply. */
#define GAP_CFG4_UVLOTH_VL_N7V                      (uint8_t)(0x0C) /**< @brief CFG4 register: sets the VL negative supply voltage UVLO threshold to -7v on the negative power supply. */

#define GAP_CFG5_2LTO_ON_FAULT                      (uint8_t)(0x08) /**< @brief CFG5 register: 2LTO active only after a fault event. */
#define GAP_CFG5_CLAMP_EN                           (uint8_t)(0x04) /**< @brief CFG5 register: enables enable the Miller clamp feature. */
#define GAP_CFG5_DESAT_EN                           (uint8_t)(0x02) /**< @brief CFG5 register: enables the DESAT desaturation protection function comparator. */
#define GAP_CFG5_SENSE_EN                           (uint8_t)(0x01) /**< @brief CFG5 register: enables the sense overcurrent function comparator. */

#define GAP_STATUS1_OVLOH                           (uint8_t)(0x80) /**< @brief STATUS1 register: VH overvoltage flag is forced high when VH is over OVVHoff threshold. */
#define GAP_STATUS1_OVLOL                           (uint8_t)(0x40) /**< @brief STATUS1 register: VL overvoltage flag is forced high when VL is over OVVLoff threshold. */
#define GAP_STATUS1_DESAT                           (uint8_t)(0x20) /**< @brief STATUS1 register: Desaturation flag is forced high when DESAT pin voltage reach VDESATth threshold. */
#define GAP_STATUS1_SENSE                           (uint8_t)(0x10) /**< @brief STATUS1 register: Sense flag is forced high when SENSE pin voltage reach VSENSEth threshold. */
#define GAP_STATUS1_UVLOH                           (uint8_t)(0x08) /**< @brief STATUS1 register: VH undervoltage flag is forced high when VH is below VHoff threshold. */
#define GAP_STATUS1_UVLOL                           (uint8_t)(0x04) /**< @brief STATUS1 register: VL undervoltage flag is forced high when VL is over VLoff threshold. */
#define GAP_STATUS1_TSD                             (uint8_t)(0x02) /**< @brief STATUS1 register: Thermal shutdown protection flag is forced high when overtemperature shutdown threshold is reached.*/
#define GAP_STATUS1_TWN                             (uint8_t)(0x01) /**< @brief STATUS1 register: Thermal warning flag is forced high when overtemperature shutdown threshold is reached. */

#define GAP_STATUS2_REGERRR                         (uint8_t)(0x02) /**< @brief STATUS2 register: Register or communication error on isolated side is forced high when:
                                                                           1 Programming procedure is not correctly performed.
                                                                           2 Isolated interface communication fails.
                                                                           3 An unexpected register value change occurs in one of the remote registers.
                                                                        It is also latched at power-up/reset and from Sleep state. */
#define GAP_STATUS2_ASC                             (uint8_t)(0x01) /**< @brief STATUS2 register: ASC pin status. When ASC pin is high the flag reports '1', otherwise is '0'.. */

#define GAP_STATUS3_CFG                             (uint8_t)(0x20) /**< @brief STATUS3 register: CFG error flag. */
#define GAP_STATUS3_DT_ERR                          (uint8_t)(0x10) /**< @brief STATUS3 register: Deadtime error flag is forced high when a violation of internal DT is detected. */
#define GAP_STATUS3_SPI_ERR                         (uint8_t)(0x08) /**< @brief STATUS3 register: SPI communication error flag is forced high when the SPI communication fails cause:
                                                                          1 Wrong CRC check.
                                                                          2  Wrong number of CK rising edges.
                                                                          3 Attempt to execute a not-allowed command.
                                                                          4 Attempt to read, write or reset at a not available address. */
#define GAP_STATUS3_REGERRL                         (uint8_t)(0x04) /**< @brief STATUS3 register: Register or communication error on low voltage side is forced high when: -
                                                                          1 Programming procedure is not correctly performed.
                                                                          2 Isolated interface communication fails.
                                                                          3 An unexpected register value change occurs in one of the remote registers.
                                                                      * It is latched at power-up/reset also. */
#define GAP_STATUS3_OVLOD                           (uint8_t)(0x02) /**< @brief STATUS3 register: VDD overvoltage flag is forced high when VDD is over OVVDDoff threshold. */
#define GAP_STATUS3_UVLOD                           (uint8_t)(0x01) /**< @brief STATUS3 register: VDD undervoltage flag is forced high when VDD is below VDDon threshold.
                                                                         It is latched at power-up/reset also. */

#define GAP_TEST1_GOFFCHK                           (uint8_t)(0x10) /**< @brief TEST1 register: enables the GOFF to gate path check mode */
#define GAP_TEST1_GONCHK                            (uint8_t)(0x08) /**< @brief TEST1 register: enables the GON to gate path check mode */
#define GAP_TEST1_DESCHK                            (uint8_t)(0x04) /**< @brief TEST1 register: enables the DESAT comparator check mode */
#define GAP_TEST1_SNSCHK                            (uint8_t)(0x02) /**< @brief TEST1 register: enables the SENSE comparator check mode */
#define GAP_TEST1_RCHK                              (uint8_t)(0x01) /**< @brief TEST1 register: enables the SENSE resistor check mode */

#define GAP_DIAG_SPI_REGERR                         (uint8_t)(0x80) /**< @brief DIAG register: enables the DIAGxCFG event SPI communication error or register failure */
#define GAP_DIAG_UVLOD_OVLOD                        (uint8_t)(0x40) /**< @brief DIAG register: enables the DIAGxCFG event VDD power supply failure */
#define GAP_DIAG_UVLOH_UVLOL                        (uint8_t)(0x20) /**< @brief DIAG register: enables the DIAGxCFG event Undervoltage failure */
#define GAP_DIAG_OVLOH_OVLOL                        (uint8_t)(0x10) /**< @brief DIAG register: enables the DIAGxCFG event Overvoltage failure */
#define GAP_DIAG_DESAT_SENSE                        (uint8_t)(0x08) /**< @brief DIAG register: enables the DIAGxCFG event SDesaturation and sense detection */
#define GAP_DIAG_ASC_DT_ERR                         (uint8_t)(0x04) /**< @brief DIAG register: enables the DIAGxCFG event ASC feedback */
#define GAP_DIAG_TSD                                (uint8_t)(0x02) /**< @brief DIAG register: enables the DIAGxCFG event Thermal shutdown */
#define GAP_DIAG_TWN                                (uint8_t)(0x01) /**< @brief DIAG register: enables the DIAGxCFG event Thermal warning */
#define GAP_DIAG_NONE                               (uint8_t)(0x00) /**< @brief DIAG register: No DIAGxCFG event. */

/** Define the GAP register enum.
  *
  */
typedef enum
{
  CFG1    = 0x0C,               /*!< Address of CFG1 register (low voltage side). */
  CFG2    = 0x1D,               /*!< Address of CFG2 register (isolated side). */
  CFG3    = 0x1E,               /*!< Address of CFG3 register (isolated side). */
  CFG4    = 0x1F,               /*!< Address of CFG4 register (isolated side). */
  CFG5    = 0x19,               /*!< Address of CFG5 register (isolated side). */
  STATUS1 = 0x02,               /*!< Address of STATUS1 register (low voltage side).
                                     The STATUS1 is a read only register that reports some device failure flags. */
  STATUS2 = 0x01,               /*!< Address of STATUS2 register (low voltage side). */
  STATUS3 = 0x0A,               /*!< Address of STATUS3 register (low voltage side). */
  TEST1   = 0x11,               /*!< Address of TEST1 register (isolated side). */
  DIAG1   = 0x05,               /*!< Address of DIAG1CFG registers (low voltage side). */
  DIAG2   = 0x06                /*!< Address of DIAG2CFG registers (low voltage side). */
} GAP_Registers_Handle_t;

/** Define test modes
  *
  */
typedef enum
{
  SENSE_RESISTOR_CHK,           /*!< Security check to verify the connection between the device and the sense shunt
                                     resistor and to verify the optional sense resistor filter network is not open. */
  SENSE_COMPARATOR_CHK,         /*!< Security check to verify the functionality of the sense comparator. */
  GON_CHK,                      /*!< Security check to verify the path integrity including the driver's GON output,
                                     the GON (turn-on) gate resistor, the power switch gate and the CLAMP pin. */
  GOFF_CHK,                     /*!< Security check to verify the path integrity including the driver's GOFF output,
                                     the GOFF (turn-off) gate resistor, the power switch gate and the CLAMP pin. */
  DESAT_CHK                     /*!< Security check to verify the functionality of the de-saturation. */
} GAP_TestMode_t;

/**
  * @brief  GAP class register bank definition
  *
  * This structure contains configuration value for register bank of the GAP driver.
  *
  */
typedef struct
{
  uint8_t CFG1;                 /*!< Configuration value for CFG1 register. */
  uint8_t CFG2;                 /*!< Configuration value for CFG2 register. */
  uint8_t CFG3;                 /*!< Configuration value for CFG3 register. */
  uint8_t CFG4;                 /*!< Configuration value for CFG4 register. */
  uint8_t CFG5;                 /*!< Configuration value for CFG5 register. */
  uint8_t DIAG1;                /*!< Configuration value for DIAG1 register. */
  uint8_t DIAG2;                /*!< Configuration value for DIAG2 register. */
} GAP_DeviceParams_Handle_t;

/**
  * @brief Handle of the GAP component
  *
  * This structure holds all the parameters needed to access and manage GAP drivers.
  */
typedef struct
{
  uint8_t       DeviceNum;                                    /*!< Number of GAP used in the daisy chain. */
  GAP_DeviceParams_Handle_t DeviceParams[MAX_DEVICES_NUMBER]; /*!< Device parameters pointers */
  /* SPI related parameters. */
  SPI_TypeDef  *SPIx;
  GPIO_TypeDef *NCSPort;                                      /*!< GPIO port used by NCS. It must be equal to GPIOx x= A, B, ...*/
  uint16_t      NCSPin;                                       /*!< GPIO pin used by NCS. It must be equal to GPIO_Pin_x x= 0, 1, ...*/
  GPIO_TypeDef *NSDPort;                                      /*!< GPIO port used by NSD. It must be equal to GPIOx x= A, B, ...*/
  uint16_t      NSDPin;                                       /*!< GPIO pin used by NSD. It must be equal to GPIO_Pin_x x= 0, 1, ...*/
  uint32_t      GAP_ErrorsNow[MAX_DEVICES_NUMBER];            /*!< Bitfield with extra error codes that is currently active.
                                                                   The error code available are listed here (TBF).*/
  uint32_t      GAP_ErrorsOccurred[MAX_DEVICES_NUMBER];       /*!< Bitfield with extra error codes that occurs and is over.
                                                                   The error code available  are listed here (TBF).*/
} GAP_Handle_t;


/* Exported functions ------------------------------------------------------- */

bool GAP_CheckErrors(GAP_Handle_t *pHandle, uint32_t *error_now, uint32_t *error_occurred);
void GAP_FaultAck(GAP_Handle_t *pHandle);
bool GAP_Configuration(GAP_Handle_t *pHandle);
bool GAP_IsDevicesProgrammed(GAP_Handle_t *pHandle);
bool GAP_DevicesConfiguration(GAP_Handle_t *pHandle);
uint16_t GAP_CRCCalculate(uint8_t data, uint8_t crc_initial_value);
bool GAP_CRCCheck(uint8_t *out, uint16_t data_in);
void wait(uint16_t count);
uint8_t GAP_RegMask(GAP_Registers_Handle_t reg);
bool GAP_ReadRegs(GAP_Handle_t *pHandle, uint8_t *pDataRead, GAP_Registers_Handle_t reg);
void GAP_StartConfig(GAP_Handle_t *pHandle);
void GAP_StopConfig(GAP_Handle_t *pHandle);
bool GAP_WriteRegs(GAP_Handle_t *pHandle, uint8_t *pDataWrite, GAP_Registers_Handle_t reg);
void GAP_GlobalReset(GAP_Handle_t *pHandle);
bool GAP_ResetStatus(GAP_Handle_t *pHandle, GAP_Registers_Handle_t reg);
bool GAP_Test(GAP_Handle_t *pHandle, GAP_TestMode_t testMode);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __GAP_GATE_DRIVER_CTR_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

