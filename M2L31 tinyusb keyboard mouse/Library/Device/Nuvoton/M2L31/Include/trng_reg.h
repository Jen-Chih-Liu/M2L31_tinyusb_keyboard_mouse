/**************************************************************************//**
 * @file     trng_reg.h
 * @version  V1.00
 * @brief    TRNG register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TRNG_REG_H__
#define __TRNG_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- True Random Number Generator -------------------------*/
/**
    @addtogroup TRNG True Random Number Generator(TRNG)
    Memory Mapped Structure for TRNG Controller
@{ */

typedef struct
{


    /**
     * @var TRNG_T::CTL
     * Offset: 0x00  TRNG Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LDOEN     |LDO Enable Bit
     * |        |          |0 = LDO Disabled.
     * |        |          |1 = LDO Enabled.
     * |[1]     |NRST      |Negative-edge Trigger Reset
     * |        |          |0 = Keep reset.
     * |        |          |1 = No reset.
     * |[2]     |TRNGEN    |True Random Number Generator Macro Enable Bit
     * |        |          |0 = TRNG macro Disabled.
     * |        |          |1 = TRNG macro Enabled.
     * |[3]     |START     |True Random Number Generator Start
     * |        |          |0 = No effect.
     * |        |          |1 = Start TRNG.
     * |        |          |Note: Do not enable START (TRNG_CTL[3]) and KATEN (TRNG_CTL[24]) at the same time
     * |        |          |This bit is always 0 when it is read back.
     * |[5:4]   |MODE      |Random Bit Generator Output Selection
     * |        |          |00 = Output data is from entropy (32-bits).
     * |        |          |01 = Output data is from NRBG (128-bits).
     * |        |          |10 = Output data is from DRBG (128-bits).
     * |        |          |11 = Reserved.
     * |[8]     |INSTANT   |CTR_DRBG User Manual Instant Function
     * |        |          |0 = No effect.
     * |        |          |1 = Enable CTR_DRBG instant function.
     * |[9]     |RESEED    |CTR_DRBG User Manual Reseed Function
     * |        |          |0 = No effect.
     * |        |          |1 = Enable CTR_DRBG reseed function.
     * |[10]    |UPDATE    |CTR_DRBG User Manual Update Function
     * |        |          |0 = No effect.
     * |        |          |1 = Enable CTR_DRBG update function.
     * |[24]    |KATEN     |CTR_DRBG Known Answer Test Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Do not enable START (TRNG_CTL[3]) and KATEN (TRNG_CTL[24]) at the same time.
     * |[26:25] |KATSEL    |CTR_DRBG Known Answer Test Selection
     * |        |          |0 = Instantiation testing.
     * |        |          |1 = Reseed testing.
     * |        |          |2 = Generation testing.
     * |        |          |Others = reserved.
     * |[30]    |ERRIEN    |TRNG Error Interrupt Enable Bit
     * |        |          |0 = Error interrupt Disabled.
     * |        |          |1 = Error interrupt Enabled.
     * |[31]    |DVIEN     |Data Valid Interrupt Enable Bit
     * |        |          |0 = Interrupt Disabled.
     * |        |          |1 = Interrupt Enabled.
     * @var TRNG_T::CFG
     * Offset: 0x04  TRNG Configure Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[25:0]  |RESEED_INTERVAL|Reseed Interval
     * |        |          |Maximum number of requests between reseeds.
     * |        |          |The value is 1 ~ 2^25 .
     * |        |          |Note: If users select out of this range (1 ~ 2^25 ), reseed_interval will be set the maximum value = 2^25.
     * |[31:28] |CTRLEN    |CTR_DRBG Bits Length Per Request
     * |        |          |CTR_DRBG bits length per request = (2^CTRLEN -4) x block length.
     * |        |          |CTR_DRBG block length is 128-bits (2^7).
     * |        |          |CTRLEN is 4~12.
     * |        |          |Note: If users select out of this range (4~12), CTRLEN will be set the maximum value = 12.
     * @var TRNG_T::STS
     * Offset: 0x08  TRNG Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LDORDY    |LDO Ready Signal
     * |        |          |0 = LDO is not ready.
     * |        |          |1 = LDO is ready.
     * |[1]     |TRNGRDY   |TRNG Ready Signal
     * |        |          |0 = True random number generator is not ready.
     * |        |          |1 = True random number generator is ready.
     * |[4]     |ESSUT     |Entropy Source Start-Up Test
     * |        |          |0 = Entropy source is still under testing or test fail.
     * |        |          |1 = Test pass.
     * |[5]     |ESRCT     |Entropy Source Repetition Count Test
     * |        |          |0 = Entropy source is still under testing or test fail.
     * |        |          |1 = Test pass.
     * |[6]     |ESAPT     |Entropy Source Adaptive Proportion Test
     * |        |          |0 = Entropy source is still under testing or test fail.
     * |        |          |1 = Test pass.
     * |[27]    |KATPASS   |CTR_DRBG Known Answer Test Pass
     * |        |          |0 = Test fail.
     * |        |          |1 = Test pass.
     * |        |          |Note: When users enable KATEN (TRNG_CTL[24]), they can check this bit after DVIF (TRNG_STS[31]) become '1'.
     * |[30]    |ERRIF     |TRNG Error Interrupt Flag
     * |        |          |0 = No TRNG error.
     * |        |          |1 = TRNGRDY became '0' over 1ms , TRNG error interrupt.
     * |[31]    |DVIF      |Data Valid Interrupt Flag
     * |        |          |0 = Data is invalid.
     * |        |          |1 = Data is valid. A valid random number can be read form TRNG_DATAx.
     * |        |          |Note: This bit is cleared to '0' by reading TRNG_DATA0 ~ TRNG_DATA3.
     * |        |          |If MODE (TRNG_CTL[5:4]) = '00', this bit is cleared to '0' by reading TRNG_DATA0.
     * @var TRNG_T::DATA[4]
     * Offset: 0x0C ~ 0x18  TRNG Data Output Word 0 ~ Word 3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DATA      |True Random Number Generator Data (Read Only)
     * |        |          |The DATA stores the output data generated by TRNG, it can be read when DVIF (TRNG_STS[31]) become 1 and can be read only once.
     * |        |          |TRNG_DATA0 stores Entropy / NRBG word 0 / DRBG word 0.
     * |        |          |TRNG_DATA1 stores NRBG word 1 / DRBG word 1.
     * |        |          |TRNG_DATA2 stores NRBG word 2 / DRBG word 2.
     * |        |          |TRNG_DATA3 stores NRBG word 3 / DRBG word 3.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] TRNG Control Register                                            */
    __IO uint32_t CFG;                   /*!< [0x0004] TRNG Configure Register                                          */
    __I  uint32_t STS;                   /*!< [0x0008] TRNG Status Register                                             */
    __I  uint32_t DATA[4];               /*!< [0x000c-0x0018] TRNG Data Output Word 0 Register                          */

} TRNG_T;

/**
    @addtogroup TRNG_CONST TRNG Bit Field Definition
    Constant Definitions for TRNG Controller
@{ */

#define TRNG_CTL_LDOEN_Pos               (0)                                               /*!< TRNG_T::CTL: LDOEN Position            */
#define TRNG_CTL_LDOEN_Msk               (0x1ul << TRNG_CTL_LDOEN_Pos)                     /*!< TRNG_T::CTL: LDOEN Mask                */

#define TRNG_CTL_NRST_Pos                (1)                                               /*!< TRNG_T::CTL: NRST Position             */
#define TRNG_CTL_NRST_Msk                (0x1ul << TRNG_CTL_NRST_Pos)                      /*!< TRNG_T::CTL: NRST Mask                 */

#define TRNG_CTL_TRNGEN_Pos              (2)                                               /*!< TRNG_T::CTL: TRNGEN Position           */
#define TRNG_CTL_TRNGEN_Msk              (0x1ul << TRNG_CTL_TRNGEN_Pos)                    /*!< TRNG_T::CTL: TRNGEN Mask               */

#define TRNG_CTL_START_Pos               (3)                                               /*!< TRNG_T::CTL: START Position            */
#define TRNG_CTL_START_Msk               (0x1ul << TRNG_CTL_START_Pos)                     /*!< TRNG_T::CTL: START Mask                */

#define TRNG_CTL_MODE_Pos                (4)                                               /*!< TRNG_T::CTL: MODE Position             */
#define TRNG_CTL_MODE_Msk                (0x3ul << TRNG_CTL_MODE_Pos)                      /*!< TRNG_T::CTL: MODE Mask                 */

#define TRNG_CTL_INSTANT_Pos             (8)                                               /*!< TRNG_T::CTL: INSTANT Position          */
#define TRNG_CTL_INSTANT_Msk             (0x1ul << TRNG_CTL_INSTANT_Pos)                   /*!< TRNG_T::CTL: INSTANT Mask              */

#define TRNG_CTL_RESEED_Pos              (9)                                               /*!< TRNG_T::CTL: RESEED Position           */
#define TRNG_CTL_RESEED_Msk              (0x1ul << TRNG_CTL_RESEED_Pos)                    /*!< TRNG_T::CTL: RESEED Mask               */

#define TRNG_CTL_UPDATE_Pos              (10)                                              /*!< TRNG_T::CTL: UPDATE Position           */
#define TRNG_CTL_UPDATE_Msk              (0x1ul << TRNG_CTL_UPDATE_Pos)                    /*!< TRNG_T::CTL: UPDATE Mask               */

#define TRNG_CTL_KATEN_Pos               (24)                                              /*!< TRNG_T::CTL: KATEN Position            */
#define TRNG_CTL_KATEN_Msk               (0x1ul << TRNG_CTL_KATEN_Pos)                     /*!< TRNG_T::CTL: KATEN Mask                */

#define TRNG_CTL_KATSEL_Pos              (25)                                              /*!< TRNG_T::CTL: KATSEL Position           */
#define TRNG_CTL_KATSEL_Msk              (0x3ul << TRNG_CTL_KATSEL_Pos)                    /*!< TRNG_T::CTL: KATSEL Mask               */

#define TRNG_CTL_ERRIEN_Pos              (30)                                              /*!< TRNG_T::CTL: ERRIEN Position           */
#define TRNG_CTL_ERRIEN_Msk              (0x1ul << TRNG_CTL_ERRIEN_Pos)                    /*!< TRNG_T::CTL: ERRIEN Mask               */

#define TRNG_CTL_DVIEN_Pos               (31)                                              /*!< TRNG_T::CTL: DVIEN Position            */
#define TRNG_CTL_DVIEN_Msk               (0x1ul << TRNG_CTL_DVIEN_Pos)                     /*!< TRNG_T::CTL: DVIEN Mask                */

#define TRNG_CFG_RESEED_INTERVAL_Pos     (0)                                               /*!< TRNG_T::CFG: RESEED_INTERVAL Position  */
#define TRNG_CFG_RESEED_INTERVAL_Msk     (0x3fffffful << TRNG_CFG_RESEED_INTERVAL_Pos)     /*!< TRNG_T::CFG: RESEED_INTERVAL Mask      */

#define TRNG_CFG_CTRLEN_Pos              (28)                                              /*!< TRNG_T::CFG: CTRLEN Position           */
#define TRNG_CFG_CTRLEN_Msk              (0xful << TRNG_CFG_CTRLEN_Pos)                    /*!< TRNG_T::CFG: CTRLEN Mask               */

#define TRNG_STS_LDORDY_Pos              (0)                                               /*!< TRNG_T::STS: LDORDY Position           */
#define TRNG_STS_LDORDY_Msk              (0x1ul << TRNG_STS_LDORDY_Pos)                    /*!< TRNG_T::STS: LDORDY Mask               */

#define TRNG_STS_TRNGRDY_Pos             (1)                                               /*!< TRNG_T::STS: TRNGRDY Position          */
#define TRNG_STS_TRNGRDY_Msk             (0x1ul << TRNG_STS_TRNGRDY_Pos)                   /*!< TRNG_T::STS: TRNGRDY Mask              */

#define TRNG_STS_ESSUT_Pos               (4)                                               /*!< TRNG_T::STS: ESSUT Position            */
#define TRNG_STS_ESSUT_Msk               (0x1ul << TRNG_STS_ESSUT_Pos)                     /*!< TRNG_T::STS: ESSUT Mask                */

#define TRNG_STS_ESRCT_Pos               (5)                                               /*!< TRNG_T::STS: ESRCT Position            */
#define TRNG_STS_ESRCT_Msk               (0x1ul << TRNG_STS_ESRCT_Pos)                     /*!< TRNG_T::STS: ESRCT Mask                */

#define TRNG_STS_ESAPT_Pos               (6)                                               /*!< TRNG_T::STS: ESAPT Position            */
#define TRNG_STS_ESAPT_Msk               (0x1ul << TRNG_STS_ESAPT_Pos)                     /*!< TRNG_T::STS: ESAPT Mask                */

#define TRNG_STS_KATPASS_Pos             (27)                                              /*!< TRNG_T::STS: KATPASS Position          */
#define TRNG_STS_KATPASS_Msk             (0x1ul << TRNG_STS_KATPASS_Pos)                   /*!< TRNG_T::STS: KATPASS Mask              */

#define TRNG_STS_ERRIF_Pos               (30)                                              /*!< TRNG_T::STS: ERRIF Position            */
#define TRNG_STS_ERRIF_Msk               (0x1ul << TRNG_STS_ERRIF_Pos)                     /*!< TRNG_T::STS: ERRIF Mask                */

#define TRNG_STS_DVIF_Pos                (31)                                              /*!< TRNG_T::STS: DVIF Position             */
#define TRNG_STS_DVIF_Msk                (0x1ul << TRNG_STS_DVIF_Pos)                      /*!< TRNG_T::STS: DVIF Mask                 */

#define TRNG_DATA0_DATA_Pos              (0)                                               /*!< TRNG_T::DATA0: DATA Position           */
#define TRNG_DATA0_DATA_Msk              (0xfffffffful << TRNG_DATA0_DATA_Pos)             /*!< TRNG_T::DATA0: DATA Mask               */

#define TRNG_DATA1_DATA_Pos              (0)                                               /*!< TRNG_T::DATA1: DATA Position           */
#define TRNG_DATA1_DATA_Msk              (0xfffffffful << TRNG_DATA1_DATA_Pos)             /*!< TRNG_T::DATA1: DATA Mask               */

#define TRNG_DATA2_DATA_Pos              (0)                                               /*!< TRNG_T::DATA2: DATA Position           */
#define TRNG_DATA2_DATA_Msk              (0xfffffffful << TRNG_DATA2_DATA_Pos)             /*!< TRNG_T::DATA2: DATA Mask               */

#define TRNG_DATA3_DATA_Pos              (0)                                               /*!< TRNG_T::DATA3: DATA Position           */
#define TRNG_DATA3_DATA_Msk              (0xfffffffful << TRNG_DATA3_DATA_Pos)             /*!< TRNG_T::DATA3: DATA Mask               */

/**@}*/ /* TRNG_CONST */
/**@}*/ /* end of TRNG register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __TRNG_REG_H__ */