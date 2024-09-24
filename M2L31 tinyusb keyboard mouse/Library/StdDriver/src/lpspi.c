/**************************************************************************//**
 * @file     lpspi.c
 * @version  V3.00
 * @brief    M2L31 series LPSPI driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPSPI_Driver LPSPI Driver
  @{
*/

/** @addtogroup LPSPI_EXPORTED_FUNCTIONS LPSPI Exported Functions
  @{
*/

/**
  * @brief  This function make LPSPI module be ready to transfer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32MasterSlave Decides the LPSPI module is operating in master mode or in slave mode. (LPSPI_SLAVE, LPSPI_MASTER)
  * @param[in]  u32SPIMode Decides the transfer timing. (LPSPI_MODE_0, LPSPI_MODE_1, LPSPI_MODE_2, LPSPI_MODE_3)
  * @param[in]  u32DataWidth Decides the data width of a LPSPI transaction.
  * @param[in]  u32BusClock The expected frequency of LPSPI bus clock in Hz.
  * @return Actual frequency of LPSPI peripheral clock.
  * @details By default, the LPSPI transfer sequence is MSB first, the slave selection signal is active low and the automatic
  *          slave selection function is disabled.
  *          In Slave mode, the u32BusClock shall be NULL and the LPSPI clock divider setting will be 0.
  *          The actual clock rate may be different from the target LPSPI clock rate.
  *          For example, if the LPSPI source clock rate is 12 MHz and the target LPSPI bus clock rate is 7 MHz, the
  *          actual LPSPI clock rate will be 6MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, LPSPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= LPSPI peripheral clock source, DIVIDER will be set to 0.
  * @note   In slave mode, the LPSPI peripheral clock rate will be equal to APB clock rate.
  */
uint32_t LPSPI_Open(LPSPI_T *lpspi,
                    uint32_t u32MasterSlave,
                    uint32_t u32SPIMode,
                    uint32_t u32DataWidth,
                    uint32_t u32BusClock)
{
    uint32_t u32ClkSrc = 0U, u32Div, u32RetValue=0U;

    if(u32DataWidth == 32U)
    {
        u32DataWidth = 0U;
    }

    if(u32MasterSlave == LPSPI_MASTER)
    {
        /* Default setting: slave selection signal is active low; disable automatic slave selection function. */
        lpspi->SSCTL = LPSPI_SS_ACTIVE_LOW;

        /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
        lpspi->CTL = u32MasterSlave | (u32DataWidth << LPSPI_CTL_DWIDTH_Pos) | (u32SPIMode) | LPSPI_CTL_SPIEN_Msk;

        /* Check clock source of LPSPI */
        if(lpspi == LPSPI0)
        {
            if((LPSCC->CLKSEL0 & (~LPSCC_CLKSEL0_LPSPI0SEL_Msk)) == LPSCC_CLKSEL0_LPSPI0SEL_HIRC)
            {
                u32ClkSrc = __HIRC; /* Clock source is HIRC */
            }
            else
            {
                u32ClkSrc = __MIRC; /* Clock source is MIRC */
            }
        }

        if(u32BusClock >= u32ClkSrc)
        {
            /* Set DIVIDER = 0 */
            lpspi->CLKDIV = 0U;
            /* Return master peripheral clock rate */
            u32RetValue = u32ClkSrc;
        }
        else if(u32BusClock == 0U)
        {
            /* Set DIVIDER to the maximum value 0xFF. f_lpspi = f_spi_clk_src / (DIVIDER + 1) */
            lpspi->CLKDIV |= LPSPI_CLKDIV_DIVIDER_Msk;
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (0xFFU + 1U));
        }
        else
        {
            u32Div = (((u32ClkSrc * 10U) / u32BusClock + 5U) / 10U) - 1U; /* Round to the nearest integer */
            if(u32Div > 0xFFU)
            {
                u32Div = 0xFFU;
                lpspi->CLKDIV |= LPSPI_CLKDIV_DIVIDER_Msk;
                /* Return master peripheral clock rate */
                u32RetValue = (u32ClkSrc / (0xFFU + 1U));
            }
            else
            {
                lpspi->CLKDIV = (lpspi->CLKDIV & (~LPSPI_CLKDIV_DIVIDER_Msk)) | (u32Div << LPSPI_CLKDIV_DIVIDER_Pos);
                /* Return master peripheral clock rate */
                u32RetValue = (u32ClkSrc / (u32Div + 1U));
            }
        }
    }
    else     /* For slave mode, force the LPSPI peripheral clock rate to equal APB clock rate. */
    {
        /* Default setting: slave selection signal is low level active. */
        lpspi->SSCTL = LPSPI_SS_ACTIVE_LOW;

        /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
        lpspi->CTL = u32MasterSlave | (u32DataWidth << LPSPI_CTL_DWIDTH_Pos) | (u32SPIMode) | LPSPI_CTL_SPIEN_Msk;

        /* Set DIVIDER = 0 */
        lpspi->CLKDIV = 0U;

        /* Select PCLK as the clock source of LPSPI */
        if(lpspi == LPSPI0)
        {
            if((LPSCC->CLKSEL0 & (~LPSCC_CLKSEL0_LPSPI0SEL_Msk)) == LPSCC_CLKSEL0_LPSPI0SEL_HIRC)
            {
                u32RetValue = __HIRC; /* Clock source is HIRC */
            }
            else
            {
                u32RetValue = __MIRC; /* Clock source is MIRC */
            }
        }
    }
    return u32RetValue;
}

/**
  * @brief  Disable LPSPI controller.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return None
  * @details This function will reset LPSPI controller.
  */
void LPSPI_Close(LPSPI_T *lpspi)
{
    if(lpspi == LPSPI0)
    {
        /* Reset LPSPI */
        LPSCC->IPRST0 |= LPSCC_IPRST0_LPSPI0RST_Msk;
        LPSCC->IPRST0 &= ~LPSCC_IPRST0_LPSPI0RST_Msk;
    }
}

/**
  * @brief  Clear RX FIFO buffer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return None
  * @details This function will clear LPSPI RX FIFO buffer. The RXEMPTY (LPSPI_STATUS[8]) will be set to 1.
  */
void LPSPI_ClearRxFIFO(LPSPI_T *lpspi)
{
    lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXFBCLR_Msk;
}

/**
  * @brief  Clear TX FIFO buffer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return None
  * @details This function will clear LPSPI TX FIFO buffer. The TXEMPTY (LPSPI_STATUS[16]) will be set to 1.
  * @note The TX shift register will not be cleared.
  */
void LPSPI_ClearTxFIFO(LPSPI_T *lpspi)
{
    lpspi->FIFOCTL |= LPSPI_FIFOCTL_TXFBCLR_Msk;
}

/**
  * @brief  Disable the automatic slave selection function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return None
  * @details This function will disable the automatic slave selection function and set slave selection signal to inactive state.
  */
void LPSPI_DisableAutoSS(LPSPI_T *lpspi)
{
    lpspi->SSCTL &= ~(LPSPI_SSCTL_AUTOSS_Msk | LPSPI_SSCTL_SS_Msk);
}

/**
  * @brief  Enable the automatic slave selection function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32SSPinMask Specifies slave selection pins. (LPSPI_SS)
  * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (LPSPI_SS_ACTIVE_HIGH, LPSPI_SS_ACTIVE_LOW)
  * @return None
  * @details This function will enable the automatic slave selection function. Only available in Master mode.
  *          The slave selection pin and the active level will be set in this function.
  */
void LPSPI_EnableAutoSS(LPSPI_T *lpspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    lpspi->SSCTL = (lpspi->SSCTL & (~(LPSPI_SSCTL_AUTOSS_Msk | LPSPI_SSCTL_SSACTPOL_Msk | LPSPI_SSCTL_SS_Msk))) | (u32SSPinMask | u32ActiveLevel | LPSPI_SSCTL_AUTOSS_Msk);
}

/**
  * @brief  Set the LPSPI bus clock.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32BusClock The expected frequency of LPSPI bus clock in Hz.
  * @return Actual frequency of LPSPI bus clock.
  * @details This function is only available in Master mode. The actual clock rate may be different from the target LPSPI bus clock rate.
  *          For example, if the LPSPI source clock rate is 12 MHz and the target LPSPI bus clock rate is 7 MHz, the actual LPSPI bus clock
  *          rate will be 6 MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, LPSPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= LPSPI peripheral clock source, DIVIDER will be set to 0.
  */
uint32_t LPSPI_SetBusClock(LPSPI_T *lpspi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc;
    uint32_t u32Div, u32RetValue;

    /* Check clock source of LPSPI */
    if(lpspi == LPSPI0)
    {
        if((LPSCC->CLKSEL0 & (~LPSCC_CLKSEL0_LPSPI0SEL_Msk)) == LPSCC_CLKSEL0_LPSPI0SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
    }
    else
    {
        u32RetValue = 0;
        return u32RetValue;
    }

    if(u32BusClock >= u32ClkSrc)
    {
        /* Set DIVIDER = 0 */
        lpspi->CLKDIV = 0U;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if(u32BusClock == 0U)
    {
        /* Set DIVIDER to the maximum value 0xFF. f_lpspi = f_spi_clk_src / (DIVIDER + 1) */
        lpspi->CLKDIV |= LPSPI_CLKDIV_DIVIDER_Msk;
        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / (0xFFU + 1U));
    }
    else
    {
        u32Div = (((u32ClkSrc * 10U) / u32BusClock + 5U) / 10U) - 1U; /* Round to the nearest integer */
        if(u32Div > 0x1FFU)
        {
            u32Div = 0x1FFU;
            lpspi->CLKDIV |= LPSPI_CLKDIV_DIVIDER_Msk;
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (0xFFU + 1U));
        }
        else
        {
            lpspi->CLKDIV = (lpspi->CLKDIV & (~LPSPI_CLKDIV_DIVIDER_Msk)) | (u32Div << LPSPI_CLKDIV_DIVIDER_Pos);
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (u32Div + 1U));
        }
    }

    return u32RetValue;
}

/**
  * @brief  Configure FIFO threshold setting.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
  * @param[in]  u32RxThreshold Decides the RX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
  * @return None
  * @details Set TX FIFO threshold and RX FIFO threshold configurations.
  */
void LPSPI_SetFIFO(LPSPI_T *lpspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    lpspi->FIFOCTL = (lpspi->FIFOCTL & ~(LPSPI_FIFOCTL_TXTH_Msk | LPSPI_FIFOCTL_RXTH_Msk)) |
                     (u32TxThreshold << LPSPI_FIFOCTL_TXTH_Pos) |
                     (u32RxThreshold << LPSPI_FIFOCTL_RXTH_Pos);
}

/**
  * @brief  Get the actual frequency of LPSPI bus clock. Only available in Master mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return Actual LPSPI bus clock frequency in Hz.
  * @details This function will calculate the actual LPSPI bus clock rate according to the SPInSEL and DIVIDER settings. Only available in Master mode.
  */
uint32_t LPSPI_GetBusClock(LPSPI_T *lpspi)
{
    uint32_t u32Div;
    uint32_t u32ClkSrc;

    /* Get DIVIDER setting */
    u32Div = (lpspi->CLKDIV & LPSPI_CLKDIV_DIVIDER_Msk) >> LPSPI_CLKDIV_DIVIDER_Pos;

    /* Check clock source of LPSPI */
    if(lpspi == LPSPI0)
    {
        if((LPSCC->CLKSEL0 & (~LPSCC_CLKSEL0_LPSPI0SEL_Msk)) == LPSCC_CLKSEL0_LPSPI0SEL_HIRC)
        {
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
        }
        else
        {
            u32ClkSrc = __MIRC; /* Clock source is MIRC */
        }
    }
    /* Return LPSPI bus clock rate */
    return (u32ClkSrc / (u32Div + 1U));
}

/**
  * @brief  Enable interrupt function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled. It is combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_TXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Enable LPSPI related interrupts specified by u32Mask parameter.
  */
void LPSPI_EnableInt(LPSPI_T *lpspi, uint32_t u32Mask)
{
    /* Enable unit transfer interrupt flag */
    if((u32Mask & LPSPI_UNIT_INT_MASK) == LPSPI_UNIT_INT_MASK)
    {
        lpspi->CTL |= LPSPI_CTL_UNITIEN_Msk;
    }

    /* Enable slave selection signal active interrupt flag */
    if((u32Mask & LPSPI_SSACT_INT_MASK) == LPSPI_SSACT_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SSACTIEN_Msk;
    }

    /* Enable slave selection signal inactive interrupt flag */
    if((u32Mask & LPSPI_SSINACT_INT_MASK) == LPSPI_SSINACT_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SSINAIEN_Msk;
    }

    /* Enable slave TX under run interrupt flag */
    if((u32Mask & LPSPI_SLVUR_INT_MASK) == LPSPI_SLVUR_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SLVURIEN_Msk;
    }

    /* Enable slave bit count error interrupt flag */
    if((u32Mask & LPSPI_SLVBE_INT_MASK) == LPSPI_SLVBE_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Enable slave TX underflow interrupt flag */
    if((u32Mask & LPSPI_TXUF_INT_MASK) == LPSPI_TXUF_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Enable TX threshold interrupt flag */
    if((u32Mask & LPSPI_FIFO_TXTH_INT_MASK) == LPSPI_FIFO_TXTH_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Enable RX threshold interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXTH_INT_MASK) == LPSPI_FIFO_RXTH_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXOV_INT_MASK) == LPSPI_FIFO_RXOV_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Enable RX time-out interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXTO_INT_MASK) == LPSPI_FIFO_RXTO_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
  * @brief  Disable interrupt function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupts will be disabled. It is combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_TXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Disable LPSPI related interrupts specified by u32Mask parameter.
  */
void LPSPI_DisableInt(LPSPI_T *lpspi, uint32_t u32Mask)
{
    /* Disable unit transfer interrupt flag */
    if((u32Mask & LPSPI_UNIT_INT_MASK) == LPSPI_UNIT_INT_MASK)
    {
        lpspi->CTL &= ~LPSPI_CTL_UNITIEN_Msk;
    }

    /* Disable slave selection signal active interrupt flag */
    if((u32Mask & LPSPI_SSACT_INT_MASK) == LPSPI_SSACT_INT_MASK)
    {
        lpspi->SSCTL &= ~LPSPI_SSCTL_SSACTIEN_Msk;
    }

    /* Disable slave selection signal inactive interrupt flag */
    if((u32Mask & LPSPI_SSINACT_INT_MASK) == LPSPI_SSINACT_INT_MASK)
    {
        lpspi->SSCTL &= ~LPSPI_SSCTL_SSINAIEN_Msk;
    }

    /* Disable slave TX under run interrupt flag */
    if((u32Mask & LPSPI_SLVUR_INT_MASK) == LPSPI_SLVUR_INT_MASK)
    {
        lpspi->SSCTL &= ~LPSPI_SSCTL_SLVURIEN_Msk;
    }

    /* Disable slave bit count error interrupt flag */
    if((u32Mask & LPSPI_SLVBE_INT_MASK) == LPSPI_SLVBE_INT_MASK)
    {
        lpspi->SSCTL &= ~LPSPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Disable slave TX underflow interrupt flag */
    if((u32Mask & LPSPI_TXUF_INT_MASK) == LPSPI_TXUF_INT_MASK)
    {
        lpspi->FIFOCTL &= ~LPSPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Disable TX threshold interrupt flag */
    if((u32Mask & LPSPI_FIFO_TXTH_INT_MASK) == LPSPI_FIFO_TXTH_INT_MASK)
    {
        lpspi->FIFOCTL &= ~LPSPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Disable RX threshold interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXTH_INT_MASK) == LPSPI_FIFO_RXTH_INT_MASK)
    {
        lpspi->FIFOCTL &= ~LPSPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Disable RX overrun interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXOV_INT_MASK) == LPSPI_FIFO_RXOV_INT_MASK)
    {
        lpspi->FIFOCTL &= ~LPSPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Disable RX time-out interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXTO_INT_MASK) == LPSPI_FIFO_RXTO_INT_MASK)
    {
        lpspi->FIFOCTL &= ~LPSPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
  * @brief  Get interrupt flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_TXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @return Interrupt flags of selected sources.
  * @details Get LPSPI related interrupt flags specified by u32Mask parameter.
  */
uint32_t LPSPI_GetIntFlag(LPSPI_T *lpspi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0U, u32TmpVal;

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_UNITIF_Msk;
    /* Check unit transfer interrupt flag */
    if((u32Mask & LPSPI_UNIT_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_UNIT_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_SSACTIF_Msk;
    /* Check slave selection signal active interrupt flag */
    if((u32Mask & LPSPI_SSACT_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_SSACT_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_SSINAIF_Msk;
    /* Check slave selection signal inactive interrupt flag */
    if((u32Mask & LPSPI_SSINACT_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_SSINACT_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_SLVURIF_Msk;
    /* Check slave TX under run interrupt flag */
    if((u32Mask & LPSPI_SLVUR_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_SLVUR_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_SLVBEIF_Msk;
    /* Check slave bit count error interrupt flag */
    if((u32Mask & LPSPI_SLVBE_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_SLVBE_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_TXUFIF_Msk;
    /* Check slave TX underflow interrupt flag */
    if((u32Mask & LPSPI_TXUF_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_TXUF_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_TXTHIF_Msk;
    /* Check TX threshold interrupt flag */
    if((u32Mask & LPSPI_FIFO_TXTH_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_FIFO_TXTH_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_RXTHIF_Msk;
    /* Check RX threshold interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXTH_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_FIFO_RXTH_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_RXOVIF_Msk;
    /* Check RX overrun interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXOV_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_FIFO_RXOV_INT_MASK;
    }

    u32TmpVal = lpspi->STATUS & LPSPI_STATUS_RXTOIF_Msk;
    /* Check RX time-out interrupt flag */
    if((u32Mask & LPSPI_FIFO_RXTO_INT_MASK) && (u32TmpVal))
    {
        u32IntFlag |= LPSPI_FIFO_RXTO_INT_MASK;
    }

    return u32IntFlag;
}

/**
  * @brief  Clear interrupt flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Clear LPSPI related interrupt flags specified by u32Mask parameter.
  */
void LPSPI_ClearIntFlag(LPSPI_T *lpspi, uint32_t u32Mask)
{
    if(u32Mask & LPSPI_UNIT_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_UNITIF_Msk; /* Clear unit transfer interrupt flag */
    }

    if(u32Mask & LPSPI_SSACT_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SSACTIF_Msk; /* Clear slave selection signal active interrupt flag */
    }

    if(u32Mask & LPSPI_SSINACT_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SSINAIF_Msk; /* Clear slave selection signal inactive interrupt flag */
    }

    if(u32Mask & LPSPI_SLVUR_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SLVURIF_Msk; /* Clear slave TX under run interrupt flag */
    }

    if(u32Mask & LPSPI_SLVBE_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SLVBEIF_Msk; /* Clear slave bit count error interrupt flag */
    }

    if(u32Mask & LPSPI_TXUF_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_TXUFIF_Msk; /* Clear slave TX underflow interrupt flag */
    }

    if(u32Mask & LPSPI_FIFO_RXOV_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_RXOVIF_Msk; /* Clear RX overrun interrupt flag */
    }

    if(u32Mask & LPSPI_FIFO_RXTO_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_RXTOIF_Msk; /* Clear RX time-out interrupt flag */
    }
}

/**
  * @brief  Get LPSPI status.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a source.
  *                     This parameter decides which flags will be read. It is combination of:
  *                       - \ref LPSPI_BUSY_MASK
  *                       - \ref LPSPI_RX_EMPTY_MASK
  *                       - \ref LPSPI_RX_FULL_MASK
  *                       - \ref LPSPI_TX_EMPTY_MASK
  *                       - \ref LPSPI_TX_FULL_MASK
  *                       - \ref LPSPI_TXRX_RESET_MASK
  *                       - \ref LPSPI_SPIEN_STS_MASK
  *                       - \ref LPSPI_SSLINE_STS_MASK
  *
  * @return Flags of selected sources.
  * @details Get LPSPI related status specified by u32Mask parameter.
  */
uint32_t LPSPI_GetStatus(LPSPI_T *lpspi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0U, u32TmpValue;

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_BUSY_Msk;
    /* Check busy status */
    if((u32Mask & LPSPI_BUSY_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_BUSY_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_RXEMPTY_Msk;
    /* Check RX empty flag */
    if((u32Mask & LPSPI_RX_EMPTY_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_RX_EMPTY_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_RXFULL_Msk;
    /* Check RX full flag */
    if((u32Mask & LPSPI_RX_FULL_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_RX_FULL_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_TXEMPTY_Msk;
    /* Check TX empty flag */
    if((u32Mask & LPSPI_TX_EMPTY_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_TX_EMPTY_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_TXFULL_Msk;
    /* Check TX full flag */
    if((u32Mask & LPSPI_TX_FULL_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_TX_FULL_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_TXRXRST_Msk;
    /* Check TX/RX reset flag */
    if((u32Mask & LPSPI_TXRX_RESET_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_TXRX_RESET_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_SPIENSTS_Msk;
    /* Check SPIEN flag */
    if((u32Mask & LPSPI_SPIEN_STS_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_SPIEN_STS_MASK;
    }

    u32TmpValue = lpspi->STATUS & LPSPI_STATUS_SSLINE_Msk;
    /* Check SPIx_SS line status */
    if((u32Mask & LPSPI_SSLINE_STS_MASK) && (u32TmpValue))
    {
        u32Flag |= LPSPI_SSLINE_STS_MASK;
    }

    return u32Flag;
}

/*@}*/ /* end of group LPSPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group LPSPI_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
