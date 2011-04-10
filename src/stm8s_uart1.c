/**
  ********************************************************************************
  * @file stm8s_uart1.c
  * @brief This file contains all the functions for the UART1 peripheral.
  * @author STMicroelectronics - MCD Application Team
  * @version V1.1.1
  * @date 06/05/2009
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s_uart1.h"
#include "stm8s_clk.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/** @}
  * @addtogroup UART1_Public_Functions
  * @{
  */

/**
  * @brief Deinitializes the UART1 peripheral.
  * @par Full description:
  * Set the UART1 peripheral registers to their default reset values.
  * @retval None
	*/
void UART1_DeInit(void)
{
    u8 dummy = 0;

    /*< Clear the Idle Line Detected bit in the status rerister by a read
       to the UART1_SR register followed by a Read to the UART1_DR register */
    dummy = UART1->SR;
    dummy = UART1->DR;

    UART1->BRR2 = UART1_BRR2_RESET_VALUE;  /*< Set UART1_BRR2 to reset value 0x00 */
    UART1->BRR1 = UART1_BRR1_RESET_VALUE;  /*< Set UART1_BRR1 to reset value 0x00 */

    UART1->CR1 = UART1_CR1_RESET_VALUE; /*< Set UART1_CR1 to reset value 0x00  */
    UART1->CR2 = UART1_CR2_RESET_VALUE; /*< Set UART1_CR2 to reset value 0x00  */
    UART1->CR3 = UART1_CR3_RESET_VALUE;  /*< Set UART1_CR3 to reset value 0x00  */
    UART1->CR4 = UART1_CR4_RESET_VALUE;  /*< Set UART1_CR4 to reset value 0x00  */
    UART1->CR5 = UART1_CR5_RESET_VALUE; /*< Set UART1_CR5 to reset value 0x00  */

    UART1->GTR = UART1_GTR_RESET_VALUE;
    UART1->PSCR = UART1_PSCR_RESET_VALUE;
}

/**
  * @brief Initializes the UART1 according to the specified parameters.
  * @param[in] BaudRate: The baudrate.
  * @param[in] WordLength : This parameter can be any of the @ref UART1_WordLength_TypeDef enumeration.
  * @param[in] StopBits: This parameter can be any of the @ref UART1_StopBits_TypeDef enumeration.
  * @param[in] Parity: This parameter can be any of the @ref UART1_Parity_TypeDef enumeration.
  * @param[in] SyncMode: This parameter can be any of the @ref UART1_SyncMode_TypeDef values.
  * @param[in] Mode: This parameter can be any of the @ref UART1_Mode_TypeDef values
  * @retval
  * None
  */
void UART1_Init(u32 BaudRate, UART1_WordLength_TypeDef WordLength, UART1_StopBits_TypeDef StopBits, UART1_Parity_TypeDef Parity, UART1_SyncMode_TypeDef SyncMode, UART1_Mode_TypeDef Mode)
{
    u32 BaudRate_Mantissa, BaudRate_Mantissa100 = 0;

    /* assert_param: BaudRate value should be <= 625000 bps */
    assert_param(IS_UART1_BAUDRATE_OK(BaudRate));

    assert_param(IS_UART1_WORDLENGTH_OK(WordLength));

    assert_param(IS_UART1_STOPBITS_OK(StopBits));

    assert_param(IS_UART1_PARITY_OK(Parity));

    /* assert_param: UART1_Mode value should exclude values such as  UART1_ModeTx_Enable|UART1_ModeTx_Disable */
    assert_param(IS_UART1_MODE_OK((u8)Mode));

    /* assert_param: UART1_SyncMode value should exclude values such as
       UART1_CLOCK_ENABLE|UART1_CLOCK_DISABLE */
    assert_param(IS_UART1_SYNCMODE_OK((u8)SyncMode));

    UART1->CR1 &= (u8)(~UART1_CR1_M);  /**< Clear the word length bit */
    UART1->CR1 |= (u8)WordLength; /**< Set the word length bit according to UART1_WordLength value */

    UART1->CR3 &= (u8)(~UART1_CR3_STOP);  /**< Clear the STOP bits */
    UART1->CR3 |= (u8)StopBits;  /**< Set the STOP bits number according to UART1_StopBits value  */

    UART1->CR1 &= (u8)(~(UART1_CR1_PCEN | UART1_CR1_PS  ));  /**< Clear the Parity Control bit */
    UART1->CR1 |= (u8)Parity;  /**< Set the Parity Control bit to UART1_Parity value */

    UART1->BRR1 &= (u8)(~UART1_BRR1_DIVM);  /**< Clear the LSB mantissa of UART1DIV  */
    UART1->BRR2 &= (u8)(~UART1_BRR2_DIVM);  /**< Clear the MSB mantissa of UART1DIV  */
    UART1->BRR2 &= (u8)(~UART1_BRR2_DIVF);  /**< Clear the Fraction bits of UART1DIV */

    /**< Set the UART1 BaudRates in BRR1 and BRR2 registers according to UART1_BaudRate value */
    BaudRate_Mantissa    = ((u32)CLK_GetClockFreq() / (BaudRate << 4));
    BaudRate_Mantissa100 = (((u32)CLK_GetClockFreq() * 100) / (BaudRate << 4));
    UART1->BRR2 |= (u8)((u8)(((BaudRate_Mantissa100 - (BaudRate_Mantissa * 100)) << 4) / 100) & (u8)0x0F); /**< Set the fraction of UART1DIV  */
    UART1->BRR2 |= (u8)((BaudRate_Mantissa >> 4) & (u8)0xF0); /**< Set the MSB mantissa of UART1DIV  */
    UART1->BRR1 |= (u8)BaudRate_Mantissa;           /**< Set the LSB mantissa of UART1DIV  */

    UART1->CR2 &= (u8)~(UART1_CR2_TEN | UART1_CR2_REN); /**< Disable the Transmitter and Receiver before seting the LBCL, CPOL and CPHA bits */
    UART1->CR3 &= (u8)~(UART1_CR3_CPOL | UART1_CR3_CPHA | UART1_CR3_LBCL); /**< Clear the Clock Polarity, lock Phase, Last Bit Clock pulse */
    UART1->CR3 |= (u8)((u8)SyncMode & (u8)(UART1_CR3_CPOL | UART1_CR3_CPHA | UART1_CR3_LBCL));  /**< Set the Clock Polarity, lock Phase, Last Bit Clock pulse */

    if ((u8)Mode & (u8)UART1_MODE_TX_ENABLE)
    {
        UART1->CR2 |= (u8)UART1_CR2_TEN;  /**< Set the Transmitter Enable bit */
    }
    else
    {
        UART1->CR2 &= (u8)(~UART1_CR2_TEN);  /**< Clear the Transmitter Disable bit */
    }
    if ((u8)Mode & (u8)UART1_MODE_RX_ENABLE)
    {
        UART1->CR2 |= (u8)UART1_CR2_REN;  /**< Set the Receiver Enable bit */
    }
    else
    {
        UART1->CR2 &= (u8)(~UART1_CR2_REN);  /**< Clear the Receiver Disable bit */
    }
    /**< Set the Clock Enable bit, lock Polarity, lock Phase and Last Bit Clock pulse bits according to UART1_Mode value */
    if ((u8)SyncMode&(u8)UART1_SYNCMODE_CLOCK_DISABLE)
    {
        UART1->CR3 &= (u8)(~UART1_CR3_CKEN); /**< Clear the Clock Enable bit */
        /**< configure in Push Pull or Open Drain mode the Tx I/O line by setting the correct I/O Port register according the product package and line configuration*/
    }
    else
    {
        UART1->CR3 |= (u8)((u8)SyncMode & UART1_CR3_CKEN);
    }
}

/**
  * @brief Returns the most recent received data by the UART1 peripheral.
  * @par Full description:
  * Returns the most recent received data by the UART1 peripheral.
  * @retval u8 Received Data
  * @par Required preconditions:
  * UART1_Cmd(ENABLE);
  */
u8 UART1_ReceiveData8(void)
{
    return ((u8)UART1->DR);
}


/**
  * @brief Returns the most recent received data by the UART1 peripheral.
  * @par Full description:
  * Returns the most recent received data by the UART1 peripheral.
  * @retval u16 Received Data
  * @par Required preconditions:
  * UART1_Cmd(ENABLE);
  */
u16 UART1_ReceiveData9(void)
{
    return (u16)( (((u16) UART1->DR) | ((u16)(((u16)( (u16)UART1->CR1 & (u16)UART1_CR1_R8)) << 1))) & ((u16)0x01FF));
}

/**
  * @brief Transmits 8 bit data through the UART1 peripheral.
  * @par Full description:
  * Transmits 8 bit data through the UART1 peripheral.
  * @param[in] Data: the data to transmit.
  * @retval
  * None
  * @par Required preconditions:
  * UART1_Cmd(ENABLE);
  */
void UART1_SendData8(u8 Data)
{
    /* Transmit Data */
    UART1->DR = Data;
}

/**
  * @brief Transmits 9 bit data through the UART1 peripheral.
  * @par Full description:
  * Transmits 9 bit data through the UART1 peripheral.
  * @param[in] Data: the data to transmit.
  * @retval
  * None
  * @par Required preconditions:
  * UART1_Cmd(ENABLE);
  */
void UART1_SendData9(u16 Data)
{
    /**< Clear the transmit data bit 8 [8]  */
    UART1->CR1 &= ((u8)~UART1_CR1_T8);
    /**< Write the transmit data bit [8]  */
    UART1->CR1 |= (u8)(((u8)(Data >> 2)) & UART1_CR1_T8);
    /**< Write the transmit data bit [0:7] */
    UART1->DR   = (u8)(Data);
}

/**
  * @brief Transmits break characters.
  * @par Full description:
  * Transmits break characters on the UART1 peripheral.
  * @retval
  * None
  */
void UART1_SendBreak(void)
{
    UART1->CR2 |= UART1_CR2_SBK;
}


/**
  * @brief Sets the address of the UART1 node.
  * @par Full description:
  * Sets the address of the UART1 node.
  * @param[in] UART1_Address: Indicates the address of the UART1 node.
  * @retval
  * None
  */
void UART1_SetAddress(u8 UART1_Address)
{
    /*assert_param for UART1_Address*/
    assert_param(IS_UART1_ADDRESS_OK(UART1_Address));

    /* Clear the UART1 address */
    UART1->CR4 &= ((u8)~UART1_CR4_ADD);
    /* Set the UART1 address node */
    UART1->CR4 |= UART1_Address;
}

/**
  * @brief Sets the specified UART1 guard time.
  * @par Full description:
  * Sets the address of the UART1 node.
  * @par This function is related to SmartCard mode.
  * @param[in] UART1_GuardTime: specifies the guard time.
  * @retval
  * None
  * @par Required preconditions:
  * SmartCard Mode Enabled
  */
void UART1_SetGuardTime(u8 UART1_GuardTime)
{
    /* Set the UART1 guard time */
    UART1->GTR = UART1_GuardTime;
}


/**
  * @brief Checks whether the specified UART1 flag is set or not.
  * @par Full description:
  * Checks whether the specified UART1 flag is set or not.
  * @param[in] UART1_FLAG specifies the flag to check.
  * This parameter can be any of the @ref UART1_Flag_TypeDef enumeration.
  * @retval FlagStatus (SET or RESET)
  */
FlagStatus UART1_GetFlagStatus(UART1_Flag_TypeDef UART1_FLAG)
{
    FlagStatus status = RESET;

    /* Check parameters */
    assert_param(IS_UART1_FLAG_OK(UART1_FLAG));


    /* Check the status of the specified UART1 flag*/
    if (UART1_FLAG == UART1_FLAG_LBDF)
    {
        if ((UART1->CR4 & (u8)UART1_FLAG) != (u8)0x00)
        {
            /* UART1_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART1_FLAG is reset*/
            status = RESET;
        }
    }
    else if (UART1_FLAG == UART1_FLAG_SBK)
    {
        if ((UART1->CR2 & (u8)UART1_FLAG) != (u8)0x00)
        {
            /* UART1_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART1_FLAG is reset*/
            status = RESET;
        }
    }
    else
    {
        if ((UART1->SR & (u8)UART1_FLAG) != (u8)0x00)
        {
            /* UART1_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART1_FLAG is reset*/
            status = RESET;
        }
    }
    /* Return the UART1_FLAG status*/
    return status;
}
/**
  * @brief Clears the UART1 flags.
  * @par Full description:
  * Clears the UART1 flags.
  * @param[in] UART1_FLAG specifies the flag to clear
  * This parameter can be any combination of the following values:
  *   - UART1_FLAG_LBDF: LIN Break detection flag.
  *   - UART1_FLAG_RXNE: Receive data register not empty flag.
  * @par Notes:
  *   - PE (Parity error), FE (Framing error), NE (Noise error), OR (OverRun error)
  *     and IDLE (Idle line detected) flags are cleared by software sequence: a read
  *    operation to UART1_SR register (UART1_GetFlagStatus())followed by a read operation
  *     to UART1_DR register(UART1_ReceiveData8() or UART1_ReceiveData9()).
  *   - RXNE flag can be also cleared by a read to the UART1_DR register
  *     (UART1_ReceiveData8()or UART1_ReceiveData9()).
  *   - TC flag can be also cleared by software sequence: a read operation to UART1_SR
  *     register (UART1_GetFlagStatus()) followed by a write operation to UART1_DR register
  *     (UART1_SendData8() or UART1_SendData9()).
  *   - TXE flag is cleared only by a write to the UART1_DR register (UART1_SendData8() or
  *     UART1_SendData9()).
  *   - SBK flag is cleared during the stop bit of break.
  * @retval
  * None
  */

void UART1_ClearFlag(UART1_Flag_TypeDef UART1_FLAG)
{
    assert_param(IS_UART1_CLEAR_FLAG_OK(UART1_FLAG));

    /* Clear the Receive Register Not Empty flag */
    if (UART1_FLAG == UART1_FLAG_RXNE)
    {
        UART1->SR = (u8)~(UART1_SR_RXNE);
    }
    /* Clear the LIN Break Detection flag */
    else
    {
        UART1->CR4 &= (u8)~(UART1_CR4_LBDF);
    }
}

