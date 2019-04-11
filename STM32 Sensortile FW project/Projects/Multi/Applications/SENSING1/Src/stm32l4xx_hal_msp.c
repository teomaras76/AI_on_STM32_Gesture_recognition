/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   HAL MSP module.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by STM32CubeMX and eventually modified 
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SENSING1_config.h"
    
/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

#ifdef STM32_NUCLEO
#include "stm32l4xx_periph_conf.h"
/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USART2_CMN_DEFAULT_TX_GPIO_CLK_ENABLE();
  USART2_CMN_DEFAULT_RX_GPIO_CLK_ENABLE();

  /* Enable USARTx clock */
  USART2_CMN_DEFAULT_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USART2_CMN_DEFAULT_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USART2_CMN_DEFAULT_TX_AF;

  HAL_GPIO_Init(USART2_CMN_DEFAULT_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USART2_CMN_DEFAULT_RX_PIN;
  GPIO_InitStruct.Alternate = USART2_CMN_DEFAULT_RX_AF;

  HAL_GPIO_Init(USART2_CMN_DEFAULT_RX_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  USART2_CMN_DEFAULT_FORCE_RESET();
  USART2_CMN_DEFAULT_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USART2_CMN_DEFAULT_TX_GPIO_PORT, USART2_CMN_DEFAULT_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USART2_CMN_DEFAULT_RX_GPIO_PORT, USART2_CMN_DEFAULT_RX_PIN);
}


/**
 * @brief  I2C MSP Initialization
 * @param hi2c: I2C handle pointer
 * @retval None
 */

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable I2C GPIO clocks */
  I2C1_CMN_DEFAULT_SCL_SDA_GPIO_CLK_ENABLE();
  
  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin = I2C1_CMN_DEFAULT_SCL_PIN |I2C1_CMN_DEFAULT_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  =I2C1_CMN_DEFAULT_SCL_SDA_AF;
  HAL_GPIO_Init(I2C1_CMN_DEFAULT_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
  
  /* Enable the I2C_EXPBD peripheral clock */
 I2C1_CMN_DEFAULT_CLK_ENABLE();
  
  /* Force the I2C peripheral clock reset */
 I2C1_CMN_DEFAULT_FORCE_RESET();
  
  /* Release the I2C peripheral clock reset */
 I2C1_CMN_DEFAULT_RELEASE_RESET();
  
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(I2C1_CMN_DEFAULT_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_CMN_DEFAULT_EV_IRQn);
  
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(I2C1_CMN_DEFAULT_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_CMN_DEFAULT_ER_IRQn);
}

/**
 * @brief  This function is used for low level initialization of the SPI 
 *         communication with the BlueNRG Expansion Board.
 * @param  hspi: SPI handle.
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Enable peripherals clock */

  /* Enable GPIO Ports Clock */  
  SPI1_CMN_DEFAULT_RESET_CLK_ENABLE();
  SPI1_CMN_DEFAULT_SCLK_CLK_ENABLE();
  SPI1_CMN_DEFAULT_MISO_CLK_ENABLE();
  SPI1_CMN_DEFAULT_MOSI_CLK_ENABLE();
  SPI1_CMN_DEFAULT_CS_CLK_ENABLE();
  SPI1_CMN_DEFAULT_IRQ_CLK_ENABLE();

  /* Enable SPI clock */
  SPI1_CMN_DEFAULT_CLK_ENABLE();

  /* Reset */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_RESET_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_RESET_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_RESET_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_RESET_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_RESET_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_RESET_PORT, &GPIO_InitStruct);	
  /*Added to avoid spurious interrupt from the BlueNRG */
  HAL_GPIO_WritePin(SPI1_CMN_DEFAULT_RESET_PORT, SPI1_CMN_DEFAULT_RESET_PIN, GPIO_PIN_RESET);

  /* SCLK */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_SCLK_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_SCLK_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_SCLK_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_SCLK_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_SCLK_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_SCLK_PORT, &GPIO_InitStruct); 

  /* MISO */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_MISO_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_MISO_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_MISO_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_MISO_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_MISO_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_MISO_PORT, &GPIO_InitStruct);

  /* MOSI */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_MOSI_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_MOSI_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_MOSI_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_MOSI_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_MOSI_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_MOSI_PORT, &GPIO_InitStruct);

  /* NSS/CSN/CS */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_CS_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_CS_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_CS_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_CS_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_CS_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SPI1_CMN_DEFAULT_CS_PORT, SPI1_CMN_DEFAULT_CS_PIN, GPIO_PIN_SET);

  /* IRQ -- INPUT */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_IRQ_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_IRQ_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_IRQ_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_IRQ_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_IRQ_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_IRQ_PORT, &GPIO_InitStruct);

  /* Configure the NVIC for SPI */  
  HAL_NVIC_SetPriority(SPI1_CMN_DEFAULT_EXTI_IRQn, 5, 0);    
  HAL_NVIC_EnableIRQ(SPI1_CMN_DEFAULT_EXTI_IRQn);
}
#endif /* STM32_NUCLEO */

#if (defined(STM32_SENSORTILE) || defined(NN_ASC))
/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Configure USB FS GPIOs */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure DM DP Pins */
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable USB FS Clock */
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  /* Set USB FS Interrupt priority */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);

  /* Enable USB FS Interrupt */
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
  * @brief  De-Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  /* Disable USB FS Clock */
  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  __HAL_RCC_SYSCFG_CLK_DISABLE();
}
#endif /* (defined(STM32_SENSORTILE) || defined(NN_ASC)) */


/**
  * @brief RTC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hrtc: RTC handle pointer
  * @note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select 
  *        the RTC clock source; in this case the Backup domain will be reset in  
  *        order to modify the RTC Clock source, as consequence RTC registers (including 
  *        the backup registers) and RCC_CSR register are set to their reset values.  
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_DisableBkUpAccess();


  /*##-1- Configure the RTC clock source ######################################*/
  /* -a- Enable LSI Oscillator */
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }

  /* -b- Select LSI as RTC clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  { 
    while(1);
  }
  HAL_PWR_EnableBkUpAccess();
  
  /*##-2- Enable the RTC peripheral Clock ####################################*/
  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();
  
  /*##-3- Configure the NVIC for RTC Wake up ###################################*/
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x00, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  
  /*##-4- Configure the NVIC for RTC Alarm ###################################*/
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);}

/**
  * @brief RTC MSP De-Initialization 
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_RTC_DISABLE();
}

/**
  * @brief TIM MSP Initialization
  * This function configures the hardware resources used in this example:
  *  - Peripheral's clock enable
  *  - Peripheral's Interrupt Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4) {
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0xF, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  } else if (htim->Instance == TIM5) {
     /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0xF, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  else if (htim->Instance == TIM2) {
     /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0xF, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
}  

/**
  * @brief TIM OC MSP Initialization
  * This function configures the hardware resources used in this example:
  *  - Peripheral's clock enable
  *  - Peripheral's Interrupt Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{ 
  /* TIM1 Peripheral clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Enable TIM1 global Interrupt & set priority  */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0x8, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/**
  * @brief CRC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
  /* CRC Peripheral clock enable */
  __HAL_RCC_CRC_CLK_ENABLE();
}

/**
  * @brief CRC MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
  /* Enable CRC reset state */
  __HAL_RCC_CRC_FORCE_RESET();

  /* Release CRC from reset state */
  __HAL_RCC_CRC_RELEASE_RESET();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
