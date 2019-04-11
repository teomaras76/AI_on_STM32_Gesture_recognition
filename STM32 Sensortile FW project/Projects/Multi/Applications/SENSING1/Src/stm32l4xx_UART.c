/**
 ******************************************************************************
 * @file    stm32l4xx_UART.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   This file provides the global UART implementation
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
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

#include "stm32l4xx_hal.h"
#include "stm32l4xx_UART.h"
#include "stm32l4xx_periph_conf.h"

/* exported variables */

/* Global UART handle */
UART_HandleTypeDef UartHandle;

/* Exported Functions */
/**
 * @brief  Configures UART interface
 * @param  None
 * @retval HAL status
 */
HAL_StatusTypeDef UART_Global_Init(void)
{
  HAL_StatusTypeDef ret_val = HAL_OK;
  if(HAL_UART_GetState(&UartHandle) == HAL_UART_STATE_RESET) {    
    /* I2C configuration */
    UartHandle.Instance        = USART2;
    UartHandle.Init.BaudRate   = USART2_CMN_DEFAULT_BAUDRATE;
    UartHandle.Init.WordLength = USART2_CMN_DEFAULT_WORLDLENGTH;
    UartHandle.Init.StopBits   = USART2_CMN_DEFAULT_STOPBITS;
    UartHandle.Init.Parity     = USART2_CMN_DEFAULT_PARITY;
    UartHandle.Init.HwFlowCtl  = USART2_CMN_DEFAULT_HWFLOWCTL;
    UartHandle.Init.Mode       = USART2_CMN_DEFAULT_MODE;

    ret_val = HAL_UART_Init(&UartHandle);
  }
  return ret_val;
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

