/**
 ******************************************************************************
 * @file    stm32l4xx_SPI.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   This file provides the global SPI implementation
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
#include "stm32l4xx_SPI.h"
#include "stm32l4xx_periph_conf.h"

/* exported variables */

/* Global SPI handle */
SPI_HandleTypeDef SpiHandle;

/* Local prototypes */

/* Exported Functions */
/**
 * @brief  Configures SPI interface (Master)
 * @param  None
 * @retval HAL status
 */
HAL_StatusTypeDef SPI_Global_Init(void)
{
  HAL_StatusTypeDef ret_val = HAL_OK;
  if(SpiHandle.State != HAL_SPI_STATE_READY) {
    SpiHandle.Instance = SPI1;
    SpiHandle.Init.Mode = SPI1_CMN_DEFAULT_MODE;
    SpiHandle.Init.Direction = SPI1_CMN_DEFAULT_DIRECTON;
    SpiHandle.Init.DataSize = SPI1_CMN_DEFAULT_DATASIZE;
    SpiHandle.Init.CLKPolarity = SPI1_CMN_DEFAULT_CLKPOLARITY;
    SpiHandle.Init.CLKPhase = SPI1_CMN_DEFAULT_CLKPHASE;
    SpiHandle.Init.NSS = SPI1_CMN_DEFAULT_NSS;
    SpiHandle.Init.FirstBit = SPI1_CMN_DEFAULT_FIRSTBIT;
    SpiHandle.Init.TIMode = SPI1_CMN_DEFAULT_TIMODE;
    SpiHandle.Init.CRCPolynomial = SPI1_CMN_DEFAULT_CRCPOLYNOMIAL;
    SpiHandle.Init.BaudRatePrescaler = SPI1_CMN_DEFAULT_BAUNDRATEPRESCALER;
    SpiHandle.Init.CRCCalculation = SPI1_CMN_DEFAULT_CRCCALCULATION;

    ret_val = HAL_SPI_Init(&SpiHandle);
  }
  return ret_val;
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

