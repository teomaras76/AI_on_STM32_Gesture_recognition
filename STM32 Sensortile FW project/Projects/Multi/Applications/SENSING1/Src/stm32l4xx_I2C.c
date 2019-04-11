/**
 ******************************************************************************
 * @file    stm32l4xx_I2C.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   This file provides the global I2C implementation
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
#include "stm32l4xx_I2C.h"
#include "stm32l4xx_periph_conf.h"

/* exported variables */

/* Global I2C handle */
I2C_HandleTypeDef I2CHandle;


/* Exported Functions */
/**
 * @brief  Configures I2C interface
 * @param  None
 * @retval Driver status
 */
DrvStatusTypeDef I2C_Global_Init(void)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;
  if(HAL_I2C_GetState(&I2CHandle) == HAL_I2C_STATE_RESET) {    
    /* I2C configuration */
    I2CHandle.Instance = I2C1;
    I2CHandle.Init.Timing = I2C1_CMN_DEFAULT_TIMING;
    I2CHandle.Init.OwnAddress1 = I2C1_CMN_DEFAULT_OWNADDRESS1;
    I2CHandle.Init.AddressingMode = I2C1_CMN_DEFAULT_ADDRESSINGMODE;
    I2CHandle.Init.DualAddressMode = I2C1_CMN_DEFAULT_DUALADDRESSMODE;
    I2CHandle.Init.OwnAddress2 = I2C1_CMN_DEFAULT_OWNADDRESS2;
    I2CHandle.Init.GeneralCallMode = I2C1_CMN_DEFAULT_GENERALCALLMODE;
    I2CHandle.Init.NoStretchMode = I2C1_CMN_DEFAULT_NOSTRETCHMODE;

    /* Init the I2C */
    if( HAL_I2C_Init(&I2CHandle) )
      ret_val= COMPONENT_ERROR;
    else
      ret_val= COMPONENT_OK;
  }
  return ret_val;
}

/**
 * @brief  Manages error callback by re-initializing I2C
 * @param  None
 * @retval None
 */
void I2C_Global_Error(uint8_t Addr)
{
    /* De-initialize the I2C comunication bus */
    HAL_I2C_DeInit(&I2CHandle);    

    HAL_Delay(1);

    /* Re-Initiaize the I2C comunication bus */
    I2C_Global_Init();
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

