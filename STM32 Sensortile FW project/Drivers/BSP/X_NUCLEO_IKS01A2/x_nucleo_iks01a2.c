/**
 ******************************************************************************
 * @file    x_nucleo_iks01a2.c
 * @author  MEMS Application Team
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   This file provides X_NUCLEO_IKS01A2 MEMS shield board specific functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

#include "x_nucleo_iks01a2.h"

#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
#include "cmsis_os.h"
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2 X_NUCLEO_IKS01A2
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_IO IO
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_IO_Private_Variables Private variables
 * @{
 */

static uint32_t I2C_EXPBD_Timeout = BOARD_I2C_EXPBD_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef I2C_EXPBD_Handle;

/**
 * @}
 */

/* Link function for sensor peripheral */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

static void I2C_EXPBD_MspInit( void );
static void I2C_EXPBD_Error( uint8_t Addr );
static DrvStatusTypeDef I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static DrvStatusTypeDef I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static DrvStatusTypeDef I2C_EXPBD_Init( void );
static DrvStatusTypeDef I2C_EXPBD_DeInit( void );

#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
static osSemaphoreId ExpBDI2cSemaphore;
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

/** @addtogroup X_NUCLEO_IKS01A2_IO_Public_Functions Public functions
 * @{
 */

/**
 * @brief  Configures sensor I2C interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_Init( void )
{
  return I2C_EXPBD_Init();
}

/**
 * @brief  Deinitialize sensor and I2C interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_DeInit( void )
{
  return I2C_EXPBD_DeInit();
}

/**
 * @brief  Configures sensor interrupts interface for LSM6DSL sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DSL_Sensor_IO_ITConfig( void )
{
#if defined(LSM6DSL_INT1_O_PIN)
  {
    /* At the moment this feature is only implemented for LSM6DS3 */
    GPIO_InitTypeDef GPIO_InitStructureInt = {0};

    /* Enable LSM6DSL INT1 GPIO clock */
    LSM6DSL_INT1_O_GPIO_CLK_ENABLE();

    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStructureInt.Pin   = LSM6DSL_INT1_O_PIN;
    GPIO_InitStructureInt.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStructureInt.Speed = BOARD_I2C_EXPBD_GPIO_SPEED;
    GPIO_InitStructureInt.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(LSM6DSL_INT1_O_GPIO_PORT, &GPIO_InitStructureInt);

    /* Enable and set EXTI Interrupt priority */
    HAL_NVIC_SetPriority(LSM6DSL_INT1_O_EXTI_IRQn, BOARD_I2C_EXPBD_IRQ_PRI_LEVEL, 0x00);
    HAL_NVIC_EnableIRQ(LSM6DSL_INT1_O_EXTI_IRQn);
  }
#endif /* LSM6DSL_INT1_O_PIN */

#if defined(LSM6DSL_INT2_O_PIN)
  {
    /* At the moment this feature is only implemented for LSM6DS3 */
    GPIO_InitTypeDef GPIO_InitStructureInt = {0};

    /* Enable LSM6DSL INT2 GPIO clock */
    LSM6DSL_INT2_O_GPIO_CLK_ENABLE();

    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStructureInt.Pin   = LSM6DSL_INT2_O_PIN;
    GPIO_InitStructureInt.Mode  = GPIO_MODE_IT_RISING;
    GPIO_InitStructureInt.Speed = BOARD_I2C_EXPBD_GPIO_SPEED;
    GPIO_InitStructureInt.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(LSM6DSL_INT2_O_GPIO_PORT, &GPIO_InitStructureInt);

    /* Enable and set EXTI Interrupt priority */
    HAL_NVIC_SetPriority(LSM6DSL_INT2_O_EXTI_IRQn, BOARD_I2C_EXPBD_IRQ_PRI_LEVEL, 0x00);
    HAL_NVIC_EnableIRQ(LSM6DSL_INT2_O_EXTI_IRQn);
  }
#endif /* LSM6DSL_INT2_O_PIN */

  return COMPONENT_OK;
}



/**
 * @brief  Configures sensor interrupts interface for LPS22HB sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LPS22HB_Sensor_IO_ITConfig( void )
{
#if defined(LPS22H_INT1_O_PIN)
  /* At the moment this feature is only implemented for LPS22HB */
  GPIO_InitTypeDef GPIO_InitStructureInt1;

  /* Enable INT1 GPIO clock */
  LPS22H_INT1_O_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin   = LPS22H_INT1_O_PIN;
  GPIO_InitStructureInt1.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = BOARD_I2C_EXPBD_GPIO_SPEED;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(LPS22H_INT1_O_GPIO_PORT, &GPIO_InitStructureInt1);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(LPS22H_INT1_O_EXTI_IRQn, BOARD_I2C_EXPBD_IRQ_PRI_LEVEL, 0x00);
  HAL_NVIC_EnableIRQ(LPS22H_INT1_O_EXTI_IRQn);
#endif /* LPS22H_INT1_O_PIN */

  return COMPONENT_OK;
}

/**
 * @}
 */


/** @addtogroup X_NUCLEO_IKS01A2_IO_Private_Functions Private functions
 * @{
 */

/******************************* Link functions *******************************/

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  DrvStatusTypeDef   ret = COMPONENT_OK;

  switch(ctx->who_am_i)
  {
    case IKS01A2_LSM303AGR_ACC_WHO_AM_I:
    case IKS01A2_LSM303AGR_MAG_WHO_AM_I:
    case IKS01A2_HTS221_WHO_AM_I:
    {
      if ( nBytesToWrite > 1 ) WriteAddr |= 0x80;  /* Enable I2C multi-bytes Write */

      /* call I2C_EXPBD Write data bus function */
      ret = I2C_EXPBD_WriteData( ctx->address, WriteAddr, pBuffer, nBytesToWrite );
    }
    break;
    case IKS01A2_LPS22HB_WHO_AM_I:
    {
      /* I2C multi-bytes Write not supported for LPS22HB */
      int i = 0;

      for (i = 0; ((i < nBytesToWrite) && (ret == COMPONENT_OK)); i++ )
      {
        /* call I2C_EXPBD Write data bus function */
        ret = I2C_EXPBD_WriteData( ctx->address, (WriteAddr + i), &pBuffer[i], 1 );
      }
    }
    break;
    case IKS01A2_LSM6DSL_WHO_AM_I:
    default:
    {
      /* call I2C_EXPBD Write data bus function */
      ret = I2C_EXPBD_WriteData( ctx->address, WriteAddr, pBuffer, nBytesToWrite );
    }
    break;
  }

  return (ret == COMPONENT_OK) ? 0 : 1;
}

/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  DrvStatusTypeDef   ret = COMPONENT_OK;

  switch(ctx->who_am_i)
  {
    case IKS01A2_LSM303AGR_ACC_WHO_AM_I:
    case IKS01A2_LSM303AGR_MAG_WHO_AM_I:
    case IKS01A2_HTS221_WHO_AM_I:
    {
      if ( nBytesToRead > 1 ) ReadAddr |= 0x80; /* Enable I2C multi-bytes Read */

      /* call I2C_EXPBD Read data bus function */
      ret = I2C_EXPBD_ReadData( ctx->address, ReadAddr, pBuffer, nBytesToRead );
    }
    break;
    case IKS01A2_LPS22HB_WHO_AM_I:
    {
      /* I2C multi-bytes Read not supported for LPS22HB */
      int i = 0;

      for (i = 0; ((i < nBytesToRead) && (ret == COMPONENT_OK)); i++ )
      {
        /* call I2C_EXPBD Read data bus function */
        ret = I2C_EXPBD_ReadData( ctx->address, (ReadAddr + i), &pBuffer[i], 1 );
      }
    }
    break;
    case IKS01A2_LSM6DSL_WHO_AM_I:
    default:
    {
      /* call I2C_EXPBD Read data bus function */
      ret = I2C_EXPBD_ReadData( ctx->address, ReadAddr, pBuffer, nBytesToRead );
    }
    break;
  }

  return (ret == COMPONENT_OK) ? 0 : 1;
}

/******************************* I2C Routines *********************************/

/**
 * @brief  Configures I2C interface.
 * @param  None
 * @retval COMPONENT_OK    in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef I2C_EXPBD_Init( void )
{
  if( HAL_I2C_GetState( &I2C_EXPBD_Handle) == HAL_I2C_STATE_RESET )
  {
#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
    /* Create semaphore to prevent multiple I2C access */
    osSemaphoreDef(I2C_EXPBD_SEM);
    ExpBDI2cSemaphore = osSemaphoreCreate(osSemaphore(I2C_EXPBD_SEM), 1);
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

    /* Setup the I2C clock speed/timing */
    BOARD_I2C_EXPBD_SET_CLK_SPEED(&I2C_EXPBD_Handle.Init);

    /* I2C_EXPBD peripheral configuration */
    I2C_EXPBD_Handle.Init.OwnAddress1    = 0x33;
    I2C_EXPBD_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_EXPBD_Handle.Instance            = BOARD_I2C_EXPBD;

    /* Init the I2C */
    I2C_EXPBD_MspInit();
    HAL_I2C_Init( &I2C_EXPBD_Handle );

    /* Setup Fast Mode */
    BOARD_I2C_EXPBD_SET_FAST_MODE();
  }

  if( HAL_I2C_GetState( &I2C_EXPBD_Handle) == HAL_I2C_STATE_READY )
  {
    return COMPONENT_OK;
  }
  else
  {
    return COMPONENT_ERROR;
  }
}

/**
  * @brief I2C2 Bus Deinitialization
  * @retval None
  */
static DrvStatusTypeDef I2C_EXPBD_DeInit( void )
{
  if(HAL_I2C_DeInit(&I2C_EXPBD_Handle) == HAL_OK)
  {
#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
    /* Delete semaphore to prevent multiple I2C access */
    osSemaphoreDelete(ExpBDI2cSemaphore);
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

    return COMPONENT_OK;
  }
  else
  {
    return COMPONENT_ERROR;
  }
}

/**
 * @brief  Write data to the register of the device through BUS
 * @param  Addr Device address on BUS
 * @param  Reg The target register address to be written
 * @param  pBuffer The data to be written
 * @param  Size Number of bytes to be written
 * @retval COMPONENT_OK    in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(ExpBDI2cSemaphore, osWaitForever);
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

  status = HAL_I2C_Mem_Write( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                              I2C_EXPBD_Timeout );

#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(ExpBDI2cSemaphore);
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

  /* Check the communication status */
  if( status != HAL_OK )
  {
    /* Execute user timeout callback */
    I2C_EXPBD_Error( Addr );
    return COMPONENT_ERROR;
  }
  else
  {
    return COMPONENT_OK;
  }
}



/**
 * @brief  Read a register of the device through BUS
 * @param  Addr Device address on BUS
 * @param  Reg The target register address to read
 * @param  pBuffer The data to be read
 * @param  Size Number of bytes to be read
 * @retval COMPONENT_OK    in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{
  HAL_StatusTypeDef status = HAL_OK;

#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
  /* Get semaphore to prevent multiple I2C access */
  osSemaphoreWait(ExpBDI2cSemaphore, osWaitForever);
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

  status = HAL_I2C_Mem_Read( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout );

#if defined(X_NUCLEO_IKS01AX_USE_CMSIS_OS)
  /* Release semaphore to prevent multiple I2C access */
  osSemaphoreRelease(ExpBDI2cSemaphore);
#endif /* X_NUCLEO_IKS01AX_USE_CMSIS_OS */

  /* Check the communication status */
  if( status != HAL_OK )
  {
    /* Execute user timeout callback */
    I2C_EXPBD_Error( Addr );
    return COMPONENT_ERROR;
  }
  else
  {
    return COMPONENT_OK;
  }
}

/**
 * @brief  Manages error callback by re-initializing I2C
 * @param  Addr I2C Address
 * @retval None
 */
static void I2C_EXPBD_Error( uint8_t Addr )
{
  /* De-initialize the I2C comunication bus */
  if (I2C_EXPBD_DeInit() == COMPONENT_OK)
  {
    /* Re-Initiaize the I2C comunication bus */
    I2C_EXPBD_Init();
  }
}

/**
 * @brief I2C MSP Initialization
 * @param None
 * @retval None
 */

static void I2C_EXPBD_MspInit( void )
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Setup the I2C clock source */
  BOARD_I2C_EXPBD_SET_CLK_SOURCE();

  /* Enable I2C GPIO clocks */
  BOARD_I2C_EXPBD_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = BOARD_I2C_EXPBD_SCL_PIN | BOARD_I2C_EXPBD_SDA_PIN;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed      = BOARD_I2C_EXPBD_GPIO_SPEED;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = BOARD_I2C_EXPBD_SCL_SDA_AF;

  HAL_GPIO_Init( BOARD_I2C_EXPBD_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

  /* Enable the I2C_EXPBD peripheral clock */
  BOARD_I2C_EXPBD_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  BOARD_I2C_EXPBD_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  BOARD_I2C_EXPBD_RELEASE_RESET();

  /* Enable and set I2C_EXPBD Interrupt priority */
  HAL_NVIC_SetPriority(BOARD_I2C_EXPBD_EV_IRQn, BOARD_I2C_EXPBD_IRQ_PRI_LEVEL, 0x01);
  HAL_NVIC_EnableIRQ(BOARD_I2C_EXPBD_EV_IRQn);

#if (defined (BOARD_I2C_EXPBD_ER_IRQn))
  HAL_NVIC_SetPriority(BOARD_I2C_EXPBD_ER_IRQn, BOARD_I2C_EXPBD_IRQ_PRI_LEVEL, 0x00);
  HAL_NVIC_EnableIRQ(BOARD_I2C_EXPBD_ER_IRQn);
#endif
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
