/**
******************************************************************************
* @file    usbd_conf_l4.c
* @author  Central Labs
* @version V1.0.0
* @date    30-Nov-2018
* @brief   This file implements the USB Device library callbacks and MSP
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
#include "usbd_core.h"

PCD_HandleTypeDef hpcd;

/*******************************************************************************
LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/


/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
  
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF(hpcd->pData);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  /* Reset Device */
  USBD_LL_Reset(hpcd->pData);

  /* Set USB Current Speed */
  USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Suspend(hpcd->pData);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Resume(hpcd->pData);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

/**
* @brief  SOF callback.
* @param  hpcd: PCD handle
* @retval None
*/
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}

/*******************************************************************************
LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
/**
* @brief  USBD_LL_Init
*         Initialize the Low Level portion of the Device driver.
* @param  pdev: Device handle
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev)
{
  /* Change Systick prioity */
  NVIC_SetPriority (SysTick_IRQn, 0);

  /* Set LL Driver parameters */
  hpcd.Instance = USB_OTG_FS;
  hpcd.Init.dev_endpoints = 4;
  hpcd.Init.use_dedicated_ep1 = 0;
  hpcd.Init.ep0_mps = 0x40;
  hpcd.Init.dma_enable = 0;
  hpcd.Init.low_power_enable = 0;
  hpcd.Init.lpm_enable = 0;
  hpcd.Init.battery_charging_enable = 0;
  hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd.Init.Sof_enable = 0;
  hpcd.Init.speed = PCD_SPEED_FULL;
  hpcd.Init.vbus_sensing_enable = 0;
  /* Link The driver to the stack */
  hpcd.pData = pdev;
  pdev->pData = &hpcd;
  /* Initialize LL Driver */
  HAL_PCD_Init(&hpcd);

  /* configure EPs FIFOs */
  HAL_PCD_SetRxFiFo(&hpcd, 64);
  HAL_PCD_SetTxFiFo(&hpcd, 0, 32);
  HAL_PCD_SetTxFiFo(&hpcd, 1, 64);
  HAL_PCD_SetTxFiFo(&hpcd, 2, 64);
  HAL_PCD_SetTxFiFo(&hpcd, 3, 36);

  return USBD_OK;
}

/**
* @brief  USBD_LL_DeInit
*         De-Initialize the Low Level portion of the Device driver.
* @param  pdev: Device handle
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK;
}

/**
* @brief  USBD_LL_Start
*         Start the Low Level portion of the Device driver.
* @param  pdev: Device handle
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}

/**
* @brief  USBD_LL_Stop
*         Stop the Low Level portion of the Device driver.
* @param  pdev: Device handle
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK;
}

/**
* @brief  USBD_LL_OpenEP
*         Open an endpoint of the Low Level Driver.
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @param  ep_type: Endpoint Type
* @param  ep_mps: Endpoint Max Packet Size
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev,
                                     uint8_t  ep_addr,
                                     uint8_t  ep_type,
                                     uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData,
                  ep_addr,
                  ep_mps,
                  ep_type);

  return USBD_OK;
}

/**
* @brief  USBD_LL_CloseEP
*         Close an endpoint of the Low Level Driver.
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
* @brief  USBD_LL_FlushEP
*         Flush an endpoint of the Low Level Driver.
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
* @brief  USBD_LL_StallEP
*         Set a Stall condition on an endpoint of the Low Level Driver.
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
* @brief  USBD_LL_ClearStallEP
*         Clear a Stall condition on an endpoint of the Low Level Driver.
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
* @brief  USBD_LL_IsStallEP
*         Return Stall condition.
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @retval Stall (1: yes, 0: No)
*/
uint8_t USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}
/**
* @brief  USBD_LL_SetDevAddress
*         Assign an USB address to the device
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}

/**
* @brief  USBD_LL_Transmit
*         Transmit data over an endpoint
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @param  pbuf:pointer to data to be sent
* @param  size: data size
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev,
                                      uint8_t  ep_addr,
                                      uint8_t  *pbuf,
                                      uint16_t  size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
* @brief  USBD_LL_PrepareReceive
*         prepare an endpoint for reception
* @param  pdev: device handle
* @param  ep_addr: Endpoint Number
* @param  pbuf:pointer to data to be received
* @param  size: data size
* @retval USBD Status
*/
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                           uint8_t  ep_addr,
                                           uint8_t  *pbuf,
                                           uint16_t  size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
* @brief  USBD_LL_GetRxDataSize
*         Return the last transfered packet size.
* @param  phost: Device handle
* @param  ep_addr: Endpoint Number
* @retval Recived Data Size
*/
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr)
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
* @brief  USBD_LL_Delay
*         Delay routine for the USB Device Library
* @param  Delay: Delay in ms
* @retval None
*/
void  USBD_LL_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
