/**
******************************************************************************
* @file    x_nucleo_cca02m1_audio_f3.c
* @author  Central Labs
* @version V3.0.0
* @date    21-March-2018
* @brief   This file provides the Audio driver for the X-NUCLEO-CCA02M1 
*          expansion board 
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "stm32f3xx_hal.h"
#include "x_nucleo_cca02m1_audio_f3.h"

/** @addtogroup BSP
* @{
*/

/** @addtogroup X-NUCLEO-CCA02M1
* @{
*/

/** @addtogroup X-NUCLEO-CCA02M1_AUDIO_L0
* @brief This file provides set of firmware functions to manage MEMS microphones
*        initialization on STM32L0xx-Nucleo Kit from STMicroelectronics.
* @{
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Types
* @{
*/ 

/**
* @}
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Defines 
* @{
*/ 

/**
* @}
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Macros 
* @{
*/

/**
* @}
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Variables
* @{
*/
#define CHANNEL_DEMUX_MASK                    	0x55

I2S_HandleTypeDef                 hAudioInI2s;

static TIM_HandleTypeDef          TimDividerHandle;
static TIM_SlaveConfigTypeDef     sSlaveConfig;
static TIM_IC_InitTypeDef         sICConfig;
static TIM_OC_InitTypeDef         sOCConfig;

static X_NUCLEO_CCA02M1_HandlerTypeDef X_NUCLEO_CCA02M1_Handler;

static uint16_t I2S_InternalBuffer[PDM_INTERNAL_BUFFER_SIZE_I2S];

static uint16_t AudioInVolume = 64;
static PDMFilter_InitStruct Filter[2];
uint8_t Channel_Demux[128] = {
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f
};

/**
* @}
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Function_Prototypes 
* @{
*/ 

static uint8_t AUDIO_IN_Timer_Init(void);
static uint8_t AUDIO_IN_Timer_Start(void);
static void AUDIO_IN_I2S_MspInit(void);

/**
* @}
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Exported_Functions 
* @{
*/   

/**
* @brief  Initializes audio acquisition recording.
* @param  AudioFreq: Audio frequency to be configured for the peripherals.
* 		  Possible values are 8000, 16000, 32000 or 48000 Hz
* @param  BitRes: Not used.
* @param  ChnlNbr: Number of channels to be recorded.
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr){
  
  /*Set Structure for internal state*/
  X_NUCLEO_CCA02M1_Handler.MicChannels=ChnlNbr;
  X_NUCLEO_CCA02M1_Handler.Sampling_Freq=AudioFreq;
  
    if(X_NUCLEO_CCA02M1_Handler.MicChannels!=1)
  {
	  /*Timer Init*/
	  if (AUDIO_IN_Timer_Init() != AUDIO_OK)
	  {
		  return AUDIO_ERROR;
	  }

	  if(AUDIO_IN_Timer_Start() != AUDIO_OK)
	  {
		  return AUDIO_ERROR;
	  }
  }
  
  switch(X_NUCLEO_CCA02M1_Handler.Sampling_Freq=AudioFreq){   
  case AUDIOFREQ_8K:
    {
      X_NUCLEO_CCA02M1_Handler.DecimationFactor=128;
      X_NUCLEO_CCA02M1_Handler.PdmBufferSize=256;      
      break;
    }
  case AUDIOFREQ_16K:
    {
      X_NUCLEO_CCA02M1_Handler.DecimationFactor=64;
      X_NUCLEO_CCA02M1_Handler.PdmBufferSize=256;     
      break;
    }    
  case AUDIOFREQ_32K:
    {
      X_NUCLEO_CCA02M1_Handler.DecimationFactor=64;
      X_NUCLEO_CCA02M1_Handler.PdmBufferSize=512;     
      
      break;
    }    
  case AUDIOFREQ_48K:
    {
      X_NUCLEO_CCA02M1_Handler.DecimationFactor=64;
      X_NUCLEO_CCA02M1_Handler.PdmBufferSize=768;     
      
      break;
    }
  default:
    {
      return AUDIO_ERROR;
    }
  }
  
  X_NUCLEO_CCA02M1_Handler.PdmBufferSize *= N_MS_PER_INTERRUPT;

  /* Initialize the hAudioInI2s Instance parameter */
  hAudioInI2s.Instance          = AUDIO_IN_I2S_INSTANCE;
  
  /* Disable I2S block */
  __HAL_I2S_DISABLE(&hAudioInI2s);
  
  if(X_NUCLEO_CCA02M1_Handler.MicChannels==1)
  {
    hAudioInI2s.Init.DataFormat   = I2S_DATAFORMAT_16B;
  }
  else
  {
    hAudioInI2s.Init.DataFormat   = I2S_DATAFORMAT_32B;    
  }
  
  if(AudioFreq == AUDIOFREQ_8K)
  {
    hAudioInI2s.Init.AudioFreq    = 4 * AudioFreq;
  }else
  {
    hAudioInI2s.Init.AudioFreq    = 2 * AudioFreq;
  }  
  
  hAudioInI2s.Init.CPOL         = I2S_CPOL_HIGH;
  hAudioInI2s.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
  hAudioInI2s.Init.Mode         = I2S_MODE_MASTER_RX;
  hAudioInI2s.Init.Standard     = I2S_STANDARD_MSB;
  
  
  /* Initialize the I2S peripheral with the structure above */
  if(HAL_I2S_GetState(&hAudioInI2s) == HAL_I2S_STATE_RESET)
  { 
    AUDIO_IN_I2S_MspInit();
  }
  
  if(HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  /*End I2S Init*/ 
  
  /*PDM Filter Init*/
  BSP_AUDIO_IN_PDMToPCM_Init(X_NUCLEO_CCA02M1_Handler.Sampling_Freq,X_NUCLEO_CCA02M1_Handler.MicChannels, X_NUCLEO_CCA02M1_Handler.MicChannels);
  /*End PDM Filter Init*/
  
  return AUDIO_OK;  
}

/**
* @brief  Starts audio recording.
* @param  * pbuf: Buffer that will contain 1 ms of PDM for each microphone.
				  Its dimension must be equal to (in uint16_t words): 
				  ((PCM sampling frequency)/1000 * DecimationFactor * Channels)/16
				  DecimationFactor is equal to 128 for 8000 KHZ sampling frequency, 64 in all the other cases
* @param  size: Not used in this driver.
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
uint8_t BSP_AUDIO_IN_Record(uint16_t * pbuf, uint32_t size)
{   
  
  X_NUCLEO_CCA02M1_Handler.PDM_Data = pbuf;  

  if(HAL_I2S_Receive_DMA(&hAudioInI2s, I2S_InternalBuffer, X_NUCLEO_CCA02M1_Handler.PdmBufferSize/2) != HAL_OK)
  {
    return AUDIO_ERROR;
  }  
  /* Return 0 if all operations are OK */
  return AUDIO_OK; 
}

/**
* @brief  Stops audio recording.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
uint8_t BSP_AUDIO_IN_Stop(void)
{
  if(HAL_I2S_DMAStop(&hAudioInI2s) != HAL_OK)
  {
    return AUDIO_ERROR;
  }   
  /* Return 0 if all operations are OK */
  return AUDIO_OK;  
}

/**
* @brief  Controls the audio in volume level.
* @param  Volume: Volume level to be set for the PDM to PCM conversion. 
		  Values must be in the range from 0 to 64
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
uint8_t BSP_AUDIO_IN_SetVolume(uint8_t Volume)
{
  /* Set the Global variable AudioInVolume */
  AudioInVolume = Volume;  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
* @brief  Converts audio format from PDM to PCM.
* @param  PDMBuf: Pointer to PDM buffer data
* @param  PCMBuf: Pointer to PCM buffer data
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
__weak uint8_t BSP_AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)
{  
 uint32_t index = 0;  
#if N_MS_PER_INTERRUPT == 1
  if(X_NUCLEO_CCA02M1_Handler.DecimationFactor == 128)
  {
    for(index = 0; index < X_NUCLEO_CCA02M1_Handler.MicChannels; index++)
    {
      PDM_Filter_128_LSB(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), AudioInVolume , (PDMFilter_InitStruct *)&Filter[index]);
    }
  }
  
  if(X_NUCLEO_CCA02M1_Handler.DecimationFactor == 64)
  {
    for(index = 0; index < X_NUCLEO_CCA02M1_Handler.MicChannels; index++)
    {
      PDM_Filter_64_LSB(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), AudioInVolume , (PDMFilter_InitStruct *)&Filter[index]);
    }
  }  
#else
  uint32_t index_ms = 0;
  uint16_t PDM_Offset = (X_NUCLEO_CCA02M1_Handler.PdmBufferSize / (2 * N_MS_PER_INTERRUPT)) * X_NUCLEO_CCA02M1_Handler.MicChannels;
  uint16_t PCM_Offset = (X_NUCLEO_CCA02M1_Handler.MicChannels * (X_NUCLEO_CCA02M1_Handler.Sampling_Freq / 1000));
  
  for (index_ms = 0; index_ms < N_MS_PER_INTERRUPT; index_ms ++)
  {
    if(X_NUCLEO_CCA02M1_Handler.DecimationFactor == 128)
    {
      for(index = 0; index < X_NUCLEO_CCA02M1_Handler.MicChannels; index++)
      {
        PDM_Filter_128_LSB(&((uint8_t*)(PDMBuf))[index + index_ms * PDM_Offset], (uint16_t*)&(PCMBuf[index + index_ms * PCM_Offset]), AudioInVolume , (PDMFilter_InitStruct *)&Filter[index]);
      }
    }
    
    if(X_NUCLEO_CCA02M1_Handler.DecimationFactor == 64)
    {
      for(index = 0; index < X_NUCLEO_CCA02M1_Handler.MicChannels; index++)
      {
        PDM_Filter_64_LSB(&((uint8_t*)(PDMBuf))[index + index_ms * PDM_Offset], (uint16_t*)&(PCMBuf[index + index_ms * PCM_Offset]), AudioInVolume , (PDMFilter_InitStruct *)&Filter[index]);
      }
    }  
  }
  
#endif
  
  return AUDIO_OK;
}


/**
* @brief  Pauses the audio file stream.
* @param  None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
uint8_t BSP_AUDIO_IN_Pause(void)
{    
  /* Call the Media layer pause function */
  HAL_I2S_DMAPause(&hAudioInI2s);
  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
* @brief  Resumes the audio file stream.
* @param  None    
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
uint8_t BSP_AUDIO_IN_Resume(void)
{    
  /* Call the Media layer pause/resume function */
  HAL_I2S_DMAResume(&hAudioInI2s);
  
  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}
/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
	     byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
		 written into the buffer that the user indicates when calling the BSP_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  uint32_t index = 0;
  
  switch(X_NUCLEO_CCA02M1_Handler.MicChannels){
  case 1:
    {
      uint16_t * DataTempI2S = &I2S_InternalBuffer[X_NUCLEO_CCA02M1_Handler.PdmBufferSize/4] ;
      for(index = 0; index < X_NUCLEO_CCA02M1_Handler.PdmBufferSize/4; index++)
      {
        X_NUCLEO_CCA02M1_Handler.PDM_Data[index] = HTONS(DataTempI2S[index]);
      }
      break;
    }
    
  case 2:
    {      
      uint16_t * DataTempI2S = &(I2S_InternalBuffer[X_NUCLEO_CCA02M1_Handler.PdmBufferSize/2]);
      uint8_t a,b=0;
      for(index=0; index<X_NUCLEO_CCA02M1_Handler.PdmBufferSize/2; index++) {
        a = ((uint8_t *)(DataTempI2S))[(index*2)];
        b = ((uint8_t *)(DataTempI2S))[(index*2)+1];
        ((uint8_t *)(X_NUCLEO_CCA02M1_Handler.PDM_Data))[(index*2)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
        ((uint8_t *)(X_NUCLEO_CCA02M1_Handler.PDM_Data))[(index*2)+1] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
      }
      break;
    }    

  default:
    {
      
      break;
    }
    
  }
  
  BSP_AUDIO_IN_TransferComplete_CallBack();
}

/**
* @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
	     byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
		 written into the buffer that the user indicates when calling the BSP_AUDIO_IN_Start(...) function.
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  uint32_t index = 0;
  switch(X_NUCLEO_CCA02M1_Handler.MicChannels){
  case 1:
    {
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      for(index = 0; index < X_NUCLEO_CCA02M1_Handler.PdmBufferSize/4; index++)
      {
        X_NUCLEO_CCA02M1_Handler.PDM_Data[index] = HTONS(DataTempI2S[index]);
      }
      break;
    }    
  case 2:
    {      
      uint16_t * DataTempI2S = I2S_InternalBuffer;
      uint8_t a,b=0;
      for(index=0; index<X_NUCLEO_CCA02M1_Handler.PdmBufferSize/2; index++) {
        a = ((uint8_t *)(DataTempI2S))[(index*2)];
        b = ((uint8_t *)(DataTempI2S))[(index*2)+1];
        ((uint8_t *)(X_NUCLEO_CCA02M1_Handler.PDM_Data))[(index*2)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
          Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4;;
          ((uint8_t *)(X_NUCLEO_CCA02M1_Handler.PDM_Data))[(index*2)+1] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
            Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4;
      }      
      break;
    }    
   
  default:
    {      
      break;
    }
    
  }
  
  BSP_AUDIO_IN_HalfTransfer_CallBack();
  
}

/**
* @brief  User callback when record buffer is filled.
* @param  None
* @retval None
*/
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}

/**
* @brief  User callback when record buffer is half filled.
* @param  None
* @retval None
*/
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  /* This function should be implemented by the user application.
  It is called into this driver when the current buffer is filled
  to prepare the next buffer pointer and its size. */
}


/**
* @brief  Audio IN Error callback function
* @param  None
* @retval None
*/
__weak void BSP_AUDIO_IN_Error_Callback(void)
{   
  /* This function is called when an Interrupt due to transfer error on or peripheral
  error occurs. */
}

/**
* @brief I2S error callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the error generated on DMA FIFO: This function 
  should be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */  
  if(hi2s->Instance == AUDIO_IN_I2S_INSTANCE)
  {
    BSP_AUDIO_IN_Error_Callback();
  }
}

/**
* @}
*/ 

/** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Functions 
* @{
*/ 

/**
* @brief  Initialize the PDM library.
* @param  AudioFreq: Audio sampling frequency
* @param  ChnlNbrIn: Number of input audio channels in the PDM buffer
* @param  ChnlNbrOut: Number of desired output audio channels in the  resulting PCM buffer
* @retval None
*/
__weak uint8_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  uint32_t i = 0;
  /* Enable CRC peripheral to unlock the PDM library */
  __CRC_CLK_ENABLE();
  
  for(i = 0; i < ChnlNbrIn; i++)
  {
    /* Filter LP and HP Init */
    Filter[i].LP_HZ = AudioFreq / 2;
    Filter[i].HP_HZ = 10;
    Filter[i].Fs = AudioFreq;
    Filter[i].Out_MicChannels = ChnlNbrOut;
    Filter[i].In_MicChannels = ChnlNbrIn;
    PDM_Filter_Init((PDMFilter_InitStruct *)&Filter[i]);
  }
  return AUDIO_OK;
}
/**
* @brief AUDIO IN I2S MSP Init
* @param None
* @retval None
*/
static void AUDIO_IN_I2S_MspInit(void)
{
  static DMA_HandleTypeDef hdma_i2sRx;
  GPIO_InitTypeDef  GPIO_InitStruct;
  I2S_HandleTypeDef *hi2s = &hAudioInI2s;
  
  /* Enable the I2S2 peripheral clock */
  AUDIO_IN_I2S_CLK_ENABLE();
  
  /* Enable I2S GPIO clocks */
  
  AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE();  
  /* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;  
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_SCK_PIN; 
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_SCK_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin       = AUDIO_IN_I2S_MOSI_PIN ;
  GPIO_InitStruct.Alternate = AUDIO_IN_I2S_MOSI_AF;
  HAL_GPIO_Init(AUDIO_IN_I2S_MOSI_GPIO_PORT, &GPIO_InitStruct); 
  
  /* Enable the DMA clock */
  AUDIO_IN_I2S_DMAx_CLK_ENABLE();
  
//  hdma_i2sRx.Init.Request = DMA_REQUEST_2;
  hdma_i2sRx.Init.Direction = DMA_PERIPH_TO_MEMORY;                     /* M2M transfer mode                */
  hdma_i2sRx.Init.PeriphInc = DMA_PINC_DISABLE;                         /* Peripheral increment mode Enable */
  hdma_i2sRx.Init.MemInc = DMA_MINC_ENABLE;                             /* Memory increment mode Enable     */
  hdma_i2sRx.Init.PeriphDataAlignment = AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE;     /* Peripheral data alignment : Word */
  hdma_i2sRx.Init.MemDataAlignment = AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE;           /* memory data alignment : Word     */
  hdma_i2sRx.Init.Mode = DMA_CIRCULAR;                         			/* Normal DMA mode                  */
  hdma_i2sRx.Init.Priority = DMA_PRIORITY_HIGH;              			/* priority level : high            */
  
  if(hi2s->Instance == AUDIO_IN_I2S_INSTANCE)
  {
    
    
    hdma_i2sRx.Instance = AUDIO_IN_I2S_DMAx_INSTANCE;
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sRx);
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);
    
    
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2sRx);      
  }
  
  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_IN_I2S_DMAx_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(AUDIO_IN_I2S_DMAx_IRQ); 
}


/**
* @brief Audio Timer Init
* @param None
* @retval None
*/
static uint8_t AUDIO_IN_Timer_Init(void){
  
  GPIO_InitTypeDef   GPIO_InitStruct;
  
  /* Enable AUDIO_TIMER clock*/
  AUDIO_IN_TIMER_CLK_ENABLE(); 
  AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE();
  AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE();
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHIN_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHIN_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHIN_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate = AUDIO_IN_TIMER_CHOUT_AF;
  GPIO_InitStruct.Pin = AUDIO_IN_TIMER_CHOUT_PIN;
  HAL_GPIO_Init(AUDIO_IN_TIMER_CHOUT_GPIO_PORT, &GPIO_InitStruct);  
  
  TimDividerHandle.Instance = AUDIO_IN_TIMER;  
  
  /* Configure the Input: channel_1 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter = 0;  
  if(HAL_TIM_IC_ConfigChannel(&TimDividerHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  
  /* Configure TIM1 in Gated Slave mode for the external trigger (Filtered Timer
  Input 1) */
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_EXTERNAL1;
  if( HAL_TIM_SlaveConfigSynchronization(&TimDividerHandle, &sSlaveConfig) != HAL_OK)
  {
    return AUDIO_ERROR;
  }  
  
  /* Initialize TIM3 peripheral in PWM mode*/
  TimDividerHandle.Init.Period            = 1;
  TimDividerHandle.Init.Prescaler         = 0;
  TimDividerHandle.Init.ClockDivision     = 0;
  TimDividerHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  // TimDividerHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_PWM_Init(&TimDividerHandle) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  
  /* Configure the PWM_channel_1  */
  sOCConfig.OCMode     = TIM_OCMODE_PWM1;
  sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOCConfig.Pulse = 1;
  if(HAL_TIM_PWM_ConfigChannel(&TimDividerHandle, &sOCConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    return AUDIO_ERROR;
  }  
  
  return AUDIO_OK;
  
}

/**
* @brief Audio Timer Start
* @param None
* @retval AUDIO_OK in case of success, AUDIO_ERROR otherwise 
*/
static uint8_t AUDIO_IN_Timer_Start(){
  
  if(HAL_TIM_IC_Start(&TimDividerHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    return AUDIO_ERROR;
  }  
  /* Start the Output Compare */
  if(HAL_TIM_OC_Start(&TimDividerHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    return AUDIO_ERROR;
  }  
  
  return AUDIO_OK;
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
