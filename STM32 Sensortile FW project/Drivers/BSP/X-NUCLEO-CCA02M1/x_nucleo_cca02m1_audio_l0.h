/**
******************************************************************************
* @file    x_nucleo_cca02m1_audio_l0.h
* @author  Central Labs
* @version V3.0.0
* @date    21-March-2018
* @brief   This file contains the common defines and functions prototypes for
*          x_nucleo_cca02m1_audio_l0.c driver.
******************************************************************************
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_CCA02M1_AUDIO_L0_H
#define __X_NUCLEO_CCA02M1_AUDIO_L0_H

#ifdef __cplusplus
extern "C" {
#endif 
  
  /* Includes ------------------------------------------------------------------*/
  #include "../../../Middlewares/ST/STM32_Audio/Addons/PDM/pdm_filter.h"

  /** @addtogroup BSP
  * @{
  */
  
  /** @addtogroup X-NUCLEO-CCA02M1
  * @{
  */
  
  /** @addtogroup X-NUCLEO-CCA02M1_AUDIO_L0
  * @{
  */
  
  /** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Private_Types 
  * @{
  */  

  /** 
  * @brief   Microphone internal structure definition  
  */ 
  typedef struct
  {      
   uint32_t MicChannels;       /*!< Specifies the number of channels */
  
  uint32_t PdmBufferSize;     /*!< Specifies the size of the PDM double buffer for 1 microphone and 1 ms in bytes*/
  
  uint32_t PCM_Sampling_Freq;     /*!< Specifies the desired sampling frequency */
  
  uint32_t PDM_Clock_Freq;     /*!< Specifies the desired sampling frequency */
  
  uint32_t DecimationFactor;  /*!< Specifies the PDM to PCM decimation factor */
  
  uint16_t * PDM_Data;      /*!< Takes track of the external PDM data buffer as passed by the user in the start function*/
  
  }
  X_NUCLEO_CCA02M1_HandlerTypeDef;
  
  /**
  * @}
  */  
  
  /** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Exported_Constants 
  * @{
  */ 
  
    /* I2S Configuration defines */
#define AUDIO_IN_I2S_INSTANCE                                            SPI2
#define AUDIO_IN_I2S_CLK_ENABLE()                               __SPI2_CLK_ENABLE()
#define AUDIO_IN_I2S_SCK_PIN                                    GPIO_PIN_13
#define AUDIO_IN_I2S_SCK_GPIO_PORT                              GPIOB
#define AUDIO_IN_I2S_SCK_GPIO_CLK_ENABLE()                      __GPIOB_CLK_ENABLE()
#define AUDIO_IN_I2S_SCK_AF                                     GPIO_AF0_SPI2  
#define AUDIO_IN_I2S_MOSI_PIN                                   GPIO_PIN_15
#define AUDIO_IN_I2S_MOSI_GPIO_PORT                             GPIOB
#define AUDIO_IN_I2S_MOSI_GPIO_CLK_ENABLE()                     __GPIOB_CLK_ENABLE();
#define AUDIO_IN_I2S_MOSI_AF                                    GPIO_AF0_SPI2
  
  /* I2S DMA definitions */
#define AUDIO_IN_I2S_DMAx_INSTANCE                              DMA1_Channel4
#define AUDIO_IN_I2S_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()    
#define AUDIO_IN_I2S_DMAx_IRQ                                   DMA1_Channel4_5_6_7_IRQn 
#define AUDIO_IN_I2S_DMAx_PERIPH_DATA_SIZE                      DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_I2S_DMAx_MEM_DATA_SIZE                         DMA_MDATAALIGN_HALFWORD  
#define AUDIO_IN_I2S_IRQHandler                                 DMA1_Channel4_5_6_7_IRQHandler
  
  /* Select the interrupt preemption priority  */
#define AUDIO_IN_IRQ_PREPRIO          6   
  
  /* SPI Configuration defines */  
#define AUDIO_IN_SPI_INSTANCE                                            SPI1
#define AUDIO_IN_SPI_CLK_ENABLE()                               __SPI1_CLK_ENABLE()
#define AUDIO_IN_SPI_SCK_GPIO_CLK_ENABLE()                      __GPIOA_CLK_ENABLE()
#define AUDIO_IN_SPI_MISO_GPIO_CLK_ENABLE()                     __GPIOA_CLK_ENABLE() 
#define AUDIO_IN_SPI_MOSI_GPIO_CLK_ENABLE()                     __GPIOA_CLK_ENABLE()   
#define AUDIO_IN_SPI_FORCE_RESET()                              __SPI1_FORCE_RESET()
#define AUDIO_IN_SPI_RELEASE_RESET()                            __SPI1_RELEASE_RESET()
#define AUDIO_IN_SPI_SCK_PIN                                    GPIO_PIN_5
#define AUDIO_IN_SPI_SCK_GPIO_PORT                              GPIOA
#define AUDIO_IN_SPI_SCK_AF                                     GPIO_AF0_SPI1
#define AUDIO_IN_SPI_MOSI_PIN                                   GPIO_PIN_7
#define AUDIO_IN_SPI_MOSI_GPIO_PORT                             GPIOA
#define AUDIO_IN_SPI_MOSI_AF                                    GPIO_AF0_SPI1
  
  /* SPI DMA definitions */
#define AUDIO_IN_SPI_DMAx_CLK_ENABLE()                          __DMA1_CLK_ENABLE()
#define AUDIO_IN_SPI_DMAx_INSTANCE                              DMA1_Channel2
#define AUDIO_IN_SPI_RX_DMA_STREAM                              DMA2_Stream2 
#define AUDIO_IN_SPI_DMA_RX_IRQn                                DMA2_Stream2_IRQn
#define AUDIO_IN_SPI_DMA_RX_IRQHandler                          DMA2_Stream2_IRQHandler
  
  /* AUDIO TIMER definitions */
#define AUDIO_IN_TIMER                                     TIM22
#define AUDIO_IN_TIMER_CLK_ENABLE()                        __TIM22_CLK_ENABLE()  
#define AUDIO_IN_TIMER_CHOUT_AF                            GPIO_AF4_TIM22
#define AUDIO_IN_TIMER_CHOUT_PIN                           GPIO_PIN_5
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT                     GPIOB
#define AUDIO_IN_TIMER_CHOUT_GPIO_PORT_CLK_ENABLE()        __GPIOB_CLK_ENABLE()  
#define AUDIO_IN_TIMER_CHIN_AF                             GPIO_AF4_TIM22
#define AUDIO_IN_TIMER_CHIN_PIN                            GPIO_PIN_4
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT                      GPIOB
#define AUDIO_IN_TIMER_CHIN_GPIO_PORT_CLK_ENABLE()         __GPIOB_CLK_ENABLE()   
  
  /*------------------------------------------------------------------------------
             CONFIGURATION: Audio Driver Configuration parameters
------------------------------------------------------------------------------*/

  /* Audio status definition */     
#ifndef AUDIO_OK
#define AUDIO_OK                            ((uint8_t)0)
#endif  

#ifndef AUDIO_ERROR
#define AUDIO_ERROR                         ((uint8_t)1)
#endif 

#ifndef AUDIO_TIMEOUT
#define AUDIO_TIMEOUT                       ((uint8_t)2)
#endif 

#define AUDIOFREQ_48K                ((uint32_t)48000)
#define AUDIOFREQ_32K                ((uint32_t)32000)
#define AUDIOFREQ_16K                ((uint32_t)16000)
#define AUDIOFREQ_8K                 ((uint32_t)8000)


/* Those defines are used to allocate the right amount of RAM depending on the
   maximum number of microphone and frequency desired */
#define MAX_MIC_FREQ                 	  2048  /*KHz*/
#define MAX_AUDIO_IN_CHANNEL_NBR_PER_IF   2 
#define MAX_AUDIO_IN_CHANNEL_NBR_TOTAL    2 

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT               1
#define PDM_FREQ_16K                     1280 //2048

/*BSP internal buffer size in half words (16 bits)*/  
#define PDM_INTERNAL_BUFFER_SIZE_I2S          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)
#if MAX_AUDIO_IN_CHANNEL_NBR_TOTAL > 2
#define PDM_INTERNAL_BUFFER_SIZE_SPI          ((MAX_MIC_FREQ / 8) * MAX_AUDIO_IN_CHANNEL_NBR_PER_IF * N_MS_PER_INTERRUPT)
#endif

typedef enum
{
  AUDIO_IN_FS_8000 = 8000,
  AUDIO_IN_FS_16000 = 16000,
  AUDIO_IN_FS_32000 = 32000,
  AUDIO_IN_FS_48000 = 48000,
  AUDIO_IN_FS_8000_LP,
  AUDIO_IN_FS_16000_LP,
  AUDIO_IN_FS_32000_LP,
  AUDIO_IN_FS_48000_LP
}
AUDIO_IN_PCM_Output;

typedef enum
{
  AUDIO_IN_PDM_768 = 0,
  AUDIO_IN_PDM_1280,
  AUDIO_IN_PDM_2048,
  AUDIO_IN_PDM_3072
}
AUDIO_IN_PDM_Input;

  
  /**
  * @}
  */ 
  
  /** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Exported_Variables 
  * @{
  */ 
  extern I2S_HandleTypeDef                hAudioInI2s;
  /**
  * @}
  */ 
  
  /** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Exported_Macros
  * @{
  */
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
  
  /**
  * @}
  */
  
  /** @defgroup X-NUCLEO-CCA02M1_AUDIO_L0_Exported_Functions 
  * @{
  */   
  
  uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
  uint8_t BSP_AUDIO_IN_Record(uint16_t* pbuf, uint32_t size);
  uint8_t BSP_AUDIO_IN_Stop(void);
  uint8_t BSP_AUDIO_IN_Pause(void);
  uint8_t BSP_AUDIO_IN_Resume(void);
  uint8_t BSP_AUDIO_IN_SetVolume(uint8_t Volume);
  uint8_t BSP_AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf);
  uint8_t BSP_AUDIO_IN_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
  uint8_t BSP_AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);

  /* User Callbacks: user has to implement these functions in his code if they are needed. */
  /* This function should be implemented by the user application.
     It is called into this driver when the current buffer is filled to prepare the next
     buffer pointer and its size. */
  void BSP_AUDIO_IN_TransferComplete_CallBack(void);
  void BSP_AUDIO_IN_HalfTransfer_CallBack(void);
  void BSP_AUDIO_IN_Error_Callback(void);
  
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
  
#ifdef __cplusplus
}
#endif

#endif /* __X_NUCLEO_CCA02M1_AUDIO_L0_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
