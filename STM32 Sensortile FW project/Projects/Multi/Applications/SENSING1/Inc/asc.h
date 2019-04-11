/**
  ******************************************************************************
  * @file    asc.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Audio Scene Classification APIs
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ASC_H_
#define _ASC_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "asc_preprocessing.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  ASC_HOME      = 0x00,
  ASC_OUTDOOR   = 0x01,
  ASC_CAR       = 0x02,
  ASC_UNDEFINED = 0xFF
} ASC_OutputTypeDef;

typedef enum
{
  ASC_OK      = 0x00,
  ASC_ERROR   = 0x01,
} ASC_StatusTypeDef;

/* Exported macros -----------------------------------------------------------*/

/** @defgroup NN_ASC_Exported_Functions NN_ASC_Exported_Functions
 * @{
 */

/* Exported constants --------------------------------------------------------*/
#define FILL_BUFFER_SIZE PROC_BUFFER_SIZE

ASC_StatusTypeDef ASC_Init(void);

ASC_StatusTypeDef ASC_DeInit(void);

ASC_OutputTypeDef ASC_Run(float32_t *pBuffer);

ASC_OutputTypeDef ASC_NN_Run(float32_t *pSpectrogram, float32_t *pNetworkOut);

ASC_OutputTypeDef ASC_GetClassificationCode(void);

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* _ASC_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
