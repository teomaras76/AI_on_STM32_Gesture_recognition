/**
  ******************************************************************************
  * @file    har_Postprocessing.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   API defined on har_Postprocessing.c file
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
#ifndef __GESTURE_POSTPROCESSING_H_
#define __GESTURE_POSTPROCESSING_H_

/* Include ------------------------------------------------------------------*/
#include "gesture_Processing.h"
#include <stdint.h>
/* Defines ------------------------------------------------------------------*/


#define FILT_ALPHA 0.2

/* Functions --------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/
//uint8_t argmax(const float * array, int size);
//uint8_t fmlp_temporal_filter(uint8_t prediction);
//const float * exp_average(float * scores, float alpha);
//uint8_t adapt_prediction(uint8_t estimate_prediction);
uint8_t gesture_postProc(float * scores);

#endif /* __HAR_POSTPROCESSING_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/