 /**
 ******************************************************************************
 * @file    asc_preprocessing.h
 * @author  Central Lab
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   Header for asc_preprocessing.c
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
#ifndef _ASC_PREPROCESSING_H_
#define _ASC_PREPROCESSING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"

#define PROC_BUFFER_SIZE 1024

#define NMELS            30

#define SPECTROGRAM_ROWS NMELS
#define SPECTROGRAM_COLS 32

/* Exported Functions Prototypes ---------------------------------------------*/
void ASC_LogMelSpectrogram(float32_t *pSpectrogram);
void ASC_MelColumn(float32_t *pInSignal, uint32_t col, float32_t *pOutSpectrogram);

#ifdef __cplusplus
}
#endif

#endif /* __ASC_PREPROCESSING_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

