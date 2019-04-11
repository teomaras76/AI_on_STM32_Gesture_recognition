/**
  ******************************************************************************
  * @file    asc.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Audio Scene Classification algorithm
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
#include "asc.h"
#include "SENSING1_config.h"
#include "ai_platform.h"
#include "ai_utilities.h"
#include "network.h"
#include "network_data.h"
#include "featureScaler.h"
#include "stm32l4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static float32_t aSpectrogram[SPECTROGRAM_ROWS * SPECTROGRAM_COLS];
static uint32_t SpectrColIndex;

static ai_buffer input  = AI_NETWORK_IN_1;
static ai_buffer output = AI_NETWORK_OUT_1;

static ai_handle network = AI_HANDLE_NULL;
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

static ASC_OutputTypeDef ClassificationCode = ASC_UNDEFINED;

static ai_network_report report;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Create and Init ASC Convolutional Neural Network
  *
  * @retval ASC Status
  */
ASC_StatusTypeDef ASC_Init(void)
{
  const ai_network_params net_params = {
    AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
    AI_NETWORK_DATA_ACTIVATIONS(activations)
  };
  ai_error error;

  ClassificationCode = ASC_UNDEFINED;

  if (network != AI_HANDLE_NULL)
  {
    SENSING1_PRINTF("\r\nAI Network already initialized...\r\n");
    return ASC_ERROR;
  }

  SENSING1_PRINTF("\r\nAI Network (AI platform API %d.%d.%d)...\r\n",
                  AI_PLATFORM_API_MAJOR,
                  AI_PLATFORM_API_MINOR,
                  AI_PLATFORM_API_MICRO);

  /* Enable CRC clock to use AI Libraries */
  __HAL_RCC_CRC_CLK_ENABLE();

  /* Create network */
  SENSING1_PRINTF("Creating network...\r\n");
  error = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  if (error.type != AI_ERROR_NONE)
  {
      aiLogErr(error, "ai_network_create");
      return ASC_ERROR;
  }

  /* Show network info */
  if (ai_network_get_info(network, &report) ) {
#ifdef SENSING1_ENABLE_PRINTF
      aiPrintNetworkInfo(&report);
#endif
  } else {
      error = ai_network_get_error(network);
      aiLogErr(error, "ai_network_get_info");
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return ASC_ERROR;
  }

  /* Initialize network */
  SENSING1_PRINTF("Initializing network...\r\n");
  if (!ai_network_init(network, &net_params)) {
    error = ai_network_get_error(network);
    aiLogErr(error, "ai_network_init");
    ai_network_destroy(&network);
    network = AI_HANDLE_NULL;
    return ASC_ERROR;
  }

  SENSING1_PRINTF("Initialized NN_ASC\r\n");
  return ASC_OK;

}

/**
  * @brief  DeInit ASC Convolutional Neural Network
  *
  * @retval ASC Status
  */
ASC_StatusTypeDef ASC_DeInit(void)
{
  ai_error error;

  SENSING1_PRINTF("Releasing network...\r\n");

  /* Check the network handle allocation */
  if (network == AI_HANDLE_NULL)
  {
    return ASC_ERROR;
  }

  if (ai_network_destroy(network) != AI_HANDLE_NULL)
  {
    error = ai_network_get_error(network);
    aiLogErr(error, "ai_network_destroy");
  }

  network = AI_HANDLE_NULL;

  /* Disable CRC Clock */
  __HAL_RCC_CRC_CLK_DISABLE();

  return ASC_OK;

}

/**
 * @brief  Run Acoustic Scene Recognition (ASC) algorithm.
 * @note   This function needs to be executed multiple times to extract audio features
 *
 * @retval Classification result code
 */
ASC_OutputTypeDef ASC_Run(float32_t *pBuffer)
{
  ai_float dense_2_out[AI_NETWORK_OUT_1_SIZE] = {0.0, 0.0, 0.0};

  /* Create a Mel-scaled spectrogram column */
  ASC_MelColumn(pBuffer, SpectrColIndex, aSpectrogram);
  SpectrColIndex++;

  if (SpectrColIndex == SPECTROGRAM_COLS)
  {
    SpectrColIndex = 0;

    /* Convert to LogMel-scaled Spectrogram */
    ASC_LogMelSpectrogram(aSpectrogram);

    /* Run AI Network */
    ClassificationCode = ASC_NN_Run(aSpectrogram, dense_2_out);
    return ClassificationCode;
  }
  else
  {
    return ASC_UNDEFINED;
  }

}

/**
 * @brief  Get classification code computed by the ASC algorithm
 *
 * @retval Classification result
 */
ASC_OutputTypeDef ASC_GetClassificationCode(void)
{
  return ClassificationCode;
}

/**
 * @brief      ASC Convolutional Neural Net inference
 * @param[in]  pSpectrogram The CNN feature input
 * @param[out] pNetworkOut  The CNN output
 *
 * @retval Classification result
 */
ASC_OutputTypeDef ASC_NN_Run(float32_t *pSpectrogram, float32_t *pNetworkOut)
{
  uint32_t i;
  uint32_t classification_result;
  ai_i32 batch = 0;
  ai_float max_out;

  /* Z-Score Scaling on input feature */
  for (i = 0; i < SPECTROGRAM_ROWS * SPECTROGRAM_COLS; i++)
  {
    pSpectrogram[i] = (pSpectrogram[i] - featureScalerMean[i]) / featureScalerStd[i];
  }

  /* I/O Buffers assignment */
  input.data  = AI_HANDLE_PTR((const ai_float *) pSpectrogram);
  output.data = AI_HANDLE_PTR(pNetworkOut);

  /* Neural Network inference */
  batch = ai_network_run(network, &input, &output);
  if (batch != 1)
  {
    aiLogErr(ai_network_get_error(network),"ai_network_run");
    while(1);
  }

  /* ArgMax to associate NN output with the most likely classification label */
  max_out = pNetworkOut[0];
  classification_result = 0;
  for (i = 1; i < 3; i++)
  {
    if (pNetworkOut[i] > max_out)
    {
      max_out = pNetworkOut[i];
      classification_result = i;
    }
  }

  return (ASC_OutputTypeDef) classification_result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
