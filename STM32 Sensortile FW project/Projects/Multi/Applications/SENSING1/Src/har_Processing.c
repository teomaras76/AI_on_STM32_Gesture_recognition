/**
 ******************************************************************************
 * @file    har_Processing.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   This file includes activity recognition interface functions
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
#include "TargetFeatures.h"
#include "har_Processing.h"
#include "har_Preprocessing.h"
#include "har_Postprocessing.h"
#ifdef TEST_IGN_WSDM 
  #include "har_ProcessingTest.h"
#endif
#include "ai_platform.h"
#include "ai_utilities.h"
#include "network.h"
#include "network_data.h"

/* Imported Variable -------------------------------------------------------------*/

/* exported Variable -------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define AI_NETWORK_IN_1_HEIGHT  (24) 
#define AI_NETWORK_IN_1_WIDTH   ((ai_int)(AI_NETWORK_IN_1_SIZE/AI_NETWORK_IN_1_HEIGHT))
#define AI_NETWORK_IN_1_FORMAT   AI_BUFFER_FORMAT_FLOAT
#define AI_NETWORK_OUT_1_FORMAT  AI_BUFFER_FORMAT_FLOAT

#ifdef NN_GMP
  #define WINDOW_STEP           (16)
  #define N_OVERLAPPING_WIN     ((ai_int)(AI_NETWORK_IN_1_HEIGHT / WINDOW_STEP) + 1)
#elif (defined(NN_IGN))
  #define N_OVERLAPPING_WIN     (1)
#elif (defined(NN_IGN_WSDM))
  #define N_OVERLAPPING_WIN     (1)
#endif

/* Declaration of network objects from network.h
-> ai_handle is a type void which points a memory space
-> ai_u8 is an unsigned int on 1 byte
*/
AI_ALIGNED(4)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

static ai_handle network = AI_HANDLE_NULL;

static ai_buffer ai_input[AI_NETWORK_IN_NUM]    = { AI_NETWORK_IN_1 };
static ai_buffer ai_output[AI_NETWORK_OUT_NUM]  = { AI_NETWORK_OUT_1 };

static uint8_t last_prediction = AR_ID_NONE;
static HAR_output_t ActivityCode = HAR_NOACTIVITY;

static ai_network_report report;

/* handling samples in a sliding window */
static ai_size n_sample = 0;
static ai_float window_buffer[N_OVERLAPPING_WIN * AI_NETWORK_IN_1_SIZE] = {0};

__STATIC_INLINE HAR_output_t map2HARClasses(uint8_t prediction)
{
  switch(prediction){
    case(AR_ID_STATIONARY): return (HAR_STATIONARY);
    case(AR_ID_WALKING)   : return (HAR_WALKING);
    case(AR_ID_JOGGING)   : return (HAR_JOGGING);
#if defined(NN_IGN_WSDM)
    case(AR_ID_STAIRS)    : return (HAR_STAIRS);
#else
    case(AR_ID_BIKING)    : return (HAR_BIKING);
    case(AR_ID_DRIVING)   : return (HAR_DRIVING);
#endif    
    default:return (HAR_NOACTIVITY);
  }
}

__STATIC_INLINE int aiCheckNetwork(const ai_network_report* report)
{
    if (!report)
        return -1;

    if (aiBufferSize(&report->activations)
            != AI_NETWORK_DATA_ACTIVATIONS_SIZE) {
        SENSING1_PRINTF("E: defined activation buffer size is not coherent (expected=%d)\r\n",
                AI_NETWORK_DATA_ACTIVATIONS_SIZE);
        return -1;
    }

    if (aiBufferSize(&report->weights)
            != AI_NETWORK_DATA_WEIGHTS_SIZE) {
        SENSING1_PRINTF("E: defined weights buffer size is not coherent (expected=%d)\r\n",
                AI_NETWORK_DATA_WEIGHTS_SIZE);
        return -1;
    }

    if ((AI_NETWORK_IN_NUM != report->n_inputs) ||
            (AI_NETWORK_OUT_NUM != report->n_outputs) ||
            (report->n_inputs != 1) || (report->n_outputs != 1)) {
        SENSING1_PRINTF("E: only one input and one output is supported\r\n");
        return -1;
    }

    if ((ai_input[0].format != AI_NETWORK_IN_1_FORMAT) ||
            (ai_output[0].format != AI_NETWORK_OUT_1_FORMAT)) {
        SENSING1_PRINTF("E: input or output format unconsistancy\r\n");

        return -1;
    }

    if (AI_NETWORK_IN_1_WIDTH != ai_input[0].width){
        SENSING1_PRINTF("E: input width unconsistancy\r\n");
        return -1;
    }

    if (AI_NETWORK_IN_1_HEIGHT != ai_input[0].height){
        SENSING1_PRINTF("E: input height unconsistancy\r\n");
        return -1;
    }
 
    return 0;
}

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run activity recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Activity Recognition Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/
#ifdef NN_GMP
HAR_output_t HAR_run(SensorAxesRaw_t ACC_Value_Raw)
{
  static ai_float out[AI_NETWORK_OUT_1_SIZE];

  ai_i32 batch;
  HAR_input_t iDataIN;
  HAR_input_t iDataInPreProc;

  if (AI_HANDLE_NULL == network) {
      SENSING1_PRINTF("E: network handle is NULL\r\n");
      return HAR_NOACTIVITY;
  }
  
  iDataIN.AccX = (float)ACC_Value_Raw.AXIS_X * TargetBoardFeatures.AccSensiMultInG;
  iDataIN.AccY = (float)ACC_Value_Raw.AXIS_Y * TargetBoardFeatures.AccSensiMultInG;
  iDataIN.AccZ = (float)ACC_Value_Raw.AXIS_Z * TargetBoardFeatures.AccSensiMultInG;
  
  iDataInPreProc = gravity_rotate(&iDataIN);
  
  /* add samples to each active window */
  ai_size n_window = n_sample / WINDOW_STEP, pos = n_sample % WINDOW_STEP;
  for (ai_size i = 0; i < N_OVERLAPPING_WIN; ++i) {
    /* avoid partial buffers at start */
    if (n_window < i) continue;

    ai_int win_idx = (n_window - i) % N_OVERLAPPING_WIN;
    ai_int index = pos + i * WINDOW_STEP;
    ai_int win_offset = win_idx * AI_NETWORK_IN_1_SIZE;

    if (index < AI_NETWORK_IN_1_HEIGHT) {
      ai_size j = win_offset + index * AI_NETWORK_IN_1_WIDTH ;
      window_buffer[j++] = iDataInPreProc.AccX;
      window_buffer[j++] = iDataInPreProc.AccY;
      window_buffer[j]   = iDataInPreProc.AccZ;
    }

    /* if buffer is full, run the network */
    if (index == (AI_NETWORK_IN_1_HEIGHT - 1)) {
      ai_input[0].data  = AI_HANDLE_PTR(&window_buffer[win_offset]);
      ai_output[0].data = AI_HANDLE_PTR(out);
      batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
      if (batch != 1) {
        aiLogErr(ai_network_get_error(network),"ai_network_run");
      }
      last_prediction = har_postProc(out);
    }
  }
  ++n_sample;
  
  ActivityCode = map2HARClasses(last_prediction);
  
  return ActivityCode;
}
#elif (defined (NN_IGN) || defined(NN_IGN_WSDM))
HAR_output_t HAR_run(SensorAxesRaw_t ACC_Value_Raw)
{
  static ai_float out[AI_NETWORK_OUT_1_SIZE];
  ai_i32 batch;
  HAR_input_t iDataIN;
  HAR_input_t iDataInPreProc;
  
  if (AI_HANDLE_NULL == network) {
      SENSING1_PRINTF("E: network handle is NULL\r\n");
      return HAR_NOACTIVITY;
  }
  
#ifdef TEST_IGN_WSDM  
  HAR_GetTestSamples(&iDataIN);
#else
  {
    float factor = TargetBoardFeatures.AccSensiMultInG;
#ifdef NN_IGN_WSDM     
    factor *= FROM_G_TO_MS_2 ;
#endif     
    iDataIN.AccX = (float)ACC_Value_Raw.AXIS_X * factor;
    iDataIN.AccY = (float)ACC_Value_Raw.AXIS_Y * factor;
    iDataIN.AccZ = (float)ACC_Value_Raw.AXIS_Z * factor;
  }
#endif  

  /* preprocessing */ 
  iDataInPreProc = gravity_rotate(&iDataIN);
  
  /* add samples to each active window */
  window_buffer[n_sample++] = iDataInPreProc.AccX;
  window_buffer[n_sample++] = iDataInPreProc.AccY;
  window_buffer[n_sample++] = iDataInPreProc.AccZ;
  
  if  ( n_sample >= AI_NETWORK_IN_1_SIZE )
  {
    ai_input[0].data  = AI_HANDLE_PTR(&window_buffer);
    ai_output[0].data = AI_HANDLE_PTR(out);
    batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
    if (batch != 1) {
        aiLogErr(ai_network_get_error(network),"ai_network_run");
    }
    last_prediction = har_postProc(out);
    n_sample         = 0;
  }
  ActivityCode = map2HARClasses(last_prediction);
  
  return ActivityCode;

}
#endif
/**
* @brief  Initialises MotionAR algorithm
* @param  None
* @retval 0 if initilazed OK, a negative value otherwise
*/

int8_t HAR_Initialize(void)
{
  ai_error err;
  if (network != AI_HANDLE_NULL){
	SENSING1_PRINTF("\r\nAI Network already initialized...\r\n");
	return -1;
  }

  last_prediction = AR_ID_NONE;

  SENSING1_PRINTF("\r\nAI Network (AI platform API %d.%d.%d)...\r\n",
          AI_PLATFORM_API_MAJOR,
          AI_PLATFORM_API_MINOR,
          AI_PLATFORM_API_MICRO);

  /* enabling CRC clock for using AI libraries (for checking if STM32 
  microprocessor is used)*/
  __HAL_RCC_CRC_CLK_ENABLE();
    
  /* create an instance of the network */
  SENSING1_PRINTF("Creating the network...\r\n");
  err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  if (err.type) {
      aiLogErr(err, "ai_network_create");
      return -3;
  }

  /* Query the created network to get relevant info from it */
  if (ai_network_get_info(network, &report) ) {
#ifdef SENSING1_ENABLE_PRINTF
      aiPrintNetworkInfo(&report);
#endif
  } else {
      err = ai_network_get_error(network);
      aiLogErr(err, "ai_network_get_info");
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return -4;
  }

  if (aiCheckNetwork(&report))
  {
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return -5;
  }

  /* initialize the instance */
  SENSING1_PRINTF("Initializing the network...\r\n");

  /* build params structure to provide the references of the
   * activation and weight buffers */
  
  const ai_network_params params = {
     AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
     AI_NETWORK_DATA_ACTIVATIONS(activations) };

  if (!ai_network_init(network, &params)) {
      err = ai_network_get_error(network);
      aiLogErr(err, "ai_network_init");
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return -6;
  }
  ActivityCode = HAR_NOACTIVITY;

#ifdef NN_GMP
  SENSING1_PRINTF("Initialized NN_GMP HAR\r\n");
#elif defined (NN_IGN)
  SENSING1_PRINTF("Initialized NN_IGN HAR\r\n");
#elif defined (NN_IGN_WSDM)
  #ifdef TEST_IGN_WSDM  
  HAR_GetTestSamples_Init();
  #endif
  SENSING1_PRINTF("Initialized NN_IGN_WSDM HAR\r\n");
#endif  

    return 0;
}
void HAR_DeInitialize(void)
{
    ai_error err;

    SENSING1_PRINTF("Releasing the network...\r\n");
    if (network == AI_HANDLE_NULL)
        return;
    if (ai_network_destroy(network) != AI_HANDLE_NULL) {
        err = ai_network_get_error(network);
        aiLogErr(err, "ai_network_destroy");
    }
    network = AI_HANDLE_NULL;
    __HAL_RCC_CRC_CLK_DISABLE();

#ifdef TEST_IGN_WSDM  
   HAR_GetTestSamples_DeInit();
#endif

}

/**
 * @brief  get latest activity code computed by Recognition Algorithm
 * @param  None
 * @retval activity index
 */
HAR_output_t HAR_get_Activity_Code(void)
{
  return ActivityCode;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
