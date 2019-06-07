/**
  ******************************************************************************
  * @file    har_Postprocessing.c
  * @author  Microcontroller Division Team
  * @version V1.0.0
  * @date    30-Nov-2018  
  * @brief   Postprocessing functions
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


/* Include ------------------------------------------------------------------*/
#include "gesture_Postprocessing.h"
#include "network.h"
/*
 **********************
 * FMLP Temporal filter
 **********************
 */
/* Defines ------------------------------------------------------------------*/
#define TF_WINDOW         10
#define TF_WALK_WINDOW    7

#define TF_SLOW_THR       7
#define TF_WALK_SHORT_THR 2
#define TF_WALK_LONG_THR  5

/* Input : Array of float
   Return index of the most high value of the array 
*/
static uint8_t argmax(const float * array, int size) 
{
  float max = -1e9f;
  uint8_t max_idx = 0;
  for (int i = 0; i < size; ++i) {
    if (array[i] > max) {
      max = array[i]; max_idx = i;
    }
  }
  /* Matteo - inserted a treshold to select only high probability result */
  if (max > 0.3){
    return (max_idx);
  }
  else{
    return 0;
  }
}
#if (defined(NN_IGN_WSDM))
static  uint8_t fmlp_temporal_filter(uint8_t prediction)
{
//  printf ( "%d \r\n" , prediction);
  return prediction;
}
#else
static  uint8_t fmlp_temporal_filter(uint8_t prediction)
{
  static uint8_t raw_predictions[TF_WINDOW] = {0};
  /* prevents starting in Driving or Biking */
  static uint8_t last_prediction = AR_ID_STATIONARY;
  
  static uint8_t index = 0;
  raw_predictions[index] = prediction;
  index = (index + 1) % TF_WINDOW;
  
  uint8_t update = 1;
  int16_t count = 0;
  if (prediction == AR_ID_BIKING || prediction == AR_ID_DRIVING) {
    for (uint8_t i = 0; i < TF_WINDOW; ++i) {
      count += (raw_predictions[i] == prediction);
    }
    update = (count > TF_SLOW_THR);
  }
#if !TF_RESTRICTED
  if (prediction == AR_ID_WALKING) {
    /* checks the last TF_WALK_WINDOW samples */
    for (uint8_t i = index + TF_WINDOW - TF_WALK_WINDOW; i < index + TF_WINDOW; ++i) {
      count += (raw_predictions[i % TF_WINDOW] == prediction);
    }
    int16_t walk_threshold = 
      (last_prediction == AR_ID_STATIONARY) ? TF_WALK_LONG_THR : TF_WALK_SHORT_THR;
    update = (count > walk_threshold);
  }
#endif /* TF_RESTRICTED */
  
  if (update) last_prediction = prediction;
  return last_prediction;
}
#endif
/**
  @biref  Consider the previous activity before updating the output.
Use a parameter alpha which allows to manage the impact of the newest activity recognized.
 **/
static const float * exp_average(float * scores, float alpha)
{
  static float last_scores[AI_NETWORK_OUT_1_SIZE] = {0};
  
  for (int i = 0; i < AI_NETWORK_OUT_1_SIZE; ++i) {
    last_scores[i] = (1.0f - alpha) * last_scores[i] + alpha * scores[i];
  }
  return last_scores;
}

/* Exported Functions ---------------------------------------------*/
uint8_t gesture_postProc(float * scores)
{
  uint8_t predict;
  predict = argmax(exp_average(scores, FILT_ALPHA), AI_NETWORK_OUT_1_SIZE);
  //return (fmlp_temporal_filter(predict));
  return(predict);
}


/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/