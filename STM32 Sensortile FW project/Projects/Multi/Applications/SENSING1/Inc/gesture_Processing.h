/**
  ******************************************************************************
  * @file    har_Processing.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   API defined on har_Processing.c file
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
#ifndef _GESTURE_PROCESSING_H_
#define _GESTURE_PROCESSING_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "sensor.h"

/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
 typedef struct
{
  float AccX;           /*  acc x axes [mg]  */
  float AccY;           /*  acc y axes [mg]  */
  float AccZ;           /*  acc z axes [mg]  */
  float GyrX;           /*  gyr x axes [mdps] */
  float GyrY;           /*  gyr y axes [mdps] */
  float GyrZ;           /*  gyr z axes [mdps] */
} Gesture_input_t;  

typedef enum 
{
    Gesture_NOACTIVITY          = 0x00,  
    Gesture_STATIONARY          = 0x01,
    Gesture_WALKING             = 0x02,
    Gesture_FASTWALKING         = 0x03,
    Gesture_JOGGING             = 0x04,
    Gesture_BIKING              = 0x05,
    Gesture_DRIVING             = 0x06,
    Gesture_STAIRS              = 0x07
} Gesture_output_t;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define AR_ID_STILL             (uint8_t)(0x00)
#define AR_ID_ROUND_CCW         (uint8_t)(0x01)
#define AR_ID_ROUND_CW          (uint8_t)(0x02)
#define AR_ID_CROSS_RIGHT       (uint8_t)(0x03)
#define AR_ID_CROSS_LEFT        (uint8_t)(0x04)
#define AR_ID_RIGHT             (uint8_t)(0x05)
#define AR_ID_LEFT              (uint8_t)(0x06)
#define AR_ID_UP                (uint8_t)(0x07)
#define AR_ID_DOWN              (uint8_t)(0x08)



#if (defined(NN_IGN))
#define AR_ID_STATIONARY  (uint8_t)(0x03)
#define AR_ID_WALKING     (uint8_t)(0x04)
#define AR_ID_JOGGING     (uint8_t)(0x02)
#define AR_ID_BIKING      (uint8_t)(0x00)
#define AR_ID_DRIVING     (uint8_t)(0x01)
#elif (defined (NN_IGN_WSDM)) 
#define AR_ID_STATIONARY  (uint8_t)(0x02)
#define AR_ID_WALKING     (uint8_t)(0x03)
#define AR_ID_JOGGING     (uint8_t)(0x00)
#define AR_ID_STAIRS      (uint8_t)(0x01)
#elif (defined (NN_GMP))  
#define AR_ID_STATIONARY  (uint8_t)(0x00)
#define AR_ID_WALKING     (uint8_t)(0x01)
#define AR_ID_JOGGING     (uint8_t)(0x02)
#define AR_ID_BIKING      (uint8_t)(0x03)
#define AR_ID_DRIVING     (uint8_t)(0x04)
#endif
#define AR_ID_NONE        (uint8_t)(0xFF)


# define sequence_length        128

/** @defgroup NN_AR_Exported_Functions NN_AR_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */


/**
 * @brief  Initialize the MotionAR engine
 * @param  None
 * @retval 0 in case of success, a negative code otherwise
 */
int8_t  Gesture_Initialize(void);

/**
 * @brief  deInitialize the MotionAR engine
 * @param  None
 * @retval None
 */
void Gesture_DeInitialize(void);

/**
 * @brief  Run Activity Recognition Algorithm
 * @param  data_in: pointer to the HAR_input_t structure
 * @retval activity index
 */
Gesture_output_t Gesture_run(SensorAxes_t ACC_Value, SensorAxes_t GYR_Value);

/**
 * @brief  get latest activity code computes by Recognition Algorithm
 * @param  None
 * @retval activity index
 */
Gesture_output_t Gesture_get_Activity_Code(void);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t Gesture_GetLibVersion(char *version);


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* HAR_PROCESSING */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
