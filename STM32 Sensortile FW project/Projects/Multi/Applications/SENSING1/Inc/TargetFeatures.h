/**
  ******************************************************************************
  * @file    TargetFeatures.h 
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Specification of the HW Features for each target platform
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
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"

#include "SENSING1_config.h"

#ifdef STM32_NUCLEO
  #include "stm32l4xx_nucleo.h"
  #include "stm32l4xx_nucleo_bluenrg.h"
  #include "stm32l4xx_hal_conf.h"
  #include "stm32l4xx_UART.h"
  #include "stm32l4xx_I2C.h"
  #include "stm32l4xx_SPI.h"
  #include "stm32l4xx_periph_conf.h"
  #include "stm32_bluenrg_ble.h"
  #include "x_nucleo_iks01a2.h"
  #include "x_nucleo_iks01a2_accelero.h"
  #include "x_nucleo_iks01a2_gyro.h"
  #include "x_nucleo_iks01a2_magneto.h"
  #include "x_nucleo_iks01a2_humidity_patch.h"
  #include "x_nucleo_iks01a2_temperature_patch.h"
  #include "x_nucleo_iks01a2_pressure_patch.h"
  #include "x_nucleo_cca02m1_audio_l4.h"
#elif STM32_SENSORTILE
  #include "SensorTile.h"
  #include "stm32l4xx_hal_conf.h"
  #include "stm32l4xx_hal_def.h"
  #include "SensorTile_BlueNRG.h"
  #include "SensorTile_accelero.h"
  #include "SensorTile_gyro.h"
  #include "SensorTile_magneto.h"
  #include "SensorTile_pressure_patch.h"
  #include "SensorTile_temperature_patch.h"
  #include "SensorTile_humidity_patch.h"
  #include "SensorTile_gg.h"
  #include "SensorTile_audio_in.h"
  #include "SensorTile_sd.h"
#endif /* STM32_NUCLEO */

#include "MetaDataManager.h"

#include "har_Processing.h"
#include "asc.h"

/* Exported defines ------------------------------------------------------- */
#define MAX_TEMP_SENSORS 2
/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    (0.001F)
/* @brief  Scale factor. It is used to scale acceleration from g to m/s2 */ 
#define FROM_G_TO_MS_2   (9.800655F)


/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */
/**
 * @brief  Target type data definition
 */
typedef enum
{
  TARGET_NUCLEO,
  TARGET_BLUECOIN,
  TARGET_SENSORTILE,
  TARGETS_NUMBER
} TargetType_t;

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  TargetType_t BoardType;
  int32_t NumTempSensors;

  void *HandleTempSensors[MAX_TEMP_SENSORS];
  void *HandlePressSensor;
  void *HandleHumSensor;

  void *HandleAccSensor;
  void *HandleGyroSensor;
  void *HandleMagSensor;

  float DefaultAccODR;
  float DefaultGyroODR;
  float DefaultMagODR;

  float AccSensiMultInG;

  int32_t NumMicSensors;

#ifdef STM32_SENSORTILE
  void *HandleGGComponent;
#endif /* STM32_SENSORTILE */

  uint32_t AudioVolume;
  uint8_t EnvSensorEnabled;
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(TargetType_t BoardType);

extern void InitMics(uint32_t AudioFreq);
extern void DeInitMics(void);

extern void InitUSBAudio(void);
extern void DeInitUSBAudio(void);

extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);

extern void enableEnvSensors (void);
extern void disableEnvSensors (void);
extern void enableMotionSensors (void);
extern void disableMotionSensors (void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

