/**
  ******************************************************************************
  * @file    sensor_service.h 
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Sensors services APIs
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
#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hci_le.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"

#include <stdlib.h>
#include "HWAdvanceFeatures.h"
   
#include "cmsis_os.h"

#include "gesture_Processing.h"
/* Exported Defines --------------------------------------------------------*/

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */

/*************** Don't Change the following defines *************/

/* Define the Max dimesion of the Bluetooth characteristics for each packet  */
#define W2ST_MAX_CHAR_LEN 20

/* Feature mask for hardware events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400

/* Feature mask for Temperature1 */
#define FEATURE_MASK_TEMP1 0x00040000

/* Feature mask for Temperature2 */
#define FEATURE_MASK_TEMP2 0x00010000

/* Feature mask for Pressure */
#define FEATURE_MASK_PRESS 0x00100000

/* Feature mask for Humidity */
#define FEATURE_MASK_HUM   0x00080000

/* Feature mask for Accelerometer */
#define FEATURE_MASK_ACC   0x00800000

/* Feature mask for Gyroscope */
#define FEATURE_MASK_GRYO  0x00400000

/* Feature mask for Magnetometer */
#define FEATURE_MASK_MAG   0x00200000

/* Feature mask for BlueVoice */
#define FEATURE_MASK_BLUEVOICE   0x08000000

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1   )

/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2)

/* Mic */
#define W2ST_CONNECT_AUDIO_LEVEL   (1<<3)

#define W2ST_CONNECT_AR            (1<<4)

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<5)

/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<6)

/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<7)

/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<8)

/* Configuration Feature */
#define W2ST_CONNECT_CONF_EVENT      (1<<9)

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  #define W2ST_CONNECT_SD_CARD_LOGGING   (1<<10)
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

/* Audio Scene Classification Feature */
#define W2ST_CONNECT_ASC_EVENT      (1<<11)


#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

/* Exported Types ------------------------------------------------------- */

typedef enum
{
  SET_CONNECTABLE     = 0x00,
  CONF_NOTIFY         = 0x01,
  PROCESS_EVENT       = 0x02,
  ACC                 = 0x03,
  ACC_STEP            = 0x04,
  ENV                 = 0x05,
  MOTION              = 0x06,
  AUDIO_SC            = 0x07,
  ACTIVITY            = 0x08,
  AUDIO_LEV           = 0x09,
  TERM_STDOUT         = 0x0A,
  TERM_STDERR         = 0x0B,
  BATTERY_INFO        = 0x0C,
  BATTERY_PLUG        = 0x0D,
  SD_CARD_LOGGING     = 0x0E,
  SET_HOST_LINK_TYPE  = 0x0F,
  NUMBER_OF_MSG_TYPE
}msgType_t;

typedef enum
{
  NOT_CONNECTED        = 0x00,
  DEFAULT_HOST_LINK    = 0x01,
  ENV_HOST_LINK        = 0x02,
  AUDIO_HOST_LINK      = 0x03,
  MOTION_HOST_LINK     = 0x04,
  NUMBER_OF_HOST_LINK_TYPE
}hostLinkType_t;

typedef struct
{
  int32_t  press;
  uint16_t hum;
  int16_t  temp2;
  int16_t  temp1;
} envData_t;

typedef struct
{
  SensorAxes_t acc;
  SensorAxes_t gyr;
  SensorAxes_t mag;
} motionData_t;

typedef struct
{
  uint32_t feature;
  uint8_t  command;
  uint8_t  data;
} conf_t;

typedef struct
{
  uint8_t  length;
  uint8_t  data[W2ST_MAX_CHAR_LEN];
} term_data_t;

typedef struct
{
  uint32_t soc;
  uint32_t voltage;
  int32_t  current;
} battery_data_t;

#if defined (__CC_ARM)
  #pragma anon_unions
#endif

typedef struct 
{
  msgType_t type;
  union
  {
    envData_t      env     ;
    conf_t         conf    ; 
    AccEventType   acc     ;
    uint16_t       stepCnt ;
    uint16_t       DBNOISE_Value_Ch[AUDIO_CHANNELS];
    volatile int32_t  audioLoc;
    motionData_t   motion   ;
    uint16_t       angle    ;
    hostLinkType_t HostLinkType ;
    Gesture_output_t   activity;
    ASC_OutputTypeDef audio_scene;
    term_data_t    term;
    battery_data_t batteryInfo;
  };
}msgData_t;

/* Exported Variables ------------------------------------------------------- */
extern uint32_t ConnectionBleStatus;

/* Exported functions ------------------------------------------------------- */
extern tBleStatus Add_HW_SW_ServW2ST_Service(void);
extern tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag);
extern tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte);
extern tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1);

extern tBleStatus AudioLevel_Update(uint16_t *Mic);

#ifdef STM32_SENSORTILE
extern tBleStatus GG_Update(uint32_t soc, uint32_t voltage, int32_t current);
  #ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    extern tBleStatus SDLog_Update(uint8_t ErrorCode);
  #endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
#endif /* STM32_SENSORTILE */

extern tBleStatus ActivityRec_Update(Gesture_output_t ActivityCode);
extern tBleStatus AudioSRec_Update(ASC_OutputTypeDef SceneClassificationCode);

extern tBleStatus Add_ConsoleW2ST_Service(void);
extern tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
extern tBleStatus Term_Update(uint8_t *data,uint8_t length);
extern tBleStatus UpdateTermStdOut(uint8_t *data,uint8_t length);
extern tBleStatus UpdateTermStdErr(uint8_t *data,uint8_t length);

extern tBleStatus Add_ConfigW2ST_Service(void);
extern tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t val);

extern void       setConnectable(void);
extern void       setNotConnectable(void);
extern void       setConnectionParameters(int min , int max, int latency , int timeout );
extern void       HCI_Event_CB(void *pckt);

extern int startProc(msgType_t Type,uint32_t period);
extern int stopProc(msgType_t Type);
extern void startBlinkLed(void);
extern void stopBlinkLed(void);

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
