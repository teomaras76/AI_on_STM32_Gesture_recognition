/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Add 4 bluetooth services using vendor specific profiles.
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"
#include "console.h"
#include "HWAdvanceFeatures.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "PowerControl.h"

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
#include "DataLog_Manager.h"
#include <math.h>
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

//#include "OTA.h"

/* Exported variables ---------------------------------------------------------*/
uint8_t set_connectable = TRUE;

uint32_t ConnectionBleStatus  =0;

/* Imported Variables -------------------------------------------------------------*/

extern TIM_HandleTypeDef    TimAudioDataHandle;

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];

extern uint8_t bdaddr[6];

extern uint8_t NodeName[8];


/* Private variables ------------------------------------------------------------*/
static uint32_t FeatureMask;
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;
static uint16_t AudioLevelCharHandle;

#ifdef STM32_SENSORTILE
static uint16_t BatteryFeaturesCharHandle;
  #ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    static uint16_t SDLogFeaturesCharHandle;
  #endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
#endif /* STM32_SENSORTILE */

#if NN_HAR
static uint16_t ActivityRecCharHandle;
#endif /* NN_HAR */

static uint16_t AudioSRecCharHandle;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize = 2; /* Size for Environmental BLE characteristic */

static uint16_t connection_handle = 0;

static uint32_t SizeOfUpdateBlueFW=0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

static void Read_Request_CB(uint16_t handle);
static void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle,
                      uint16_t charHandle,
                      uint8_t charValOffset,
                      uint8_t charValueLen,
                      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;

  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);

    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }

  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //SENSING1_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
     goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //SENSING1_PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR;
}


tBleStatus UpdateTermStdOut(uint8_t *data,uint8_t length)
{
  if (aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, length , data) != BLE_STATUS_SUCCESS) {
      SENSING1_PRINTF("Error Updating Stdout Char\r\n");
    return BLE_STATUS_ERROR;
  }
  osDelay(20);
  return BLE_STATUS_SUCCESS;
}

tBleStatus UpdateTermStdErr(uint8_t *data,uint8_t length)
{
  if (aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, length , data) != BLE_STATUS_SUCCESS) {
      SENSING1_PRINTF("Error Updating Stdout Char\r\n");
    return BLE_STATUS_ERROR;
  }
  osDelay(20);
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  uint8_t Offset;
  uint8_t DataToSend;
  msgData_t msg;
  msg.type        = TERM_STDERR;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_MAX_CHAR_LEN) ?  W2ST_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    msg.term.length   =  DataToSend;
    memcpy(msg.term.data,data+Offset,DataToSend);
    SendMsgToHost(&msg);

  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  uint8_t   Offset;
  uint8_t   DataToSend;
  msgData_t msg;
  msg.type = TERM_STDOUT;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_MAX_CHAR_LEN) ?  W2ST_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    msg.term.length   =  DataToSend;
    memcpy(msg.term.data,data+Offset,DataToSend);
    SendMsgToHost(&msg);
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  msgData_t msg;
  msg.type        = TERM_STDERR;
  msg.term.length = LastStderrLen;
  memcpy(msg.term.data,LastStderrBuffer,LastStderrLen);
  SendMsgToHost(&msg);
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  msgData_t msg;
  msg.type        = TERM_STDOUT;
  msg.term.length = LastTermLen;
  memcpy(msg.term.data,LastTermBuffer,LastTermLen);
  SendMsgToHost(&msg);
  return BLE_STATUS_SUCCESS;
}
#if (NN_HAR)
/**
 * @brief  Update Activity Recognition value
 * @param  HAR_output_t ActivityCode Activity Recognized
 * @retval tBleStatus      Status
 */
tBleStatus ActivityRec_Update(Gesture_output_t ActivityCode)
{
  tBleStatus ret;
  uint8_t buff[2+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = ActivityCode;
  buff[3] = HAR_ALG_ID ;

  ret = aci_gatt_update_char_value(HWServW2STHandle, ActivityRecCharHandle, 0, 2+1+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating ActivityRec Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating ActivityRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif
/**
 * @brief  Update Audio Scene Recognition value
 * @param  ASC_OutputTypeDef SceneClassificationCode Scene Recognized
 * @retval tBleStatus      Status
 */
tBleStatus AudioSRec_Update(ASC_OutputTypeDef SceneClassificationCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff, (HAL_GetTick() >> 3));
  buff[2] = SceneClassificationCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, AudioSRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AudioRec Char\n");
      Stderr_Update(BufferToWrite, BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating AudioRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  msgData_t msg;
  msg.type         = CONF_NOTIFY;
  msg.conf.feature = Feature;
  msg.conf.command = Command;
  msg.conf.data    = data;
  SendMsgToHost(&msg);
  return 0;  //!!!
}

tBleStatus Config_NotifyBLE(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte)
{
  tBleStatus ret= BLE_STATUS_SUCCESS;
  uint8_t buff[2+3];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));

  if(dimByte==3) {
    buff[2]= 0;
    STORE_LE_16(buff+3,Command);
  } else {
    STORE_LE_16(buff+2,Command);
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+dimByte,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AccEvent_Notify Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating AccEvent_Notify Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  /* default characteristics :
  1- HW_SENS_W2ST_SERVICE_UUID
  2- ENVIRONMENTAL_W2ST_CHAR_UUID
  3- ACC_GYRO_MAG_W2ST_CHAR_UUID
  4- ACC_EVENT_W2ST_CHAR_UUID
  5- MIC_W2ST_CHAR_UUID
 */
  uint8_t max_attr_records = 5;

#if ((NN_HAR) || defined (NN_ASC))
  max_attr_records++;
#endif

#ifdef STM32_SENSORTILE
  if(TargetBoardFeatures.HandleGGComponent) {
    /* Battery Present */
    max_attr_records++;
  }

  /* SD Log Feature */
  #ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    max_attr_records++;
  #endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
#endif /* STM32_SENSORTILE */

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*max_attr_records,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  if(TargetBoardFeatures.NumTempSensors==2) {
    uuid[14] |= 0x05; /* Two Temperature values*/
    EnvironmentalCharSize+=2*2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    uuid[14] |= 0x04; /* One Temperature value*/
    EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
   uuid[14] |= 0x08; /* Humidity */
   EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    uuid[14] |= 0x10; /* Pressure value*/
    EnvironmentalCharSize+=4;
  }

  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3, //2+2,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &AccEventCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_MIC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid,2+AUDIO_CHANNELS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioLevelCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

#if (NN_HAR)
  COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1+1, /* 2 byte timestamp, 1 byte action, 1 byte algorithm */
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ActivityRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#elif defined (NN_ASC)
  COPY_AUDIO_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1, /* 2 byte timestamp, 1 byte aucoustic scene classification */
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioSRecCharHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif

#ifdef STM32_SENSORTILE
  if(TargetBoardFeatures.HandleGGComponent) {
    COPY_GG_W2ST_CHAR_UUID(uuid);
    ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2+2+2+1,
                             CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                             ATTR_PERMISSION_NONE,
                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                             16, 0, &BatteryFeaturesCharHandle);

    if (ret != BLE_STATUS_SUCCESS) {
      goto fail;
    }
  }

  #ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    COPY_SDLOG_W2ST_CHAR_UUID(uuid);
    ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &SDLogFeaturesCharHandle);

    if (ret != BLE_STATUS_SUCCESS) {
      goto fail;
    }
  #endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
#endif /* STM32_SENSORTILE */


  return BLE_STATUS_SUCCESS;

fail:
  //SENSING1_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag)
{
  tBleStatus ret;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));

  STORE_LE_16(buff+2 ,Acc->AXIS_X);
  STORE_LE_16(buff+4 ,Acc->AXIS_Y);
  STORE_LE_16(buff+6 ,Acc->AXIS_Z);

  Gyro->AXIS_X/=100;
  Gyro->AXIS_Y/=100;
  Gyro->AXIS_Z/=100;

  STORE_LE_16(buff+8 ,Gyro->AXIS_X);
  STORE_LE_16(buff+10,Gyro->AXIS_Y);
  STORE_LE_16(buff+12,Gyro->AXIS_Z);

  STORE_LE_16(buff+14, Mag->AXIS_X);
  STORE_LE_16(buff+16, Mag->AXIS_Y);
  STORE_LE_16(buff+18, Mag->AXIS_Z);

  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
  tBleStatus ret;
  uint8_t BuffPos;

  uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp2*/+2/*Temp1*/];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;

  if(TargetBoardFeatures.HandlePressSensor) {
    STORE_LE_32(buff+BuffPos,Press);
    BuffPos+=4;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    STORE_LE_16(buff+BuffPos,Hum);
    BuffPos+=2;
  }

  if(TargetBoardFeatures.NumTempSensors==2) {
    STORE_LE_16(buff+BuffPos,Temp2);
    BuffPos+=2;

    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating Environmental Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Microphones characteristic values
 * @param  uint16_t *Mic SNR dB Microphones array
 * @retval tBleStatus   Status
 */
tBleStatus AudioLevel_Update(uint16_t *Mic)
{
  tBleStatus ret;
  uint16_t Counter;

  uint8_t buff[2+1*AUDIO_CHANNELS]; /* BlueCoin has 4 Mics */

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  for(Counter=0;Counter<AUDIO_CHANNELS;Counter++) {
    buff[2+Counter]= Mic[Counter]&0xFF;
  }

  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AudioLevelCharHandle, 0, 2+AUDIO_CHANNELS,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Mic Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

#ifdef STM32_SENSORTILE
/**
 * @brief  Update Gas Gouge characteristic
 * @param  None
 * @retval tBleStatus   Status
 */
tBleStatus GG_Update(uint32_t soc, uint32_t voltage, int32_t current)
{
  tBleStatus ret;

  uint8_t buff[2+2+2+2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,soc*10);
  STORE_LE_16(buff+4,voltage);
  STORE_LE_16(buff+6,current);

  if(soc<15) {
    /* if it's < 15% Low Battery*/
    buff[8] = 0x00; /* Low Battery */
  } else {
    static uint32_t Status     = 0x04; /* Unknown */
    if(current <= 0) {
      Status = 0x01; /* Discharging */
    } else {
      Status = 0x03; /* Charging */
    }
    buff[8] = Status;
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, BatteryFeaturesCharHandle, 0, 2+2+2+2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating GG Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating GG Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

  #ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    /**
     * @brief  Update SD Log characteristic
     * @param  None
     * @retval tBleStatus   Status
     */
    tBleStatus SDLog_Update(uint8_t ErrorCode)
    {
      tBleStatus ret;
      uint8_t buff[2+1];

      STORE_LE_16(buff  ,(HAL_GetTick()>>3));
      buff[2] = ErrorCode;

      ret = aci_gatt_update_char_value(HWServW2STHandle, SDLogFeaturesCharHandle, 0, 3,buff);

      if (ret != BLE_STATUS_SUCCESS){
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
          BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating SD Log Char\n");
          Stderr_Update(BufferToWrite,BytesToWrite);
        } else {
          SENSING1_PRINTF("Error Updating SD Log Char\r\n");
        }
        return BLE_STATUS_ERROR;
      }
      return BLE_STATUS_SUCCESS;
    }
  #endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

#endif /* STM32_SENSORTILE */

/**
 * @brief  Puts the device in connectable mode.
 * @param  None
 * @retval None
 */
void setConnectable(void)
{
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7]};
  uint8_t manuf_data[26] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7], // Complete Name
    13,0xFF,0x01/*SKD version */,
#ifdef  STM32_NUCLEO
    0x80,
#elif STM32_SENSORTILE
    0x02,
#endif /* STM32_NUCLEO */
    0x00 /* AudioSync+AudioData */,
    0xE0 /* ACC+Gyro+Mag*/,
    0x00 /*  */,
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };

  /* BLE MAC */
  manuf_data[20] = bdaddr[5];
  manuf_data[21] = bdaddr[4];
  manuf_data[22] = bdaddr[3];
  manuf_data[23] = bdaddr[2];
  manuf_data[24] = bdaddr[1];
  manuf_data[25] = bdaddr[0];

#ifdef STM32_SENSORTILE
  if(TargetBoardFeatures.HandleGGComponent){
    manuf_data[17] |= 0x02; /* Battery Present */
  }
#endif /* STM32_SENSORTILE */

  manuf_data[16] |= 0x04; /* Mic */

  if(TargetBoardFeatures.NumTempSensors==2) {
    manuf_data[17] |= 0x05; /* Two Temperature values*/
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    manuf_data[17] |= 0x04; /* One Temperature value*/
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    manuf_data[17] |= 0x08; /* Humidity */
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    manuf_data[17] |= 0x10; /* Pressure value*/
  }

  /* Accelerometer Events */
  manuf_data[18] |=0x04;

#if (NN_HAR)
    manuf_data[19] |= 0x10;
#endif /* NN_HAR */

   Enable_SPI_IRQ();
  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
#ifndef BLE_CHANGE_ADV_INTERVAL
  aci_gap_set_discoverable(ADV_IND, 0, 0,
#else /* BLE_CHANGE_ADV_INTERVAL */
  aci_gap_set_discoverable(ADV_IND, 0x0640, 0x0640,
// 0x800 default value - 1.28 s
// 0x640  - 1.00 s
#endif /* BLE_CHANGE_ADV_INTERVAL */
#ifndef MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
                           STATIC_RANDOM_ADDR,
  #else /* MAC_STM32UID_BLUEMS */
                           RANDOM_ADDR,
  #endif /* MAC_STM32UID_BLUEMS */
#else /* MAC_BLUEMS */
                           PUBLIC_ADDR,
#endif /* MAC_BLUEMS */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(26, manuf_data);
}
/**
 * @brief  Exits the device from connectable mode.
 * @param  None
 * @retval None
 */
void setNotConnectable(void)
{
  aci_gap_set_non_discoverable();
  Disable_SPI_IRQ();
}
#ifdef BLE_LINK_ADAPT
void setConnectionParameters(int min , int max, int latency , int timeout )
{
  int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
                                              min /* interval_min*/,
                                              max /* interval_max */,
                                              latency /* slave_latency */,
                                              timeout /*timeout_multiplier*/);
  if (ret != BLE_STATUS_SUCCESS) {
    while (1) {
      ;
    }
  }
}
#endif

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connection_handle = handle;

  SENSING1_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);

  ConnectionBleStatus=0;
  DisableHWFeatures();
  LedBlinkStop();

  /* Force one UUID rescan for FOTA */
  {
    tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
    uint32_t charValue = 0x0000FFFF; /* Delete all the Handles from 0x0000 to 0xFFFF */
    ret = aci_gatt_update_char_value(0x0001,0x0002,0,4,(uint8_t*) &charValue);

    if (ret == BLE_STATUS_SUCCESS){
      SENSING1_PRINTF("UUID Rescan Forced\r\n");
    } else {
      SENSING1_PRINTF("Problem forcing UUID Rescan\r\n");
    }
  }
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  SENSING1_PRINTF("<<<<<<DISCONNECTED\r\n");

  /* Make the device connectable again. */
  set_connectable = TRUE;
  ConnectionBleStatus=0;

  DisableHWFeatures();

  /* Reset for any problem during FOTA update */
  SizeOfUpdateBlueFW = 0;

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  /* Close the Log if the user exit in a dirty way */

  if(SD_LogMems_Enabled) {
    SD_Card_FeaturesMask &= ~(FEATURE_MASK_TEMP1 |
                              FEATURE_MASK_TEMP2|
                              FEATURE_MASK_PRESS |
                              FEATURE_MASK_HUM |
                              FEATURE_MASK_ACC |
                              FEATURE_MASK_GRYO |
                              FEATURE_MASK_MAG);
    SD_CardLoggingMemsStop();
  }

  if(SD_LogAudio_Enabled) {
    /* For waiting the close of the MEMS/Annotation file */
    osDelay(100);

    SD_Card_FeaturesMask &= ~FEATURE_MASK_BLUEVOICE;
    SD_CardLoggingAudioStop();
  }
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

  /************************/
  /* Stops all the Timers */
  /************************/
  /* Stop Timer For MotionAR */
   stopProc(ACTIVITY);

  /* Stop Timer For Acc/Gyro/Mag */
  stopProc(MOTION);

  /* Stop Timer For Environmental */
  stopProc(ENV);

  /* Stop Timer For Audio Level*/
  stopProc(AUDIO_LEV);
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  uint8_t Status;
  if(handle == EnvironmentalCharHandle + 1){
    /* Read Request for Pressure,Humidity, and Temperatures*/
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.HandlePressSensor) {
      if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
        /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
        BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
#ifdef ONE_SHOT
        BSP_PRESSURE_Set_One_Shot(TargetBoardFeatures.HandlePressSensor);
#endif
        MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
        PressToSend=intPart*100+decPart;
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
        BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
#ifdef ONE_SHOT
        BSP_HUMIDITY_Set_One_Shot(TargetBoardFeatures.HandleHumSensor);
#endif
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        HumToSend = intPart*10+decPart;
      }
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
#ifdef ONE_SHOT
        BSP_TEMPERATURE_Set_One_Shot(TargetBoardFeatures.HandleTempSensors[0]);
#endif
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
      }

      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
        /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
#ifdef ONE_SHOT
        BSP_TEMPERATURE_Set_One_Shot(TargetBoardFeatures.HandleTempSensors[1]);
#endif
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart;
      }
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
#ifdef ONE_SHOT
        BSP_TEMPERATURE_Set_One_Shot(TargetBoardFeatures.HandleTempSensors[0]);
#endif
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
      }
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  } else if(handle == AccEventCharHandle +1) {
    {
      uint16_t StepCount;
      if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
        StepCount = GetStepHWPedometer();
      } else {
        StepCount = 0;
      }
      AccEvent_Notify(StepCount, 2);
    }
  } else if (handle == StdErrCharHandle + 1) {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
#if (NN_HAR)
  } else if (handle == ActivityRecCharHandle + 1) {
     ActivityRec_Update(Gesture_get_Activity_Code());
#elif defined(NN_ASC)
  } else if (handle == AudioSRecCharHandle + 1) {
    AudioSRec_Update(ASC_GetClassificationCode());
#endif /* NN_HAR*/
#ifdef STM32_SENSORTILE
  } else if(handle == BatteryFeaturesCharHandle + 1){

    uint32_t voltage, soc;
    int32_t current;
    uint8_t v_mode;

    /* Update Gas Gouge Status */
    BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

    /* Read the Gas Gouge Status */
    BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
    BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);
    BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);

    GG_Update(soc, voltage, current);
#endif /* STM32_SENSORTILE */
  }

  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
#if NN_HAR
  if (attr_handle == ActivityRecCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_AR);
      startProc(ACTIVITY,INERTIAL_ACQ_ACTIVITY_MS);
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_AR);
      stopProc(ACTIVITY);
    }
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->ActRec=%s\n",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? " ON" : " OFF");
      Term_Update(BufferToWrite,BytesToWrite);
    } else
      SENSING1_PRINTF("--->ActRec=%s\r\n",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? " ON" : " OFF");
#endif /* SENSING1_DEBUG_CONNECTION */
  }
  else
#endif /* NN_HAR */
  if (attr_handle == AudioSRecCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ASC_EVENT);
      PowerCtrlLock();
#ifdef ASC_USE_USB_AUDIO
      InitUSBAudio();
#else
      InitMics(AUDIO_SAMPLING_FREQUENCY);
#endif /* ASC_USE_USB_AUDIO */
      startProc(AUDIO_SC, 0 /* Not Used for Audio */);
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ASC_EVENT);
#ifdef ASC_USE_USB_AUDIO
      DeInitUSBAudio();
#else
      DeInitMics();
#endif /* ASC_USE_USB_AUDIO */
      PowerCtrlUnLock();
      stopProc(AUDIO_SC);
    }
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->AudioSRec=%s\n",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_ASC_EVENT) ? " ON" : " OFF");
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("--->AudioSRec=%s\r\n",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_ASC_EVENT) ? " ON" : " OFF");
    }
#endif /* SENSING1_DEBUG_CONNECTION */
#ifdef STM32_SENSORTILE
  } else if(attr_handle == BatteryFeaturesCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_GG_EVENT);
      /* Start the TIM Base generation in interrupt mode */
      startProc(BATTERY_INFO,ENV_UPDATE_MS);
    }else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_GG_EVENT);
      /* Stop the TIM Base generation in interrupt mode */
      stopProc(BATTERY_INFO);
   }
#ifdef SENSING1_DEBUG_CONNECTION
   if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite =sprintf((char *)BufferToWrite,"--->GG=%s\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT) ? " ON" : " OFF");
     Term_Update(BufferToWrite,BytesToWrite);
   } else {
     SENSING1_PRINTF("--->GG=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT) ? " ON\r\n" : " OFF\r\n");
   }
#endif /* SENSING1_DEBUG_CONNECTION */
#endif /* STM32_SENSORTILE */
   } else if(attr_handle == ConfigCharHandle + 2){
      if (att_data[0] == 01) {
        W2ST_ON_CONNECTION(W2ST_CONNECT_CONF_EVENT);
      } else if (att_data[0] == 0){
        W2ST_OFF_CONNECTION(W2ST_CONNECT_CONF_EVENT);
      }
#ifdef SENSING1_DEBUG_CONNECTION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite =sprintf((char *)BufferToWrite,"--->Conf=%s\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT) ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
      } else {
        SENSING1_PRINTF("--->Conf=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT) ? "ON" : "OFF");
      }
#endif /* SENSING1_DEBUG_CONNECTION */
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    } else if(attr_handle == SDLogFeaturesCharHandle + 2){
      if (att_data[0] == 01) {
        W2ST_ON_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING);
      } else if (att_data[0] == 0){
        W2ST_OFF_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING);
      }
#ifdef SENSING1_DEBUG_CONNECTION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite =sprintf((char *)BufferToWrite,"--->SDLog=%s\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING) ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
      } else {
        SENSING1_PRINTF("--->SDLog=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING) ? "ON" : "OFF");
      }
#endif /* SENSING1_DEBUG_CONNECTION */
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
  } else if(attr_handle == AccGyroMagCharHandle + 2) {
     if (att_data[0] == 01) {
       W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);
       startProc(MOTION, INERTIAL_UPDATE_MS );
    } else if (att_data[0] == 0) {
       W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);
       stopProc(MOTION);
    }
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Acc/Gyro/Mag=%s\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON" : " OFF");
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("--->Acc/Gyro/Mag=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON" : " OFF");
    }
#endif /* SENSING1_DEBUG_CONNECTION */
  } else if(attr_handle == AccEventCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_EVENT);
      EnableHWMultipleEvents();
      ResetHWPedometer();
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
        Config_Notify(FEATURE_MASK_ACC_EVENTS,'m',1);
      }
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_EVENT);
      DisableHWMultipleEvents();
      //DisableHWFeatures();
    }
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->AccEvent=%s\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? " ON" : " OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("--->AccEvent=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? " ON" : " OFF");
    }
#endif /* SENSING1_DEBUG_CONNECTION */
  } else if(attr_handle == EnvironmentalCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

      /* Start the TIM Base generation in interrupt mode */
      startProc(ENV,ENV_UPDATE_MS);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

      /* Stop the TIM Base generation in interrupt mode */
      stopProc(ENV);
    }
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? " ON" : " OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? " ON" : " OFF");
    }
#endif /* SENSING1_DEBUG_CONNECTION */
  } else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
    }
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
    }
  } else if (attr_handle == TermCharHandle + 1){
    uint32_t SendBackData =1; /* By default Answer with the same message received */
    if(SizeOfUpdateBlueFW!=0) {
      /* FP-AI-SENSING1 firwmare update */
      //int8_t RetValue = UpdateFWBlueMS(&SizeOfUpdateBlueFW,att_data, data_length,1);
//      if(RetValue!=0) {
//        MCR_FAST_TERM_UPDATE_FOR_OTA(((uint8_t *)&RetValue));
//        if(RetValue==1) {
//          /* if OTA checked */
//          BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
//          Term_Update(BufferToWrite,BytesToWrite);
//          SENSING1_PRINTF("%s will restart in 5 seconds\r\n",SENSING1_PACKAGENAME);
//          HAL_Delay(5000);
//          HAL_NVIC_SystemReset();
//        }
//      }
      SendBackData=0;
    } else {
      /* Received one write from Client on Terminal characteristc */
      SendBackData = DebugConsoleCommandParsing(att_data,data_length);
    }

    /* Send it back for testing */
    if(SendBackData) {
      Term_Update(att_data,data_length);
    }
  } else if (attr_handle == AudioLevelCharHandle + 2) {
    if (att_data[0] == 01) {
      int32_t Count;

      W2ST_ON_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL);

      InitMics(AUDIO_SAMPLING_FREQUENCY);

      for(Count=0;Count<TargetBoardFeatures.NumMicSensors;Count++) {
        RMS_Ch[Count]=0;
        DBNOISE_Value_Old_Ch[Count] =0;
      }
      startProc(AUDIO_LEV,MICS_DB_UPDATE_MS);

    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL);

      DeInitMics();
      stopProc(AUDIO_LEV);
    }
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->dB Noise AudioLevel=%s\n", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)   ? " ON" : " OFF") );
      Term_Update(BufferToWrite,BytesToWrite);
    }else {
      SENSING1_PRINTF("--->dB Noise AudioLevel=%s\r\n", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)   ? " ON" : " OFF"));
    }
#endif /* SENSING1_DEBUG_CONNECTION */
  } else if (attr_handle == ConfigCharHandle + 1) {
    /* Received one write command from Client on Configuration characteristc */
    ConfigCommandParsing(att_data, data_length);
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  } else if (attr_handle == SDLogFeaturesCharHandle + 1) {
    /* Received one write command from Client on SD Log characteristc */
    switch(att_data[0]) {
      case SD_CARD_LOGGING_STOP:
        /* Stop Log Features */
        if(SD_Card_FeaturesMask & (FEATURE_MASK_TEMP1 |
                                   FEATURE_MASK_TEMP2|
                                   FEATURE_MASK_PRESS |
                                   FEATURE_MASK_HUM |
                                   FEATURE_MASK_ACC |
                                   FEATURE_MASK_GRYO |
                                   FEATURE_MASK_MAG)) {

          SD_Card_FeaturesMask &= ~(FEATURE_MASK_TEMP1 |
                                   FEATURE_MASK_TEMP2|
                                   FEATURE_MASK_PRESS |
                                   FEATURE_MASK_HUM |
                                   FEATURE_MASK_ACC |
                                   FEATURE_MASK_GRYO |
                                   FEATURE_MASK_MAG);
          SD_CardLoggingMemsStop();
       } else if(SD_Card_FeaturesMask == FEATURE_MASK_BLUEVOICE) {
          SD_CardLoggingMemsStop();
       }

        if(SD_Card_FeaturesMask & FEATURE_MASK_BLUEVOICE) {
          /* For waiting the close of the MEMS/Annotation file */
          osDelay(100);

          SD_Card_FeaturesMask &= ~FEATURE_MASK_BLUEVOICE;
          SD_CardLoggingAudioStop();
        }

        /* Send back the message */
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
          SDLog_Update(SD_CARD_LOGGING_STOP);
        }
      break;
      case SD_CARD_LOGGING_START:
        /* Start Log Features */

        /* Read the Features Mask */
        {
          uint8_t *p8_Feature = (uint8_t *) &SD_Card_FeaturesMask;
          p8_Feature[0] = att_data[1];
          p8_Feature[1] = att_data[2];
          p8_Feature[2] = att_data[3];
          p8_Feature[3] = att_data[4];
#if 0
          /*Just for Debug */
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            BytesToWrite =sprintf((char *)BufferToWrite, "FeatureMask=%X\n",SD_Card_FeaturesMask);
            Term_Update(BufferToWrite,BytesToWrite);
          }
#endif
        }

        /* Read the Sample rate for Inertial/Environmental Features*/
        {
          uint16_t SampleRateEnvFeatures;
          uint8_t *p8_SampleRateIneFeatures = (uint8_t *) &SampleRateIneFeatures;
          uint8_t *p8_SampleRateEnvFeatures = (uint8_t *) &SampleRateEnvFeatures;

          p8_SampleRateEnvFeatures[0] = att_data[5];
          p8_SampleRateEnvFeatures[1] = att_data[6];

          p8_SampleRateIneFeatures[0] = att_data[7];
          p8_SampleRateIneFeatures[1] = att_data[8];

          /* The Desired Sample Rate is Hz*10 */
          /* We have the thread for Saving the data that could be waked up every multiple of mSec */
          RoundedInertialWakeUpTimer = (int32_t) round(1000.0/(SampleRateIneFeatures/10));

          RoundCounterEnvironmental  = (int32_t) round((10000.0/RoundedInertialWakeUpTimer)/SampleRateEnvFeatures);
          RoundedEnvironmentalFreq   = (int32_t) round(10000.0/(RoundCounterEnvironmental*RoundedInertialWakeUpTimer));
        }

        /* Read the Microphone Volume Level */
        {
          int32_t AudioVolume = *((int8_t*)(att_data+9));

          if((AudioVolume<0) & (AudioVolume>64)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"setMicVol Not Correct\n");
            Stderr_Update(BufferToWrite,BytesToWrite);
          } else {
            TargetBoardFeatures.AudioVolume = AudioVolume;
            BytesToWrite =sprintf((char *)BufferToWrite,"setMicVol Correct =%ld\n",
                                TargetBoardFeatures.AudioVolume);
            Term_Update(BufferToWrite,BytesToWrite);
          }
        }

        /* Read the Data File Name if it's present */
        if(data_length>10) {
          uint8_t Count;
          for(Count=0;Count<(data_length-10);Count++) {
            DefaultDataFileName[Count]=att_data[Count+9];
          }
          /* Termination String */
          DefaultDataFileName[Count] ='\0';
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            BytesToWrite =sprintf((char *)BufferToWrite, "FileName %s\n",DefaultDataFileName);
            Term_Update(BufferToWrite,BytesToWrite);
          }
        }

        /* Start Log Inertial/Enviromental Feature */
        if(((SD_Card_FeaturesMask&FEATURE_MASK_TEMP1 )!=0) |
           ((SD_Card_FeaturesMask&FEATURE_MASK_TEMP2)!=0) |
           ((SD_Card_FeaturesMask&FEATURE_MASK_PRESS)!=0) |
           ((SD_Card_FeaturesMask&FEATURE_MASK_HUM  )!=0) |
           ((SD_Card_FeaturesMask&FEATURE_MASK_ACC  )!=0) |
           ((SD_Card_FeaturesMask&FEATURE_MASK_GRYO )!=0) |
           ((SD_Card_FeaturesMask&FEATURE_MASK_MAG  )!=0)) {
          SD_CardLoggingMemsStart(0);
         } else if (SD_Card_FeaturesMask==FEATURE_MASK_BLUEVOICE) {
           SD_CardLoggingMemsStart(1);
         }

        /* Start Log Inertial/Enviromental Feature */
        if(SD_Card_FeaturesMask&FEATURE_MASK_BLUEVOICE) {
          osDelay(100);
          SD_CardLoggingAudioStart();
        }

        /* Send back the message */
        if(SD_LogAudio_Enabled | SD_LogMems_Enabled) {
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
            SDLog_Update(SD_CARD_LOGGING_START);
          }
        }
      break;
      case SD_CARD_LOGGING_UPDATE:
        /* Update Annotation */
        SaveDataAnnotation(att_data+1);
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
          BytesToWrite =sprintf((char *)BufferToWrite, "Ann->[%s]\n",att_data+1);
          Term_Update(BufferToWrite,BytesToWrite);
        }
      break;
      default:
        SENSING1_PRINTF("SD Log Feature Error First Byte=%d\n",att_data[0]);
    }
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
  } else {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOWN handle\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Notification UNKNOWN handle\r\n");
    }
  }
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

    /* Help Command */
    if(!strncmp("help",(char *)(att_data),4)) {
      /* Print Legend */
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,
         "\ninfo\n"
         "setName xxxxxxx (7 Chars Max)\nversionFw\n"
#ifdef STM32_SENSORTILE
         "powerstatus\n"
#endif /* STM32_SENSORTILE */
           );
      Term_Update(BufferToWrite,BytesToWrite);

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
      BytesToWrite =sprintf((char *)BufferToWrite,
         "setDate wd/dd/mm/aa\n"
         "setTime hh:mm:ss\n"
         "setMicVol Val (0-64)\n");
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,
           "MemsStart/MemsStop\n"
           "AudioStart/AudioStop\n");
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,
           "AudioMemsStart/AudioMemsStop\n");
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,
           "SDName XXXXXX (10 Chars Max)\n");
      Term_Update(BufferToWrite,BytesToWrite);
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

    } else if(!strncmp("versionFw",(char *)(att_data),9)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
#ifdef STM32L476xx
                            "L476",
#else
#error "Undefined STM32 processor type"
#endif
                            SENSING1_PACKAGENAME,
                            SENSING1_VERSION_MAJOR,
                            SENSING1_VERSION_MINOR,
                            SENSING1_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
#ifdef STM32_SENSORTILE
    } else if(!strncmp("powerstatus",(char *)(att_data),11)) {
      SendBackData=0;
      if(TargetBoardFeatures.HandleGGComponent) {
        uint32_t voltage, soc;
        uint8_t v_mode;
        int32_t current;
        /* Update Gas Gouge Status */
        BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

        /* Read the Gas Gouge Status */
        BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
        BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);
        BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);

        BytesToWrite =sprintf((char *)BufferToWrite,"Battery %ld%% %ld mV Current=%ld mA\n",soc,voltage,current);
      } else {
        BytesToWrite =sprintf((char *)BufferToWrite,"Battery not present\n");
      }
      Term_Update(BufferToWrite,BytesToWrite);
#endif /* STM32_SENSORTILE */
    } else if(!strncmp("info",(char *)(att_data),4)) {
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\n"
         "\tVersion %c.%c.%c\n"
#ifdef STM32_SENSORTILE
        "\tSTM32476RG-SensorTile board"
#elif STM32_NUCLEO
        "\tSTM32L476RG-Nucleo board"
#endif /* STM32_SENSORTILE */
          "\n",
          SENSING1_PACKAGENAME,
          SENSING1_VERSION_MAJOR,SENSING1_VERSION_MINOR,SENSING1_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\n",
#elif defined (__CC_ARM)
        " (KEIL)\n",
#elif defined (__GNUC__)
        " (openstm32)\n",
#endif
          HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
           __DATE__,__TIME__);
      Term_Update(BufferToWrite,BytesToWrite);

#if   defined(NN_GMP)
      BytesToWrite =sprintf((char *)BufferToWrite,"\tEnabled AR NN_GMP\n");
#elif defined(NN_IGN)
      BytesToWrite =sprintf((char *)BufferToWrite,"\tEnabled AR NN_IGN\n");
#elif defined(NN_IGN_WSDM)
      BytesToWrite =sprintf((char *)BufferToWrite,"\tEnabled AR NN_IGN_WSDM\n");
#elif defined(NN_ASC)
      BytesToWrite =sprintf((char *)BufferToWrite,"\tEnabled ASC\n");
#endif /* defined(NN_GMP) */
      Term_Update(BufferToWrite,BytesToWrite);

    } else if(!strncmp("upgradeFw",(char *)(att_data),9)) {
      uint32_t uwCRCValue;
      uint8_t *PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;

      SizeOfUpdateBlueFW=atoi((char *)(att_data+9));
      PointerByte[0]=att_data[ 9];
      PointerByte[1]=att_data[10];
      PointerByte[2]=att_data[11];
      PointerByte[3]=att_data[12];

      /* Check the Maximum Possible OTA size */
//      if(SizeOfUpdateBlueFW>OTA_MAX_PROG_SIZE) {
//        SENSING1_PRINTF("OTA %s SIZE=%ld > %d Max Allowed\r\n",SENSING1_PACKAGENAME,SizeOfUpdateBlueFW, OTA_MAX_PROG_SIZE);
//        /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
//        PointerByte[0]= att_data[13];
//        PointerByte[1]=(att_data[14]!=0) ? 0 : 1;/* In order to be sure to have a wrong CRC */
//        PointerByte[2]= att_data[15];
//        PointerByte[3]= att_data[16];
//        BytesToWrite = 4;
//        Term_Update(BufferToWrite,BytesToWrite);
//      } else {
        PointerByte = (uint8_t*) &uwCRCValue;
        PointerByte[0]=att_data[13];
        PointerByte[1]=att_data[14];
        PointerByte[2]=att_data[15];
        PointerByte[3]=att_data[16];

        SENSING1_PRINTF("OTA %s SIZE=%ld uwCRCValue=%lx\r\n",SENSING1_PACKAGENAME,SizeOfUpdateBlueFW,uwCRCValue);

        /* Reset the Flash */
        //StartUpdateFWBlueMS(SizeOfUpdateBlueFW,uwCRCValue);

        /* Reduce the connection interval */
        {
          int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
                                                        10 /* interval_min*/,
                                                        10 /* interval_max */,
                                                        0   /* slave_latency */,
                                                        400 /*timeout_multiplier*/);
          /* Go to infinite loop if there is one error */
          if (ret != BLE_STATUS_SUCCESS) {
            while (1) {
              SENSING1_PRINTF("Problem Changing the connection interval\r\n");
            }
          //}
        }

        /* Signal that we are ready sending back the CRV value*/
        BufferToWrite[0] = PointerByte[0];
        BufferToWrite[1] = PointerByte[1];
        BufferToWrite[2] = PointerByte[2];
        BufferToWrite[3] = PointerByte[3];
        BytesToWrite = 4;
        Term_Update(BufferToWrite,BytesToWrite);
      }

      SendBackData=0;
    } else if(!strncmp("uid",(char *)(att_data),3)) {
      /* Write back the STM32 UID */
      uint8_t *uid = (uint8_t *)STM32_UUID;
      uint32_t MCU_ID = STM32_MCU_ID[0]&0xFFF;
      BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\n",
                            uid[ 3],uid[ 2],uid[ 1],uid[ 0],
                            uid[ 7],uid[ 6],uid[ 5],uid[ 4],
                            uid[11],uid[ 10],uid[9],uid[8],
                            MCU_ID);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if(!strncmp("setName ",(char *)(att_data),8)) {
      int32_t NameLength= data_length -1;
      int32_t i;

      if(NameLength > 8) {
        if((NameLength - 8) > 7) {
          NameLength= 7;
          BytesToWrite =sprintf((char *)BufferToWrite,"NodeName too long\n");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          NameLength= NameLength - 8;
        }

        for(i=1;i<NameLength+1;i++) {
          NodeName[i]= att_data[i+7];
        }
        /* Fill the Remaining chars with ' ' */
        for(;i<8;i++) {
          NodeName[i]= ' ';
        }

        MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
        NecessityToSaveMetaDataManager=1;

        BytesToWrite =sprintf((char *)BufferToWrite,"New NodeName= %s\n",NodeName);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        BytesToWrite =sprintf((char *)BufferToWrite,"Node Name missing\n");
        Term_Update(BufferToWrite,BytesToWrite);
      }

      SendBackData=0;
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    } else if(!strncmp("SDName ",(char *)(att_data),7)) {
      int32_t NameLength= data_length -1;
      int32_t i;

      if(NameLength > 7) {
        if((NameLength - 7) > 10) {
          NameLength= 10;
          BytesToWrite =sprintf((char *)BufferToWrite,"SD File Name too long\n");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          NameLength= NameLength - 7;
        }

        for(i=0;i<NameLength;i++) {
          DefaultDataFileName[i]= att_data[i+7];
        }
        DefaultDataFileName[i]='\0';
        BytesToWrite =sprintf((char *)BufferToWrite,"SD File Name =%s\n",DefaultDataFileName);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        BytesToWrite =sprintf((char *)BufferToWrite,"File Name missing\n");
        Term_Update(BufferToWrite,BytesToWrite);
        BytesToWrite =sprintf((char *)BufferToWrite,"Use: SDName 'xxxxxxx'\n");
        Term_Update(BufferToWrite,BytesToWrite);
      }
      SendBackData=0;
    } else if(!strncmp("setDate",(char *)(att_data),7)) {

      int32_t NameLength= data_length -1;

      if(NameLength == 19) {
        RTC_DateTypeDef StartDate;

        StartDate.WeekDay= att_data[9] - 48;
        StartDate.Date=  ((att_data[11] - 48) * 16) + (att_data[12] - 48);
        StartDate.Month= ((att_data[14] - 48) * 16) + (att_data[15] - 48);
        StartDate.Year=  ((att_data[17] - 48) * 16) + (att_data[18] - 48);

        if( ((StartDate.WeekDay  > 0x00) && (StartDate.WeekDay  < 0x08)) &&
            ((StartDate.Date     > 0x00) && (StartDate.Date     < 0x32)) &&
            ((StartDate.Month    > 0x00) && (StartDate.Month    < 0x13)) &&
            (StartDate.Year < 0x99) ) {
          /* Configure RTC Data */
          RTC_DataConfig(StartDate.WeekDay, StartDate.Date, StartDate.Month, StartDate.Year);
          BytesToWrite =sprintf((char *)BufferToWrite,"Date format Correct\n");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          BytesToWrite =sprintf((char *)BufferToWrite,"Date format not correct\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"setDate wd/dd/mm/yy\n");
          Term_Update(BufferToWrite,BytesToWrite);
        }
        SendBackData=0;
      }
    } else if(!strncmp("setTime",(char *)(att_data),7)) {
      int32_t NameLength= data_length -1;

      if(NameLength == 16) {
        RTC_TimeTypeDef StartTime;

        StartTime.Hours=   ((att_data[8]  - 48) * 16) + (att_data[9]  - 48);
        StartTime.Minutes= ((att_data[11] - 48) * 16) + (att_data[12] - 48);
        StartTime.Seconds= ((att_data[14] - 48) * 16) + (att_data[15] - 48);

        if( (StartTime.Hours   < 0x24) &&
            (StartTime.Minutes < 0x60) &&
            (StartTime.Seconds < 0x60) ) {
          /* Configure RTC Time */
          RTC_TimeConfig(StartTime.Hours, StartTime.Minutes, StartTime.Seconds);
          BytesToWrite =sprintf((char *)BufferToWrite,"Time format Correct\n");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          BytesToWrite =sprintf((char *)BufferToWrite,"Time format not correct\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"setTime hh:mm:ss\n");
          Term_Update(BufferToWrite,BytesToWrite);
        }
        SendBackData=0;
      }
    } else if(!strncmp("setMicVol",(char *)(att_data),9)) {
      int32_t AudioVolume = atoi((char *)(att_data+9));

      if((AudioVolume<0) & (AudioVolume>64)) {
        BytesToWrite =sprintf((char *)BufferToWrite,"setMicVol Not Correct\n");
      } else {
        TargetBoardFeatures.AudioVolume = AudioVolume;
        BytesToWrite =sprintf((char *)BufferToWrite,"setMicVol Correct =%ld\n",
                            TargetBoardFeatures.AudioVolume);
      }

      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if(!strncmp("MemsStart",(char *)(att_data),9)) {
        SD_Card_FeaturesMask |= FEATURE_MASK_TEMP1 |
                                FEATURE_MASK_TEMP2 |
                                FEATURE_MASK_PRESS |
                                FEATURE_MASK_HUM   |
                                FEATURE_MASK_ACC   |
                                FEATURE_MASK_GRYO  |
                                FEATURE_MASK_MAG;
        SD_CardLoggingMemsStart(0);
        SendBackData=0;

        /* Send back the message */
        if(SD_LogMems_Enabled) {
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
            SDLog_Update(SD_CARD_LOGGING_START);
          }
        }
      } else if(!strncmp("MemsStop",(char *)(att_data),8)) {
        SD_Card_FeaturesMask &= ~(FEATURE_MASK_TEMP1 |
                                  FEATURE_MASK_TEMP2 |
                                  FEATURE_MASK_PRESS |
                                  FEATURE_MASK_HUM   |
                                  FEATURE_MASK_ACC   |
                                  FEATURE_MASK_GRYO  |
                                  FEATURE_MASK_MAG);
        SD_CardLoggingMemsStop();
        SendBackData=0;

        /* Send back the message */
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
          SDLog_Update(SD_CARD_LOGGING_STOP);
        }
      } else if(!strncmp("AudioStart",(char *)(att_data),10)) {
        SD_Card_FeaturesMask |= FEATURE_MASK_BLUEVOICE;
        SD_CardLoggingAudioStart();
        SendBackData=0;

        /* Send back the message */
        if(SD_LogAudio_Enabled) {
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
            SDLog_Update(SD_CARD_LOGGING_START);
          }
        }
      } else if(!strncmp("AudioStop",(char *)(att_data),9)) {
        SD_Card_FeaturesMask &= ~FEATURE_MASK_BLUEVOICE;
        SD_CardLoggingAudioStop();
        SendBackData=0;

        /* Send back the message */
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
          SDLog_Update(SD_CARD_LOGGING_STOP);
        }
      } else if(!strncmp("AudioMemsStart",(char *)(att_data),14)) {
        SD_Card_FeaturesMask |= FEATURE_MASK_BLUEVOICE   |
                                FEATURE_MASK_TEMP1 |
                                FEATURE_MASK_TEMP2 |
                                FEATURE_MASK_PRESS |
                                FEATURE_MASK_HUM   |
                                FEATURE_MASK_ACC   |
                                FEATURE_MASK_GRYO  |
                                FEATURE_MASK_MAG;
        SD_CardLoggingMemsStart(0);
        osDelay(100);
        SD_CardLoggingAudioStart();
        SendBackData=0;

        /* Send back the message */
        if(SD_LogAudio_Enabled & SD_LogMems_Enabled) {
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
            SDLog_Update(SD_CARD_LOGGING_START);
          }
        }
      } else if(!strncmp("AudioMemsStop",(char *)(att_data),13)) {
        SD_Card_FeaturesMask &= ~(FEATURE_MASK_BLUEVOICE   |
                                  FEATURE_MASK_TEMP1 |
                                  FEATURE_MASK_TEMP2 |
                                  FEATURE_MASK_PRESS |
                                  FEATURE_MASK_HUM   |
                                  FEATURE_MASK_ACC   |
                                  FEATURE_MASK_GRYO  |
                                  FEATURE_MASK_MAG);
        SD_CardLoggingMemsStop();
        osDelay(100);
        SD_CardLoggingAudioStop();
        SendBackData=0;

        /* Send back the message */
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)) {
          SDLog_Update(SD_CARD_LOGGING_STOP);
        }
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
    }
  return SendBackData;
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];
  uint32_t SendItBack = 1;

  switch (FeatureMask) {
  case FEATURE_MASK_ACC_EVENTS:
    /* Acc events */
#ifdef SENSING1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%c D=%x\n",FeatureMask,Command,Data);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Conf Sig F=%lx C=%c D=%x\r\n",FeatureMask,Command,Data);
    }
#endif /* SENSING1_DEBUG_CONNECTION */
    if (Data == 1 ) {
      BSP_ACCELERO_Sensor_Enable( TargetBoardFeatures.HandleAccSensor );
    }
    switch(Command) {
      case 'm':
        /* Multiple Events */
        switch(Data) {
          case 1:
            EnableHWMultipleEvents();
            ResetHWPedometer();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)){
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWMultipleEvents();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)){
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
         }
        break;
      case 'f':
        /* FreeFall */
        switch(Data) {
          case 1:
            EnableHWFreeFall();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)){
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWFreeFall();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
         }
      break;
      case 'd':
        /* Double Tap */
        switch(Data) {
          case 1:
            EnableHWDoubleTap();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWDoubleTap();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
        }
      break;
      case 's':
        /* Single Tap */
        switch(Data) {
          case 1:
            EnableHWSingleTap();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWSingleTap();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
        }
      break;
      case 'p':
        /* Pedometer */
        switch(Data) {
          case 1:
            EnableHWPedometer();
            ResetHWPedometer();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWPedometer();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
        }
       break;
      case 'w':
        /* Wake UP */
        switch(Data) {
          case 1:
            EnableHWWakeUp();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWWakeUp();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
        }
       break;
       case 't':
         /* Tilt */
        switch(Data) {
          case 1:
            EnableHWTilt();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
          case 0:
            DisableHWTilt();
            if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
              Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            }
            break;
        }
      break;
      case 'o' :
        /* Tilt */
        switch(Data) {
        case 1:
          EnableHWOrientation6D();
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          }
          break;
        case 0:
          DisableHWOrientation6D();
          if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_CONF_EVENT)) {
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          }
          break;
        }
      break;
    }
    if (Data == 0 ) {
      BSP_ACCELERO_Sensor_Disable( TargetBoardFeatures.HandleAccSensor );
    }
    break;
  }
  return SendItBack;
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
//#define TRACE_HCI_CB
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT) {
#ifdef TRACE_HCI_CB
    SENSING1_PRINTF("HCI_Event_CB: INVALID\n\r");
#endif
    return;
  }

  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
#ifdef TRACE_HCI_CB
    SENSING1_PRINTF("HCI_Event_CB: disconnect\n\r");
#endif
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
#ifdef TRACE_HCI_CB
    SENSING1_PRINTF("HCI_Event_CB: meta event");
#endif
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
#ifdef TRACE_HCI_CB
    SENSING1_PRINTF(" - LE connect\n\r");
#endif
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
        default :
#ifdef TRACE_HCI_CB
    SENSING1_PRINTF(" - other\n\r");
#endif
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
#ifdef TRACE_HCI_CB
    SENSING1_PRINTF("HCI_Event_CB: vendor");
#endif
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
#ifdef TRACE_HCI_CB
          SENSING1_PRINTF(" - read permit req\n\r");
#endif
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
          Read_Request_CB(pr->attr_handle);
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
#ifdef TRACE_HCI_CB
          SENSING1_PRINTF(" - att modified\n\r");
#endif

          {
            evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
            Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
          }
        break;
        default :
#ifdef TRACE_HCI_CB
          SENSING1_PRINTF(" - others \n\r");
#endif
        break;
       }
    }
    break;
  }
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
