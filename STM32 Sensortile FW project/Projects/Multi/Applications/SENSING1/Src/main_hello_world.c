
/**
  ******************************************************************************
  * @file    main.c
  * @author  SRA - Central Labs
  * @version V1.0.0
  * @date    13-Feb-2019
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
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
#include <stdio.h>
#include <string.h> // for memset
#include "main.h"
#include "OTA.h"
#include "SensorTile.101_env_sensors.h"
#include "SensorTile.101_env_sensors_ex.h"
#include "SensorTile.101_motion_sensors.h"
#include "SensorTile.101_motion_sensors_ex.h"
#include "SensorTile.101_audio.h"

/* Private define ------------------------------------------------------------*/
#define MAX_TEMP_SENSORS 2
/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    (0.001F)
/* @brief  Scale factor. It is used to scale acceleration from g to m/s2 */ 
#define FROM_G_TO_MS_2   (9.800655F)

/* @brief  Check Values for understanding if one MEMS Sensor is present or Not */ 
#define SENSING1_SNS_NOT_VALID 9999

/* For enabling the printf on IAR */
//#define SENSING1_ENABLE_PRINTF

/* Uncomment the following define for reading the Environmental sensors with a Single shot 
 * modality instead of Continuous mode */
#define ONE_SHOT

/* Private Macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

#ifdef SENSING1_ENABLE_PRINTF
  #define SENSING1_PRINTF(...) printf(__VA_ARGS__)
#else /* SENSING1_ENABLE_PRINTF */
  #define SENSING1_PRINTF(...)
#endif /* SENSING1_ENABLE_PRINTF */

/* STM32 Unique ID This must be checked for L4+*/
#define STM32_UUID ((uint32_t *)0x1FFF7590)

#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */

#define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000
#define AUDIO_CHANNELS 1

#define MCR_HEART_BIT() \
{ \
  BSP_LED_On(LED1);\
  BSP_LED_On(LED2);\
  HAL_Delay(200);\
  BSP_LED_Off(LED1);\
  BSP_LED_Off(LED2);\
  HAL_Delay(400);\
  BSP_LED_On(LED1);\
  BSP_LED_On(LED2);\
  HAL_Delay(200);\
  BSP_LED_Off(LED1);\
  BSP_LED_Off(LED2);\
  HAL_Delay(1000);\
}

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  int32_t NumTempSensors;

  uint32_t HandleTempSensors[MAX_TEMP_SENSORS];
  uint32_t HandlePressSensor;
  uint32_t HandleHumSensor;

  uint32_t HandleAccSensor;
  uint32_t HandleGyroSensor;
  uint32_t HandleMagSensor;

  float AccSensiMultInG;

} TargetFeatures_t;

/* Exported functions --------------------------------------------------------*/
void APP_UserEvtRx(void *pData);

/* Private variables ---------------------------------------------------------*/
static volatile int ButtonPressed = 0;
static volatile int PowerButtonPressed;
static TargetFeatures_t TargetBoardFeatures;
static volatile int HCI_ProcessEvent=0;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint16_t HWServW2STHandle;
static uint16_t BatteryFeaturesCharHandle;
static uint16_t AudioLevelCharHandle;

Service_UUID_t service_uuid;
Char_UUID_t char_uuid;


uint8_t BufferToWrite[256];
int32_t BytesToWrite;

static uint8_t LastStderrBuffer[W2ST_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_MAX_CHAR_LEN];
static uint8_t LastTermLen;


uint32_t StdErrNotification;
uint32_t TermNotification;

uint8_t set_connectable = 1;
static uint16_t connection_handle = 0;

uint8_t bdaddr[6];
char BoardName[8];

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];
uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
uint16_t PCM_Buffer[AUDIO_CHANNELS*PCM_AUDIO_IN_SAMPLES];


TIM_HandleTypeDef    TimBattHandle;
volatile int SendBatt=0;

TIM_HandleTypeDef    TimMicHandle;
volatile int SendMic=0;

BSP_AUDIO_Init_t MicParams;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void ButtonCallback(void);
static void Init_MEM1_Sensors(void);
static void EnableEnvSensors (void);
static void EnableMotionSensors (void);
static void DisableEnvSensors (void);
static void DisableMotionSensors (void);
static void ReadEnvironmentalData(void);
static void ReadMotionData(void);
void Set2GAccelerometerFullScale(void);
void Set4GAccelerometerFullScale(void);
static void Init_BlueNRG_Stack(void);
static void Init_BlueNRG_Custom_Services(void);

void Set_DeviceConnectable(void);

tBleStatus Add_ConsoleW2ST_Service(void);
tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
tBleStatus Term_Update(uint8_t *data,uint8_t length);
tBleStatus UpdateTermStdOut(uint8_t *data,uint8_t length);
tBleStatus UpdateTermStdErr(uint8_t *data,uint8_t length);

tBleStatus Add_HW_SW_ServW2ST_Service(void);

static void InitTimer(void);

static void Init_MEMS_Mics(void);
void InitMics(void);
void DeInitMics(void);

static void Read_Request_CB(uint16_t handle);
static void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset, uint8_t Attr_Data_Length, uint8_t *Attr_Data);  

tBleStatus GG_Update(void);
tBleStatus AudioLevel_Update(uint16_t *Mic);
static void SendAudioLevelData(void);
void TIM3_Base_MspInit(TIM_HandleTypeDef *htim);
void TIM2_Base_MspInit(TIM_HandleTypeDef *htim);
void AudioProcess_DB_Noise(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure HAL */  
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

  /* Init Led1 */
  BSP_LED_Init(LED1);

  /* Init Led2 */
  BSP_LED_Init(LED2);

  /* Check the BootLoader Compliance */
  if(CheckBootLoaderCompliance()) {
    SENSING1_PRINTF("BootLoader Compliant with FOTA\r\n");
  } else {
    SENSING1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA\r\n");
    BSP_LED_On(LED1);
    BSP_LED_On(LED2);
    while(1);
  }
  
  MCR_HEART_BIT();

  /* Initialize User Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize the Power Button */
  BSP_PowerButton_Init();

  /* Initialize the Battery Charger */
  BSP_BC_Init();

  /* In order to be able to Read Battery Volt */
  BSP_BC_BatMS_Init();

  /* In order to be able to generate one Interrupt when the battery charger
     changes its state */
  BSP_BC_ChrgPin_Init();
  
  MCR_HEART_BIT();
  
  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
  DisableEnvSensors();
  DisableMotionSensors();
  
  /* Enable Sensors */
  EnableEnvSensors();
  EnableMotionSensors();
  
  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();
  
  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();

  /* initialize timer */
  InitTimer();
  
  MCR_HEART_BIT();

  /* Reset the volatile for avoiding to switch off the SensorTile */
  PowerButtonPressed=0;

  while(1) {    
    /* Make the device discoverable */
    if(set_connectable)
    {
      Set_DeviceConnectable();
      set_connectable = FALSE;
    }
  
     /* Handle HCI process */
     if (HCI_ProcessEvent) {
       HCI_ProcessEvent =0;
       hci_user_evt_proc();
     }

    /* Handle the Timer3 for Battery */
    if(SendBatt) {
      SendBatt=0;
      GG_Update();
    }
    
    /* Handle the Timer2 for Mic */
    if(SendMic) {
      SendMic=0;
      SendAudioLevelData();
    }
    
    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;
    }
        
    if(!connection_handle) {
      /* Led Blinking*/
      static int32_t Counter =0;
      Counter++;
      if(Counter==500) {
        BSP_LED_Toggle(LED1);
        Counter=0;
      }
    }

    /* Power Off the SensorTile.101 */
    if(PowerButtonPressed){
      BSP_BC_CmdSend(SHIPPING_MODE_ON);
      PowerButtonPressed =0;
    }

    /* Wait for Event */
    __WFI();
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV5;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
    case USER_BUTTON_PIN:
      /* Handle user button */
      ButtonPressed = 1;
    break;
    case HCI_TL_SPI_EXTI_PIN:
      /* Same define name for Nucleo, SensorTile and IoT01A1 */
      hci_tl_lowlevel_isr();
      HCI_ProcessEvent =1;
    break;
    case POWER_BUTTON_PIN:
      /* Power off the board */
      PowerButtonPressed = 1;
    break;
    case STBC02_CHG_PIN:
      /* For understanding if the SensorTile.101 is under charge */
      BSP_BC_ChgPinHasToggled();
    break;
  }
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  BSP_LED_Toggle(LED2);
  /* Read Environmental Data */
  ReadEnvironmentalData();
  /* Read Motion Data */
  ReadMotionData();
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
#ifdef STM32_SENSORTILE101
  /* Handles for SensorTile.101 */
  TargetBoardFeatures.HandleAccSensor  = LSM6DSO_0;
  TargetBoardFeatures.HandleGyroSensor = LSM6DSO_0;
  TargetBoardFeatures.HandleMagSensor  = LIS2MDL_0;

  TargetBoardFeatures.HandleHumSensor      = HTS221_0;
  TargetBoardFeatures.HandleTempSensors[0] = HTS221_0;

  TargetBoardFeatures.HandlePressSensor    = LPS22HH_0;
  TargetBoardFeatures.HandleTempSensors[1] = LPS22HH_0;
#else
  #error "Write Something here"
#endif /* STM32_SENSORTILE101 */

  /* Accelero/Gyro It's necessary to Init the Component ONLY one Time */
  if (MOTION_SENSOR_Init(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO|MOTION_GYRO) == BSP_ERROR_NONE){
    SENSING1_PRINTF("OK Accelero/Gyroscope Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Accelero/Gyroscope Sensor\n\r");
    TargetBoardFeatures.HandleAccSensor  = SENSING1_SNS_NOT_VALID;
    TargetBoardFeatures.HandleGyroSensor = SENSING1_SNS_NOT_VALID;
  }
  
  /* set default range to 2G */
  Set2GAccelerometerFullScale();

  /* For accelero HW features */
  //InitHWFeatures();


  if(MOTION_SENSOR_Init(TargetBoardFeatures.HandleMagSensor, MOTION_MAGNETO)==BSP_ERROR_NONE){
    SENSING1_PRINTF("OK Magneto Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Magneto Sensor\n\r");
    TargetBoardFeatures.HandleMagSensor = SENSING1_SNS_NOT_VALID;
  }

  /* Humidity/Temperature1 It's necessary to Init the Component ONLY one Time */  
  if(ENV_SENSOR_Init(TargetBoardFeatures.HandleHumSensor, ENV_HUMIDITY|ENV_TEMPERATURE)==BSP_ERROR_NONE){
    SENSING1_PRINTF("OK Humidity/Temperature1 Sensor\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    SENSING1_PRINTF("Error Humidity/Temperature1 Sensor\n\r");
    TargetBoardFeatures.HandleHumSensor      = SENSING1_SNS_NOT_VALID;
    TargetBoardFeatures.HandleTempSensors[0] = SENSING1_SNS_NOT_VALID;
  }
  
  /* Pressure/Temperature 2 It's necessary to Init the Component ONLY one Time */
  if(ENV_SENSOR_Init(TargetBoardFeatures.HandlePressSensor, ENV_PRESSURE|ENV_TEMPERATURE)==BSP_ERROR_NONE){
    SENSING1_PRINTF("OK Pressure/Temperature2 Sensor\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    SENSING1_PRINTF("Error Pressure/Temperature2 Sensor\n\r");
    TargetBoardFeatures.HandlePressSensor    = SENSING1_SNS_NOT_VALID;
    TargetBoardFeatures.HandleTempSensors[1] = SENSING1_SNS_NOT_VALID;
  }
}

/** @brief disable all the Inertial MEMS1 sensors
 * @param None
 * @retval None
 */
static void DisableMotionSensors (void)
{
  if(TargetBoardFeatures.HandleAccSensor != SENSING1_SNS_NOT_VALID) {
    if(MOTION_SENSOR_Disable(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Disabled Accelero Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleGyroSensor != SENSING1_SNS_NOT_VALID) {
    if(MOTION_SENSOR_Disable(TargetBoardFeatures.HandleGyroSensor, MOTION_GYRO)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Disabled Gyroscope Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleMagSensor != SENSING1_SNS_NOT_VALID) {
    if(MOTION_SENSOR_Disable(TargetBoardFeatures.HandleMagSensor, MOTION_MAGNETO)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Disabled Magneto Sensor\r\n");
    }
  }
}

/** @brief Disable all the Environmental MEMS1 sensors
 * @param None
 * @retval None
 */
static void DisableEnvSensors (void)
{ 
  if(TargetBoardFeatures.HandleHumSensor != SENSING1_SNS_NOT_VALID) {
    if(ENV_SENSOR_Disable(TargetBoardFeatures.HandleHumSensor, ENV_HUMIDITY)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Disabled Humidity Sensor\r\n");
      if(ENV_SENSOR_Disable(TargetBoardFeatures.HandleTempSensors[0], ENV_TEMPERATURE)==BSP_ERROR_NONE) {
        SENSING1_PRINTF("Disabled Temperature Sensor1\r\n");
      }
    }
  }

  if(TargetBoardFeatures.HandlePressSensor != SENSING1_SNS_NOT_VALID) {
    if(ENV_SENSOR_Disable(TargetBoardFeatures.HandlePressSensor, ENV_PRESSURE)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Disabled Pressure Sensor\r\n");
      if(ENV_SENSOR_Disable(TargetBoardFeatures.HandleTempSensors[1], ENV_TEMPERATURE)==BSP_ERROR_NONE) {
        SENSING1_PRINTF("Disabled Temperature Sensor2\r\n");
      }
    }
  }
}


/** @brief enable all the Inertial MEMS1 sensors
 * @param None
 * @retval None
 */
static void EnableMotionSensors (void)
{
  if(TargetBoardFeatures.HandleAccSensor != SENSING1_SNS_NOT_VALID) {
    if(MOTION_SENSOR_Enable(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Enabled Accelero Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleGyroSensor != SENSING1_SNS_NOT_VALID) {
    if(MOTION_SENSOR_Enable(TargetBoardFeatures.HandleGyroSensor, MOTION_GYRO)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Enabled Gyroscope Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleMagSensor != SENSING1_SNS_NOT_VALID) {
    if(MOTION_SENSOR_Enable(TargetBoardFeatures.HandleMagSensor, MOTION_MAGNETO)==BSP_ERROR_NONE) {
      SENSING1_PRINTF("Enabled Magneto Sensor\r\n");
    }
  }
}

/** @brief enable all the Environmental MEMS1 sensors
 * @param None
 * @retval None
 */
static void EnableEnvSensors (void)
{ 
  if(TargetBoardFeatures.HandleHumSensor != SENSING1_SNS_NOT_VALID) {
    if(ENV_SENSOR_Enable(TargetBoardFeatures.HandleHumSensor, ENV_HUMIDITY)==BSP_ERROR_NONE) {
#ifdef ONE_SHOT
      SENSING1_PRINTF("Enabled Humidity Sensor (One Shot)\r\n");
#else /* ONE_SHOT */
      SENSING1_PRINTF("Enabled Humidity Sensor\r\n");
#endif /* ONE_SHOT */
      if(ENV_SENSOR_Enable(TargetBoardFeatures.HandleTempSensors[0], ENV_TEMPERATURE)==BSP_ERROR_NONE) {
#ifdef ONE_SHOT
        SENSING1_PRINTF("Enabled Temperature Sensor1 (One Shot)\r\n");
#else /* ONE_SHOT */
        SENSING1_PRINTF("Enabled Temperature Sensor1\r\n");
#endif /* ONE_SHOT */
      }
    }
  }

  if(TargetBoardFeatures.HandlePressSensor != SENSING1_SNS_NOT_VALID) {
    if(ENV_SENSOR_Enable(TargetBoardFeatures.HandlePressSensor, ENV_PRESSURE)==BSP_ERROR_NONE) {
#ifdef ONE_SHOT
      SENSING1_PRINTF("Enabled Pressure Sensor (One Shot)\r\n");
#else /* ONE_SHOT */
      SENSING1_PRINTF("Enabled Pressure Sensor\r\n");
#endif /* ONE_SHOT */
      if(ENV_SENSOR_Enable(TargetBoardFeatures.HandleTempSensors[1], ENV_TEMPERATURE)==BSP_ERROR_NONE) {
#ifdef ONE_SHOT
        SENSING1_PRINTF("Enabled Temperature Sensor2 (One Shot)\r\n");
#else /* ONE_SHOT */
        SENSING1_PRINTF("Enabled Temperature Sensor2\r\n");
#endif /* ONE_SHOT */
      }
    }
  }
}

/**
  * @brief  Read The Environmetal Data (Temperature/Pressure/Humidity)
  * @param  None
  * @retval None
  */
static void ReadEnvironmentalData(void)
{
  float SensorValue;
  int32_t decPart, intPart;
  int32_t PressToSend;
  uint16_t HumToSend;
  int16_t Temp1ToSend, Temp2ToSend;
  

  if(TargetBoardFeatures.HandleHumSensor != SENSING1_SNS_NOT_VALID) {
    /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
    ENV_SENSOR_GetValue(TargetBoardFeatures.HandleHumSensor,ENV_HUMIDITY,&SensorValue);
    MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
    HumToSend = intPart*10+decPart;
    if(TermNotification) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Humidity=%d\r\n",HumToSend);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Humidity=%d\r\n",HumToSend);
    }

    if(TargetBoardFeatures.HandleTempSensors[0] != SENSING1_SNS_NOT_VALID) {
      ENV_SENSOR_GetValue(TargetBoardFeatures.HandleTempSensors[0],ENV_TEMPERATURE,&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart;
      if(TermNotification) {
        BytesToWrite =sprintf((char *)BufferToWrite,"Temp1=%d\r\n",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        SENSING1_PRINTF("Temp1=%d\r\n",Temp1ToSend);
      }
    }
#ifdef ONE_SHOT
    ENV_SENSOR_Set_One_Shot(TargetBoardFeatures.HandleHumSensor);
#endif
  }
  
  if(TargetBoardFeatures.HandlePressSensor != SENSING1_SNS_NOT_VALID) {
    /*Read the previous value of the sensor and restart the One Shot for the next measurement*/
    ENV_SENSOR_GetValue(TargetBoardFeatures.HandlePressSensor,ENV_PRESSURE,&SensorValue);
    MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
    PressToSend=intPart*100+decPart;
    if(TermNotification) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Press=%ld\r\n",PressToSend);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Press=%ld\r\n",PressToSend);
    }

    if(TargetBoardFeatures.HandleTempSensors[1] != SENSING1_SNS_NOT_VALID) {
      ENV_SENSOR_GetValue(TargetBoardFeatures.HandleTempSensors[1],ENV_TEMPERATURE,&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp2ToSend = intPart*10+decPart;
      if(TermNotification) {
        BytesToWrite =sprintf((char *)BufferToWrite,"Temp2=%d\r\n",Temp2ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        SENSING1_PRINTF("Temp2=%d\r\n",Temp2ToSend);
      }
    }
#ifdef ONE_SHOT
    ENV_SENSOR_Set_One_Shot(TargetBoardFeatures.HandlePressSensor);
#endif
  }
}

/**
  * @brief  Read The Motion Data (Acc/Gyro/Mag)
  * @param  None
  * @retval None
  */
static void ReadMotionData(void)
{
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;
  

  /* Read the Acc values */
  if(TargetBoardFeatures.HandleAccSensor != SENSING1_SNS_NOT_VALID ) {
    MOTION_SENSOR_GetAxes(TargetBoardFeatures.HandleAccSensor,MOTION_ACCELERO,&ACC_Value);
    if(TermNotification) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Acc=%d %d %d\r\n",ACC_Value.x,ACC_Value.y,ACC_Value.z);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Acc=%d %d %d\r\n",ACC_Value.x,ACC_Value.y,ACC_Value.z);
    }
  } else {
    ACC_Value.x = ACC_Value.y = ACC_Value.z =0;
  }

  /* Read the Magneto values */
  if(TargetBoardFeatures.HandleMagSensor != SENSING1_SNS_NOT_VALID ) {
    MOTION_SENSOR_GetAxes(TargetBoardFeatures.HandleMagSensor,MOTION_MAGNETO,&MAG_Value);
    if(TermNotification) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Mag=%d %d %d\r\n",MAG_Value.x,MAG_Value.y,MAG_Value.z);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Mag=%d %d %d\r\n",MAG_Value.x,MAG_Value.y,MAG_Value.z);
    }
  } else {
    MAG_Value.x = MAG_Value.y = MAG_Value.z =0;
  }

  /* Read the Gyro values */
  if(TargetBoardFeatures.HandleGyroSensor != SENSING1_SNS_NOT_VALID ) {
    MOTION_SENSOR_GetAxes(TargetBoardFeatures.HandleGyroSensor,MOTION_GYRO,&GYR_Value);
    if(TermNotification) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Gyro=%d %d %d\r\n",GYR_Value.x,GYR_Value.y,GYR_Value.z);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Gyro=%d %d %d\r\n",GYR_Value.x,GYR_Value.y,GYR_Value.z);
    }
  } else {
    GYR_Value.x = GYR_Value.y = GYR_Value.z =0;
  }
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
static void Set2GAccelerometerFullScale(void)
{
  float sensitivity;
  /* Set Full Scale to +/-2g */
  MOTION_SENSOR_SetFullScale( TargetBoardFeatures.HandleAccSensor,MOTION_ACCELERO,2);

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(TargetBoardFeatures.HandleAccSensor,MOTION_ACCELERO,&sensitivity);
  TargetBoardFeatures.AccSensiMultInG = sensitivity * FROM_MG_TO_G ;
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
static void Set4GAccelerometerFullScale(void)
{
  float sensitivity;

  /* Set Full Scale to +/-4g */
  MOTION_SENSOR_SetFullScale( TargetBoardFeatures.HandleAccSensor,MOTION_ACCELERO,4);

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(TargetBoardFeatures.HandleAccSensor,MOTION_ACCELERO,&sensitivity);
  TargetBoardFeatures.AccSensiMultInG = sensitivity * FROM_MG_TO_G;
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  
 BoardName[0]= 'T';
 BoardName[1]= 'i';
 BoardName[2]= 'l';
 BoardName[3]= 'e';
 BoardName[4]= '1';
 BoardName[5]= '0';
 BoardName[6]= '1';
 BoardName[7]= 0;

#ifdef MAC_SENSING1
  {
    uint8_t tmp_bdaddr[6]= {MAC_SENSING1};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_SENSING1 */

  /* Initialize the BlueNRG HCI */
  hci_init(APP_UserEvtRx, NULL);

#ifndef MAC_SENSING1
  #ifdef MAC_STM32UID_SENSING1
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((SENSING1_VERSION_MAJOR-48)*10) + (SENSING1_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_SENSING1 */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, &data_len_out, bdaddr);
 
    if(ret){
      SENSING1_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_SENSING1 */
#else /* MAC_SENSING1 */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret){
     SENSING1_PRINTF("\r\nSetting Public BD_ADDR failed\r\n");
     goto fail;
  }
#endif /* MAC_SENSING1 */

  ret = aci_gatt_init();
  if(ret){
     SENSING1_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS){
     SENSING1_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

#ifndef  MAC_SENSING1
  #ifdef MAC_STM32UID_SENSING1
    ret = hci_le_set_random_address(bdaddr);

    if(ret){
       SENSING1_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
       goto fail;
    }
  #endif /* MAC_STM32UID_SENSING1 */
#endif /* MAC_SENSING1 */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     SENSING1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7, 
                                               16,
                                               USE_FIXED_PIN_FOR_PAIRING,
                                               123456,
                                               0x00);
  if (ret != BLE_STATUS_SUCCESS) {
     SENSING1_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  SENSING1_PRINTF("\r\nSERVER: BLE Stack Initialized \r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n",
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4); /* -2,1 dBm */

  return;

fail:
  return;
}

void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    }
    else if(event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;        

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     SENSING1_PRINTF("Console Service W2ST added successfully\r\n");
  } else {
     SENSING1_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     SENSING1_PRINTF("HW/SW Service W2ST added successfully\r\n");
  } else {
     SENSING1_PRINTF("\r\nError while adding HW/SW Service W2ST\r\n");
  }
}

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
                      uint8_t *charValue)
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
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
   BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
   BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_MAX_CHAR_LEN,
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
  HAL_Delay(20);
  return BLE_STATUS_SUCCESS;
}

tBleStatus UpdateTermStdErr(uint8_t *data,uint8_t length)
{
  if (aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, length , data) != BLE_STATUS_SUCCESS) {
      SENSING1_PRINTF("Error Updating Stdout Char\r\n");
    return BLE_STATUS_ERROR;
  }
  HAL_Delay(20);
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
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_MAX_CHAR_LEN) ?  W2ST_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
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
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_MAX_CHAR_LEN) ?  W2ST_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      } else {
        printf("Error Updating Stdout Char\r\n");
      }
      return BLE_STATUS_ERROR;
    }
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
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      printf("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}


/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void)
{  
 uint8_t local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,BoardName[0],BoardName[1],BoardName[2],BoardName[3],BoardName[4],BoardName[5],BoardName[6]};
  uint8_t manuf_data[26] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,BoardName[0],BoardName[1],BoardName[2],BoardName[3],BoardName[4],BoardName[5],BoardName[6], // Complete Name
    13,0xFF,0x01/*SKD version */,
#ifdef  STM32_SENSORTILE101
    0x06, /* For SensorTile.101 */
#else /* STM32_SENSORTILE101 */
  #error "Define the right platform"
#endif /* STM32_SENSORTILE101 */
    0x00 /* AudioSync+AudioData */,
    0x00 /* ACC+Gyro+Mag*/,
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
  
  manuf_data[17] |= 0x02; /* Battery Present */
  manuf_data[16] |= 0x04; /* Mic */

  /* disable scan response */
  hci_le_set_scan_response_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,

#ifndef MAC_SENSING1
  #ifdef MAC_STM32UID_SENSING1
                           STATIC_RANDOM_ADDR,
  #else /* MAC_STM32UID_SENSING1 */
                           RANDOM_ADDR,
  #endif /* MAC_STM32UID_SENSING1 */
#else /* MAC_SENSING1 */
                           PUBLIC_ADDR,
#endif /* MAC_SENSING1 */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);


  /* Send Advertising data */
  aci_gap_update_adv_data(26, manuf_data);
}


/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  if (handle == StdErrCharHandle + 1) {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
  }

  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

static void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset, uint8_t Attr_Data_Length, uint8_t *Attr_Data)
{
   if(Attr_Handle == StdErrCharHandle + 2){
    if (Attr_Data[0] == 01) {
      SENSING1_PRINTF("-->StdErr\r\n");
      StdErrNotification=1;
    } else if (Attr_Data[0] == 0){
      SENSING1_PRINTF("StdErr<--\r\n");
      StdErrNotification=0;
    }
  } else if(Attr_Handle == TermCharHandle + 2){
    if (Attr_Data[0] == 01) {
      SENSING1_PRINTF("-->TermNot\r\n");
      TermNotification=1;
    } else if (Attr_Data[0] == 0){
      SENSING1_PRINTF("TermNot<--\r\n");
      TermNotification=0;
    }
  } else if(Attr_Handle == BatteryFeaturesCharHandle + 2){
    if (Attr_Data[0] == 01) {
      SENSING1_PRINTF("-->Battery\r\n");
      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Start_IT(&TimBattHandle) != HAL_OK){
        /* Starting Error */
        while(1);
      }
    } else if (Attr_Data[0] == 0){
      SENSING1_PRINTF("Battery<--\r\n");
      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Stop_IT(&TimBattHandle) != HAL_OK){
        /* Stopping Error */
        while(1);
      }
    }
  } else if(Attr_Handle == AudioLevelCharHandle + 2){
    if (Attr_Data[0] == 01) {
      SENSING1_PRINTF("-->Mic\r\n");

//      /* Init Microphone */
//       InitMics();
//
//      {
//        int32_t Count;
//        for(Count=0;Count<AUDIO_CHANNELS;Count++) {
//          RMS_Ch[Count]=0;
//          DBNOISE_Value_Old_Ch[Count] =0;
//        }
//      }
//
//      /* Start the TIM Base generation in interrupt mode */
//      if(HAL_TIM_Base_Start_IT(&TimMicHandle) != HAL_OK){
//        /* Starting Error */
//        while(1);
//      }      
      
    } else if (Attr_Data[0] == 0){
      SENSING1_PRINTF("Mic<--\r\n");
//      DeInitMics();
//
//      /* Stop the TIM Base generation in interrupt mode */
//      if(HAL_TIM_Base_Stop_IT(&TimMicHandle) != HAL_OK){
//        /* Stopping Error */
//        while(1);
//      }
    }    
  } else if (Attr_Handle == TermCharHandle + 1){    
    Term_Update(Attr_Data,Attr_Data_Length);
  } else if (Attr_Handle==(0x0002+2)) {
    /* Check if it's the first Run after a FOTA */
#ifdef BLE_FORCE_RESCAN
    {
#else /* BLE_FORCE_RESCAN */
    if(CheckFirstRunAfterFOTA()) {
#endif /* BLE_FORCE_RESCAN */
      /* Force one UUID rescan for FOTA */
      tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
      uint8_t buff[4];

      /* Delete all the Handles from 0x0001 to 0xFFFF */
      STORE_LE_16(buff  ,0x0001);
      STORE_LE_16(buff+2,0xFFFF);

      ret = aci_gatt_update_char_value(0x0001,0x0002,0,4,buff);

      if (ret == BLE_STATUS_SUCCESS){
        SENSING1_PRINTF("UUID Rescan Forced\r\n");
      } else {
        SENSING1_PRINTF("Problem forcing UUID Rescan\r\n");
      }
    }
  } else {
    if(StdErrNotification){
      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOWN handle\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Notification UNKNOWN handle =%d\r\n",Attr_Handle);
    }
  }
}


/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void InitMics(void)
{
  Init_MEMS_Mics();
  BSP_AUDIO_IN_Record(AUDIO_INSTANCE, (uint8_t *) PCM_Buffer,PCM_AUDIO_IN_SAMPLES*2);
}

/** @brief DeInitialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void DeInitMics(void)
{
  uint8_t ret= BSP_ERROR_NONE;
    
  BSP_AUDIO_IN_Stop(AUDIO_INSTANCE);
  ret= BSP_AUDIO_IN_DeInit(AUDIO_INSTANCE);

  if(ret != BSP_ERROR_NONE) {
    SENSING1_PRINTF("Error Audio DeInit\r\n");
    while(1);
  } else {
    SENSING1_PRINTF("OK Audio DeInit\r\n");
  }
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

  uint8_t max_attr_records =2; /* Battery and Mic */

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
                          1+3*max_attr_records,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_MIC_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid,2+AUDIO_CHANNELS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioLevelCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_GG_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2+2+2+2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &BatteryFeaturesCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //SENSING1_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Gas Gouge characteristic
 * @param  None
 * @retval tBleStatus   Status
 */
tBleStatus GG_Update(void)
{
  tBleStatus ret;

  uint8_t buff[2+2+2+2+1];
  uint16_t volt;
  static uint32_t Status;
  stbc02_State_TypeDef BC_State = {(stbc02_ChgState_TypeDef)0, ""};
  
  /* Battery Volt */
  BSP_BC_GetVoltage(&volt);
  if(TermNotification==0) {
    SENSING1_PRINTF("volt=%d\r\n",volt);
  }
  
  /* Battery State */
  BSP_BC_GetState(&BC_State);
  if(TermNotification==0) {
    SENSING1_PRINTF("BC State [%s]\r\n",BC_State.Name);
  }
  
  if(BC_State.Id==0) {
    Status = 0x01; /* Discharging */
  } else if(BC_State.Id==4) {
    Status = 0x03; /* Charging */
  } else {
    Status     = 0x04; /* Unknown */
  }

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,70); // Fake No %Battery
  STORE_LE_16(buff+4,volt);
  STORE_LE_16(buff+6,0); // No Current
  buff[8] = Status;
  

  ret = aci_gatt_update_char_value(HWServW2STHandle, BatteryFeaturesCharHandle, 0, 2+2+2+2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating GG Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Error Updating GG Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
  
  for(NumberMic=0;NumberMic<(AUDIO_CHANNELS);NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= (16.0f*50);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_INPUT /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  AudioLevel_Update(DBNOISE_Value_Ch);
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
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Mic Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess_DB_Noise(void)
{
  int32_t i;
  int32_t NumberMic;

  for(i = 0; i < 16; i++){
    for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
      RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]));
    }
  }
}

/**
 * @brief  Function for initializing one timer for sending
 * a ping value to BLE
 * @param  None
 * @retval None
 */
static void InitTimer(void)
{
  uint32_t uwPrescalerValue;

  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM3 instance */
  TimBattHandle.Instance = TIM3;
  TimBattHandle.Init.Period = 5000 - 1;
  TimBattHandle.Init.Prescaler = uwPrescalerValue;
  TimBattHandle.Init.ClockDivision = 0;
  TimBattHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM3_Base_MspInit(&TimBattHandle);
  if(HAL_TIM_Base_Init(&TimBattHandle) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
  
  /* Set TIM2 instance */
  TimMicHandle.Instance = TIM2;
  TimMicHandle.Init.Period = 500 - 1;
  TimMicHandle.Init.Prescaler = uwPrescalerValue;
  TimMicHandle.Init.ClockDivision = 0;
  TimMicHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM2_Base_MspInit(&TimMicHandle);
  if(HAL_TIM_Base_Init(&TimMicHandle) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
  
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
static void Init_MEMS_Mics(void)
{
  uint8_t ret;
  
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_CHANNELS;
  MicParams.Device = DMIC_ONBOARD;
  MicParams.SampleRate = AUDIO_SAMPLING_FREQUENCY;
  
  ret = BSP_AUDIO_IN_Init(AUDIO_INSTANCE, &MicParams);

  if(ret != BSP_ERROR_NONE) {
    SENSING1_PRINTF("\nError Audio Init\r\n");
    while(1) {
      ;
    }
  } else {
    SENSING1_PRINTF("\nOK Audio Init\t(Audio Freq.= %ld)\r\n", AUDIO_SAMPLING_FREQUENCY);
  }
  
  /* Set the volume level */
  ret= BSP_AUDIO_IN_SetVolume(AUDIO_INSTANCE,AUDIO_VOLUME_INPUT);
  
  if(ret != BSP_ERROR_NONE) {
    SENSING1_PRINTF("Error Audio Volume\r\n\n");
    
    while(1) {
      ;
    }
  } else {
    SENSING1_PRINTF("OK Audio Volume\t(Volume= %ld)\r\n", AUDIO_VOLUME_INPUT);
  }

}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess_DB_Noise();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess_DB_Noise();
}

 /**
 * @brief TIM MSP Initialization
 * This function configures the hardware resources used in this example:
 *  - Peripheral's clock enable
 *  - Peripheral's Interrupt Configuration
 * @param htim: TIM handle pointer
 * @retval None
 */
void TIM3_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM3) {
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM3_IRQn, 8, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

 /**
 * @brief TIM MSP Initialization
 * This function configures the hardware resources used in this example:
 *  - Peripheral's clock enable
 *  - Peripheral's Interrupt Configuration
 * @param htim: TIM handle pointer
 * @retval None
 */
void TIM2_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM2) {
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM2_IRQn, 8, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{ 
  connection_handle = Connection_Handle;

  /* Switch off the LED1 */
  BSP_LED_Off(LED1);

  SENSING1_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",Peer_Address[5],Peer_Address[4],Peer_Address[3],Peer_Address[2],Peer_Address[1],Peer_Address[0]);

}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  SENSING1_PRINTF("<<<<<<DISCONNECTED\r\n");

  /* Make the device connectable again. */
  set_connectable = TRUE;
  
  /* No Device Connected */
  connection_handle =0;

}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  Read_Request_CB(Attribute_Handle);    
}

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event is given when an attribute change his value.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_Request_CB(Connection_Handle, Attr_Handle, Offset, Attr_Data_Length, Attr_Data);
}


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/


       