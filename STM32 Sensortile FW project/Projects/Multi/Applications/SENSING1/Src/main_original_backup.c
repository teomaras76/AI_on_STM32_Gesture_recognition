/**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Main program body
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

/**
 * @mainpage FP-AI-SENSING1 ultra-low power IoT node with Artificial Intelligence
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This firmware package includes Components Device Drivers, Board Support Package
 * and example application for the following STMicroelectronics elements:
 * \if STM32_NUCLEO
 * - X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards
 * - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
 *       HTS221, LPS22HB, LSM6DSL, LSM303AGR
 * - X-NUCLEO-CCA02M1 Digital MEMS microphones expansion board
 * \endif
 * \if STM32_SENSORTILE
 * - STEVAL-STLKT01V1 (SensorTile) evaluation board that contains the following components:
 *     - MEMS sensor devices: HTS221, LPS22HB, LSM303, LSM6DSM
 *     - digital microphone: MP34DT04
 *     - Gas Gauge IC with alarm output: STC3115
 * - FatFs generic FAT file system module (for only STEVAL-STLCS01V1) provides access the storage devices
 *   such as memory card and hard disk.
 * \endif
 * - FreeRTOS Real Time Kernel/Scheduler that allows applications to be organized as a collection of independent threads of execution
 *   (under MIT open source license)
 *
 * - Middleware libraries generated thanks to STM32CubeMx extension called X-CUBE-AI featuring example implementation of neural networks for real-time:
 *	  - human activity recognition (HAR)
 *	  - acoustic scene classification (ASC)
 *
 * \if STM32_NUCLEO
 * @attention
 * <b>Important Hardware Additional Information</b><br>
 * <br>\image html X-NUCLEO-IKS01A2_HW_Info.jpg "Figure 1: X-NUCLEO-IKS01A2 expansion board"
 * <br>Before to connect X-NUCLEO-IKS01A2 with X-NUCLEO-CCA02M1 expansion board through the Arduino UNO R3 extension connector
 * on to X-NUCLEO-IKS01A2 board remove SB25 0-ohm resistor
 * 
 * <br>\image html X-NUCLEO-CCA02M1_HW_Info.jpg "Figure 2: X-NUCLEO-CCA02M1 expansion board"
 * <br>Before to connect the board X-NUCLEO-CCA02M1 with the STM32 L4 Nucleo motherboard through the Morpho connector layout,
 * on to X-NUCLEO-CCA02M1 board, close the solder bridges SB12, SB16 and open the solder bridges SB7, SB15 and SB17
 * \endif
 *
 * <b>Example Application</b>
 *
 * The Example application initializes all the Components and Library creating some Custom Bluetooth services:
 * - The first service exposes all the HW and SW characteristics:
 *  - HW characteristics:
 *      - related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelerometer
 *        and Microphones Signal Noise dB level.
 * \if STM32_SENSORTILE
 *      - Battery status and SD Data Log control
 * \endif
 *
 *      - human activity recognition (HAR) or acoustic scene classification (ASC)
 *
 * - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * \if STM32_NUCLEO
 * The example application allows the user to monitor the initialization phase via UART.
 * Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
 * \elseif STM32_SENSORTILE
 * To enable UART monitoring, it is necessary to disable the SD card data logging functionality and
 * recompile the code with the following line uncommented:
 *
 * >  //#define SENSING1_ENABLE_PRINTF
 * on file:
 *   Projects\Multi\Applications\SENSING1\Inc\SENSING1_config.h
 *
 * This enables the UART that starts with a delay of 10 Seconds for allowing the time to open the UART for looking
 * the initialization phase launching a terminal application and setting the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
 * \endif
 *
 * This example must be used with the related BlueMS Android/iOS application available on Play/itune store (Version 4.1.0 or higher),
 * in order to read the sent information by Bluetooth Low Energy protocol
 *
 *                              --------------------
 *                              |  VERY IMPORTANT  |
 *                              --------------------
 * 4) For each IDE (IAR/µVision/System Workbench), and for each platform (NUCLEO-L476RG/SensorTile),
 * there are some scripts *.bat and *.sh that makes the following operations:
 * - Full Flash Erase
 * - Load the BootLoader on the right flash region
 * - Load the Program (after the compilation) on the right flash region
 * - Reset the board
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "TargetFeatures.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "main.h"

#include "bluenrg_utils.h"
#include "PowerControl.h"

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
#include "DataLog_Manager.h"
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

#include "har_Processing.h"

#include "OTA.h"

#include "asc.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define LED_TIME_OFF 2000UL /* shall not be same than ON time */
#define LED_TIME_ON   100UL

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];

#if defined (__ICCARM__) /*!< IAR Compiler */
  #pragma data_alignment = 4
#endif
float32_t Proc_Buffer_f[PROC_BUFFER_SIZE];

int16_t Fill_Buffer[FILL_BUFFER_SIZE];

/* Exported Variables --------------------------------------------------------*/
osSemaphoreId semRun;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
RTC_DateTypeDef CurrentDate;
RTC_TimeTypeDef CurrentTime;
static volatile uint32_t SD_CardLogging  =0;
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;

uint8_t bdaddr[6];
uint8_t  NodeName[8];

/* Table with All the known Meta Data */
MDM_knownGMD_t known_MetaData[]={
  {GMD_NODE_NAME,      (sizeof(NodeName))},
  {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
};

uint16_t PedometerStepCount= 0;

/* Private variables ---------------------------------------------------------*/
static volatile int           ButtonPressed    = 0 ;
static volatile int           MEMSInterrupt    = 0 ;
static volatile uint32_t      HCI_ProcessEvent = 0 ;
static volatile uint32_t      RunASCEvent      = 0 ;
static volatile uint32_t      SendEnv          = 0 ;
static volatile uint32_t      SendAudioLevel   = 0 ;
static volatile uint32_t      SendAccGyroMag   = 0 ;
static volatile uint32_t      ledTimer         = 0 ;
static volatile hostLinkType_t hostConnection  = NOT_CONNECTED ;

#ifdef STM32_SENSORTILE
static volatile uint32_t SendBatteryInfo =0;
#endif /* STM32_SENSORTILE */

static volatile uint32_t UpdateMotionAR  =0;

static uint32_t index_buff_fill = 0;
static int hciProcessEnable = 1;
static int audioInProgress = 0 ;

/* Private function prototypes -----------------------------------------------*/

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);

static unsigned char ReCallNodeNameFromMemory(void);

static void SendEnvironmentalData(void);
static void MEMSCallback(void);

static void ButtonCallback(void);
static void SendMotionData(void);
static void SendAudioLevelData(void);

static void LedBlinkCb(void const *arg);

static int  HardwareInit(void);

static void ProcessThread(void const *argument);
static void HostThread   (void const *argument);

#ifdef STM32_SENSORTILE
static void SendBatteryInfoData(void);
#endif /* STM32_SENSORTILE */

#ifdef STM32_SENSORTILE
#ifdef SENSING1_ENABLE_PRINTF
static void CdcCb (void const *arg);
static void CdcStart (void);
#ifdef NOT_USED
static void CdcStop (void);
#endif /* NOT_USED */
#endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */


static void AudioProcess(void);
void AudioProcess_FromMics(void);
void AudioProcess_FromPC(uint8_t *pData, uint32_t count);
void AudioProcess_DB_Noise(void);

#if NN_HAR
static void ComputeMotionAR(void);
#elif defined(NN_ASC)
static void RunASC(void);
#endif /* (NN_HAR) */

extern int hci_flush_ReadQ(void);

extern tBleStatus Config_NotifyBLE(uint32_t Feature,uint8_t Command,uint8_t data);
static void startProcessing (void const *arg);

osTimerId timLedId,timEnvId,timBatId,timMotionId,timBattPlugId;
osTimerId timAudioLevId,timActivityId;

/* CMSIS-OS  definitions                                                        */
/* threads */
osThreadDef(THREAD_1, ProcessThread, osPriorityNormal     , 0, configMINIMAL_STACK_SIZE*8);
osThreadDef(THREAD_2, HostThread   , osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*5);

/* Semaphores */
osSemaphoreDef(SEM_Sm1);
/* Mail Queue */
osMailQId  mail;                                               // Mail queue ID
osMailQDef(mail, 15, msgData_t);
/* Timers */
osTimerDef (TimerLedHandle , LedBlinkCb);
osTimerDef (TimerEnvHandle , startProcessing);
osTimerDef (TimerBatHandle , startProcessing);
osTimerDef (TimerBattPlugHandle , startProcessing);
osTimerDef (TimerMotionHandle, startProcessing);
osTimerDef (TimerAudioLevHandle, startProcessing);
osTimerDef (TimerActivityHandle, startProcessing);

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
osTimerId timSdCardLoggingId;
osTimerDef (TimerSdRecordingHandle , startProcessing);
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

#ifdef STM32_SENSORTILE
#ifdef SENSING1_ENABLE_PRINTF
osTimerId timCdcId;
osTimerDef (TimerCdcHandle , CdcCb);
#endif
#endif

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HardwareInit();

#if ( configUSE_TRACE_FACILITY == 1 )
  vTraceEnable(TRC_START);
#endif

  /* Create threads                                                           */
  osThreadCreate(osThread(THREAD_1), NULL);
  osThreadCreate(osThread(THREAD_2), NULL);

  /* Create the semaphores                                                    */
  semRun = osSemaphoreCreate(osSemaphore(SEM_Sm1), 1);

  /* create mail queue                                                        */
  mail = osMailCreate(osMailQ(mail), NULL);

  /* set lowest reachable power mode                                          */

#if (defined(STM32_SENSORTILE) && defined(SENSING1_ENABLE_PRINTF))
  SetMinPowerMode(IDLE_WFI_TICK_SUPRESS);
#else
  SetMinPowerMode(IDLE_SLEEP_STOP);
#endif

  /* Start scheduler                                                          */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler        */
  for (;;);
}

static void ProcessThread(void const *argument)
{
  msgData_t msg;

  while (1){
  if (semRun != NULL){
    if(osSemaphoreWait(semRun, osWaitForever) == osOK) {
      if(set_connectable){
        if(NecessityToSaveMetaDataManager) {
          uint32_t Success = EraseMetaDataManager();
          if(Success) {
            SaveMetaDataManager();
          }
        }
        msg.type  = SET_CONNECTABLE ;
        SendMsgToHost(&msg);
        set_connectable =0;
      }
      /* Handle Interrupt from MEMS */
      if(MEMSInterrupt) {
        MEMSCallback();
        MEMSInterrupt=0;
      }
      /* Handle user button */
      if(ButtonPressed) {
        ButtonCallback();
        ButtonPressed=0;
      }
      /* Environmental Data */
      if(SendEnv) {
        SendEnv=0;
        SendEnvironmentalData();
      }
      /* Mic Data */
      if (SendAudioLevel) {
        SendAudioLevel = 0;
        SendAudioLevelData();
      }

#ifdef NN_ASC
      /* ASC */
      if (RunASCEvent) {
        RunASCEvent = 0;
        RunASC();
      }
#endif /* NN_ASC */

      /* Motion Data */
      if(SendAccGyroMag) {
        SendAccGyroMag=0;
        SendMotionData();
      }

#if (NN_HAR)
      if(UpdateMotionAR) {
        UpdateMotionAR=0;
        ComputeMotionAR();
      }
#endif /* NN_HAR */

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
      if(SD_CardLogging) {
        /* For MEMS data */
        SD_CardLogging=0;
        SdCardMemsRecordingRun(0);
      }

      /* For Audio data */
      if(writeAudio_flag) {
        writeAudio_flag=0;
        SdCardAudioRecordingRun();
      }
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

#ifdef STM32_SENSORTILE
      /* Battery Info Data */
      if(SendBatteryInfo) {
        SendBatteryInfo=0;
        SendBatteryInfoData();
      }
#endif /* STM32_SENSORTILE */
      }
    }
  }
}

int SendMsgToHost(msgData_t *msgPtr)
{
  msgData_t *ptr;
  if (mail)
  {
    ptr = osMailAlloc(mail, osWaitForever);       /* Allocate memory */
    if (ptr)
    {
      Osal_MemCpy(ptr,msgPtr, sizeof(msgData_t));
      osMailPut(mail, ptr);
    }
    else
    {
      SENSING1_PRINTF("SendMsgToHost: mem allocation failed %d\r\n",msgPtr->type);
      return 0;
    }
  }
  return 1;
}

static void HostThread(void const *argument)
{
  msgData_t *msgPtr;
  osEvent  evt;
#ifdef STM32_SENSORTILE
#ifdef SENSING1_ENABLE_PRINTF
  CdcStart();
#endif
#endif

  for (;;) {
    evt = osMailGet(mail, osWaitForever);        /* wait for mail */
    if (evt.status == osEventMail) {
      msgPtr = evt.value.p;
      switch(msgPtr->type)
      {
        case SET_CONNECTABLE:
          hciProcessEnable = 1 ;
          setConnectable();
          hostConnection = NOT_CONNECTED;
          LedBlinkStart();
          break;
        case SET_HOST_LINK_TYPE:
          if (msgPtr->HostLinkType != hostConnection)
          {
#ifdef BLE_LINK_ADAPT
            switch (msgPtr->HostLinkType)
            {
              case AUDIO_HOST_LINK:
                setConnectionParameters(8,17,0,400);
                break;
              case MOTION_HOST_LINK:
                setConnectionParameters(8,17,0,400);
                break;
              case ENV_HOST_LINK:
                setConnectionParameters(400,400,0,400);
                break;
              case DEFAULT_HOST_LINK:
                setConnectionParameters(400,400,0,400);
                break;
              default:
                break;
            }
#endif
            hostConnection = msgPtr->HostLinkType;
          }
          break;

        case  PROCESS_EVENT :
          if (hciProcessEnable)
          {
            HCI_Process();
          }
          break;

        case  CONF_NOTIFY :
          Config_NotifyBLE(msgPtr->conf.feature,msgPtr->conf.command,msgPtr->conf.data);
          break;

        case  ACC :
          AccEvent_Notify(msgPtr->acc, 2);
          break;

        case  ACC_STEP :
          if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER))
            AccEvent_Notify(msgPtr->stepCnt, 2);
          if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS))
            AccEvent_Notify(msgPtr->stepCnt, 3);
          break;

        case  AUDIO_LEV :
          AudioLevel_Update(msgPtr->DBNOISE_Value_Ch);
          break;
        case  ENV :
          Environmental_Update(msgPtr->env.press,msgPtr->env.hum,msgPtr->env.temp2, msgPtr->env.temp1);
          break;

        case MOTION :
          AccGyroMag_Update(&(msgPtr->motion.acc),&(msgPtr->motion.gyr),&(msgPtr->motion.mag));
          break;
#if NN_HAR
        case ACTIVITY:
          ActivityRec_Update(msgPtr->activity);
          break;
#endif /* NN_HAR */
        case AUDIO_SC:
          AudioSRec_Update(msgPtr->audio_scene);
          break;
#ifdef STM32_SENSORTILE
        case BATTERY_INFO:
          GG_Update(msgPtr->batteryInfo.soc, msgPtr->batteryInfo.voltage,
                    msgPtr->batteryInfo.current);
          break;
#endif /* STM32_SENSORTILE */
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
      case SD_CARD_LOGGING:
        break;
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
        case TERM_STDOUT:
          UpdateTermStdOut(msgPtr->term.data,msgPtr->term.length);
          break;

        case TERM_STDERR:
          UpdateTermStdErr(msgPtr->term.data,msgPtr->term.length);
          break;

        default :
          SENSING1_PRINTF("HostThread unexpected message:%d\r\n",msgPtr->type );
      }
      osMailFree(mail, msgPtr);                                              /* free memory allocated for mail */
      if( semRun    ) osSemaphoreRelease(semRun);                            /* check subsequent processing    */
    }
  }
}

static int HardwareInit(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

  initPowerController();

#ifdef STM32_NUCLEO
  InitTargetPlatform(TARGET_NUCLEO);
#elif STM32_SENSORTILE
  InitTargetPlatform(TARGET_SENSORTILE);
#endif /* STM32_NUCLEO */

  /* Check the MetaDataManager */
 InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL);

  SENSING1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4dmS dB noise\r\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         ENV_UPDATE_MS,
         INERTIAL_UPDATE_MS,
         MICS_DB_UPDATE_MS);
  
#if   defined(NN_GMP)
  SENSING1_PRINTF("\tEnabled AR NN_GMP\r\n");
#elif defined(NN_IGN)
  SENSING1_PRINTF("\tEnabled AR NN_IGN\r\n");
#elif defined(NN_IGN_WSDM)
  SENSING1_PRINTF("\tEnabled AR NN_IGN_WSDM\r\n");
#elif defined(NN_ASC)
  SENSING1_PRINTF("\tEnabled ASC\r\n");
#endif /* defined(NN_GMP) */

  /* Set Node Name */
  ReCallNodeNameFromMemory();

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();
  
  /* Check the BootLoader Compliance */
  if(CheckBootLoaderCompliance()) {
    SENSING1_PRINTF("BootLoader Compliant with FOTA\r\n");
  } else {
    SENSING1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA\r\n");
  }

#ifdef STM32_SENSORTILE
  #ifdef SENSING1_ENABLE_PRINTF
    CDC_FlushBuff();
  #endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
  return 0;
}

static void startProcessing  (void const *arg)
{
  if     (arg == timEnvId){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV))
      SendEnv=1;
  }
#ifdef STM32_SENSORTILE
  else if     (arg == timBatId){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT))
      SendBatteryInfo= 1;
  }
#endif /* STM32_SENSORTILE */
  else if (arg == timMotionId){
    SendAccGyroMag=1;
  }
  else if (arg == timAudioLevId){
    SendAudioLevel=1;
  }
  else if (arg == timActivityId){
    UpdateMotionAR=1;
  }
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  else if(arg == timSdCardLoggingId){
    SD_CardLogging= 1;
  }
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
  else{
    SENSING1_PRINTF("wrong timer : %ld\n",(uint32_t)arg);
  }
  if( semRun ) osSemaphoreRelease(semRun);
}

int startProc(msgType_t type,uint32_t period)
{
  msgData_t msg;
  msg.type         = SET_HOST_LINK_TYPE;
  msg.HostLinkType = DEFAULT_HOST_LINK;
  osTimerId id     = NULL;
  switch (type)
  {
    case BATTERY_INFO:
      if (!timBatId)
        timBatId = osTimerCreate (osTimer(TimerBatHandle), osTimerPeriodic, NULL);
      id = timBatId;
      break;
    case BATTERY_PLUG:
      if (!timBattPlugId)
        timBattPlugId = osTimerCreate (osTimer(TimerBattPlugHandle), osTimerPeriodic, NULL);
      id = timBattPlugId;
      break;
    case ENV:
      if (!timEnvId)
        timEnvId = osTimerCreate (osTimer(TimerEnvHandle), osTimerPeriodic, NULL);
      if(!TargetBoardFeatures.EnvSensorEnabled)
        enableEnvSensors();
      msg.HostLinkType  = ENV_HOST_LINK;
      id = timEnvId;
       break;
    case MOTION:
      if (!timMotionId)
        timMotionId = osTimerCreate (osTimer(TimerMotionHandle),osTimerPeriodic, NULL);
      enableMotionSensors ();
      Set2GAccelerometerFullScale();
      msg.HostLinkType  = MOTION_HOST_LINK;
      id = timMotionId;
      break;
    case AUDIO_LEV:
      if (!timAudioLevId)
        timAudioLevId = osTimerCreate (osTimer(TimerAudioLevHandle),osTimerPeriodic, NULL);
      id = timAudioLevId;
      PowerCtrlLock();
      msg.HostLinkType  = AUDIO_HOST_LINK;
      audioInProgress |= 0x1 ;
      break;
#if defined(NN_ASC)
    case AUDIO_SC:
      /* Initialize Acoustic Scene Recognition */
      ASC_Init();
      break;
#endif /* defined(NN_ASC) */
#if (NN_HAR)
    case ACTIVITY:
     if (!timActivityId)
        timActivityId = osTimerCreate (osTimer(TimerActivityHandle),osTimerPeriodic, NULL);
      BSP_ACCELERO_Sensor_Enable( TargetBoardFeatures.HandleAccSensor );
#ifdef NN_IGN_WSDM
      Set2GAccelerometerFullScale();
#else
      Set4GAccelerometerFullScale();
#endif
      /* Initialize HAR Library */
      HAR_Initialize();
      id = timActivityId;
      break;
#endif /* NN_HAR */
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
     case SD_CARD_LOGGING:
       if (!timSdCardLoggingId)
         timSdCardLoggingId = osTimerCreate (osTimer(TimerSdRecordingHandle), osTimerPeriodic, NULL);
       id = timSdCardLoggingId;
       break;
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
    default :
      SENSING1_PRINTF("wrong type : %d\n",type);
      break;
  }
  if (id){
    if  (osTimerStart (id, period) != osOK){
      SENSING1_PRINTF("failed starting timer\n");
    }

  }
  SendMsgToHost(&msg);
  return 0;
}

int stopProc(msgType_t type)
{
  msgData_t msg;
  msg.type          = SET_HOST_LINK_TYPE;
  msg.HostLinkType  = DEFAULT_HOST_LINK;
  osTimerId id = NULL;
  switch (type)
  {
    case BATTERY_INFO:
      id           = timBatId;
      timBatId     = NULL;
      break;
    case BATTERY_PLUG:
      id           = timBattPlugId;
      timBattPlugId= NULL;
      break;
    case ENV:
      id           = timEnvId;
      timEnvId     = NULL;
      disableEnvSensors();
      break;
    case MOTION:
      id            = timMotionId;
      timMotionId   = NULL;
      disableMotionSensors ();
      break;
    case AUDIO_LEV:
      id            = timAudioLevId;
      if (audioInProgress&0x1) PowerCtrlUnLock();
      timAudioLevId = NULL;
      audioInProgress &= 0x2+0x4 ;
      break;
#if defined(NN_ASC)
    case AUDIO_SC:
      /* DeInitialize Acoustic Scene Recognition */
      ASC_DeInit();
      break;
#endif /* defined(NN_ASC) */
#if (NN_HAR)
    case ACTIVITY:
      id            = timActivityId;
      timActivityId = NULL;
      BSP_ACCELERO_Sensor_Disable( TargetBoardFeatures.HandleAccSensor );
      HAR_DeInitialize();
      break;
#endif /* (NN_HAR) */
#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
    case SD_CARD_LOGGING:
      id                 = timSdCardLoggingId;
      timSdCardLoggingId = NULL;
      break;
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

  default :
      break;
  }
  if (id){
    if  (osTimerStop (id) != osOK){
      SENSING1_PRINTF("could not stop timer\n");
    }
    if (osTimerDelete (id) != osOK)  {
      SENSING1_PRINTF("could not delete timer\n");
    }
  }
  SendMsgToHost(&msg);
  return 0;
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
void Set2GAccelerometerFullScale(void)
{
  float sensitivity;
  /* Set Full Scale to +/-2g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,2.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  TargetBoardFeatures.AccSensiMultInG = sensitivity * FROM_MG_TO_G ;
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set4GAccelerometerFullScale(void)
{
  float sensitivity;

  /* Set Full Scale to +/-4g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,4.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  TargetBoardFeatures.AccSensiMultInG = sensitivity * FROM_MG_TO_G;
}


/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  SENSING1_PRINTF("UserButton Pressed\r\n");
}

void  AccEvent_Msg(AccEventType Event)
{
  msgData_t msg;
  msg.type  = ACC;
  msg.acc   = Event;
  SendMsgToHost(&msg);
}
void  AccStepEvent_Msg(uint16_t stepCnt)
{
  msgData_t msg;
  msg.type    = ACC_STEP;
  msg.stepCnt = stepCnt;
  SendMsgToHost(&msg);
}

/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  ACCELERO_Event_Status_t status;

  BSP_ACCELERO_Get_Event_Status_Ext(TargetBoardFeatures.HandleAccSensor,&status);

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) ||
	  (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) )
  {
    /* Check if the interrupt is due to Pedometer */
    if(status.StepStatus != 0) {
      PedometerStepCount = GetStepHWPedometer();
       if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER))
        AccStepEvent_Msg(PedometerStepCount);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) )
  {
    /* Check if the interrupt is due to Free Fall */
    if(status.FreeFallStatus != 0) {
      AccEvent_Msg(ACC_FREE_FALL);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) )
  {
    /* Check if the interrupt is due to Single Tap */
    if(status.TapStatus != 0) {
      AccEvent_Msg(ACC_SINGLE_TAP);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) )
  {
    /* Check if the interrupt is due to Double Tap */
    if(status.DoubleTapStatus != 0) {
      AccEvent_Msg(ACC_DOUBLE_TAP);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) )
  {
    /* Check if the interrupt is due to Tilt */
    if(status.TiltStatus != 0) {
      AccEvent_Msg(ACC_TILT);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) )
  {
    /* Check if the interrupt is due to 6D Orientation */
    if(status.D6DOrientationStatus != 0) {
      AccEventType Orientation = GetHWOrientation6D();
      AccEvent_Msg(Orientation);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    if(status.WakeUpStatus != 0) {
      AccEvent_Msg(ACC_WAKE_UP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS))
  {
    AccStepEvent_Msg(PedometerStepCount);
  }
}
/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  SensorAxes_t ACC_Value;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;
  msgData_t msg;

  /* Read the Acc values */
  BSP_ACCELERO_Get_Axes(TargetBoardFeatures.HandleAccSensor,&ACC_Value);

  /* Read the Magneto values */
  BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);
  msg.type    = MOTION;
  msg.motion.acc  = ACC_Value ;
  msg.motion.gyr  = GYR_Value;
  msg.motion.mag  = MAG_Value;
  SendMsgToHost(&msg);
}

#if (NN_HAR)
/**
  * @brief  MotionAR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionAR(void)
{
  static HAR_output_t ActivityCodeStored = HAR_NOACTIVITY;
  HAR_output_t ActivityCode;
  SensorAxesRaw_t ACC_Value_Raw;
  msgData_t msg;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);

  ActivityCode =  HAR_run(ACC_Value_Raw);

  if(ActivityCodeStored!=ActivityCode){
    ActivityCodeStored = ActivityCode;
    msg.type           = ACTIVITY;
    msg.activity       = ActivityCode ;
    SendMsgToHost(&msg);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: AR=%d\n",ActivityCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      SENSING1_PRINTF("Sending: AR=%d\r\n",ActivityCode);
    }
  }
}
#endif /* NN_HAR */

/**
* @brief  Callback function when 1ms PCM Audio is received from Microphones
* @param  none
* @retval None
*/
void AudioProcess_FromMics(void)
{

  /* Copy PCM Buffer from Microphones onto FILL BUFFER */
  memcpy(Fill_Buffer + index_buff_fill, PCM_Buffer, sizeof(int16_t) * 16);
  index_buff_fill += 16;

  AudioProcess();

  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  if(SD_LogAudio_Enabled)
  {
    AudioProcess_SD_Recording();
  } else
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
  {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)) {
      AudioProcess_DB_Noise();
    }
  }
}

/**
* @brief  Callback function when 1ms PCM Audio is received from USB
* @param  none
* @retval None
*/
void AudioProcess_FromPC(uint8_t *pData, uint32_t count)
{
  /* Copy PCM Buffer on FILL BUFFER */
  /* In this USB Audio configuration, the signal comes in as 2ch, extract 1ch */
  for (uint32_t i = 0; i < count / 4; i++)
  {
    Fill_Buffer[index_buff_fill] = ((int16_t *) pData)[2 * i];
    index_buff_fill++;
  }
  /* Use memcpy if input buffer is 1ch */
  // memcpy(Fill_Buffer + index_buff_fill, pData, sizeof(int16_t) * 16);
  // index_buff_fill += 16;

  AudioProcess();
}

/**
* @brief  Function that is called when data has been appended to Fill Buffer
* @param  none
* @retval None
*/
static void AudioProcess(void)
{
  float32_t sample;

  /* Create a 64ms (1024 samples) window every 32ms (512 samples)
   Audio Feature Extraction is ran every 32ms on a 64ms window (50% overlap) */
  if (index_buff_fill == FILL_BUFFER_SIZE) {
    /* Copy Fill Buffer in Proc Buffer */
    for (uint32_t i = 0; i < FILL_BUFFER_SIZE; i++) {
      sample = ((float32_t) Fill_Buffer[i]);
      /* Invert the scale of the data */
      sample /= (float32_t) ((1 << (8 * sizeof(int16_t) - 1)));
      Proc_Buffer_f[i] = sample;
    }

    /* Left shift Fill Buffer by 512 samples */
    memmove(Fill_Buffer, Fill_Buffer + (FILL_BUFFER_SIZE / 2), sizeof(int16_t) * (FILL_BUFFER_SIZE / 2));
    index_buff_fill = (FILL_BUFFER_SIZE / 2);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ASC_EVENT)) {
      /* Release processing thread to start Audio Feature Extraction */
      RunASCEvent = 1;
    }

    if (semRun) {
      osSemaphoreRelease(semRun);
    }
  }
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

#ifdef NN_ASC
/**
  * @brief  Acoustic Scene Recognition Working function
  * @param  None
  * @retval None
  */
void RunASC(void)
{
  static ASC_OutputTypeDef classification_result_stored = ASC_UNDEFINED;
  ASC_OutputTypeDef classification_result;
  msgData_t msg;

  /* ASC_Run needs to be called 32 times before it can run the NN and return a classification */
  classification_result = ASC_Run(Proc_Buffer_f);

  /* Only display classification result if a valid classification is returned */
  if (classification_result != ASC_UNDEFINED)
  {
    if (classification_result_stored != classification_result)
    {
      classification_result_stored = classification_result;
      msg.type       = AUDIO_SC;
      msg.audio_scene = classification_result;
      SendMsgToHost(&msg);

      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
         BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ASC=%d\n", classification_result);
         Term_Update(BufferToWrite,BytesToWrite);
      } else {
        SENSING1_PRINTF("Sending: ASC=%d\r\n", classification_result);
      }
    }
  }
}
#endif /* NN_ASC */

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
  msgData_t msg;

  for(NumberMic=0;NumberMic<(AUDIO_CHANNELS);NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= (16.0f*MICS_DB_UPDATE_MS);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (TargetBoardFeatures.AudioVolume /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  msg.type  = AUDIO_LEV;
  memcpy(&(msg.DBNOISE_Value_Ch),&DBNOISE_Value_Ch,AUDIO_CHANNELS* sizeof(uint16_t));
  SendMsgToHost(&msg);
}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  AudioProcess_FromMics();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  AudioProcess_FromMics();
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  uint8_t Status;
  msgData_t msg;

  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
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
    msg.type      = ENV;
    msg.env.press = PressToSend;
    msg.env.hum   = HumToSend;
    msg.env.temp2 = Temp2ToSend;
    msg.env.temp1 = Temp1ToSend;
    SendMsgToHost(&msg);
  }
}

#ifdef STM32_SENSORTILE
/**
  * @brief  Send Battery Info Data (Voltage/Current/Soc) to BLE
  * @param  None
  * @retval None
  */
static void SendBatteryInfoData(void)
{
  uint32_t soc;
  int32_t current= 0;

  msgData_t msg;
  uint32_t voltage;
  uint8_t v_mode;

  /* Update Gas Gouge Status */
  BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

  /* Read the Gas Gouge Status */
  BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
  BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);
  BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);

  /* Battery Informations */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT)) {

    msg.type                 = BATTERY_INFO;
    msg.batteryInfo.soc      = soc;
    msg.batteryInfo.current  = current;
    msg.batteryInfo.voltage  = voltage;

    SendMsgToHost(&msg);
  }
}
#endif /* STM32_SENSORTILE */


/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  char BoardName[8];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;

  for(int i=0; i<7; i++) {
    BoardName[i]= NodeName[i+1];
  }

  BoardName[7]= 0;

#ifdef MAC_BLUEMS
  {
    uint8_t tmp_bdaddr[6]= {MAC_BLUEMS};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_BLUEMS */

#ifndef STM32_NUCLEO
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
#endif /* STM32_NUCLEO */

  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();

#ifndef MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((SENSING1_VERSION_MAJOR-48)*10) + (SENSING1_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_BLUEMS */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

    if(ret){
      SENSING1_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_BLUEMS */
#else /* MAC_BLUEMS */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret){
     SENSING1_PRINTF("\r\nSetting Public BD_ADDR failed\r\n");
     goto fail;
  }
#endif /* MAC_BLUEMS */

  ret = aci_gatt_init();
  if(ret){
     SENSING1_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS){
     SENSING1_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

#ifndef  MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
    ret = hci_le_set_random_address(bdaddr);

    if(ret){
       SENSING1_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
       goto fail;
    }
  #endif /* MAC_STM32UID_BLUEMS */
#endif /* MAC_BLUEMS */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     SENSING1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
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

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     SENSING1_PRINTF("HW & SW Service W2ST added successfully\r\n");
  } else {
     SENSING1_PRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
  }

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     SENSING1_PRINTF("Console Service W2ST added successfully\r\n");
  } else {
     SENSING1_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     SENSING1_PRINTF("Config  Service W2ST added successfully\r\n");
  } else {
     SENSING1_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

#if (defined(STM32_NUCLEO) && !defined(NN_ASC))
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
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;  /* Previous value = RCC_MSIRANGE_6; */
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6; /* Previous value = 1 */
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 4; /* Previous value = 2 */
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
}
#else

/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

//  __HAL_RCC_PWR_CLK_ENABLE();
//  HAL_PWR_EnableBkUpAccess();

  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }

  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();

  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }

  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();

  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}
#endif /* STM32_NUCLEO */

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
/**
  * @brief  Configure the current date.
  * @param  WeekDay Specifies the RTC Date WeekDay and it can be a value of @ref RTC_WeekDay_Definitions
  * @param  Date    Specifies the RTC Date Month (in BCD format) and it can be a value of @ref RTC_Month_Date_Definitions
  * @param  Month   Specifies the RTC Date and it must be a number between Min_Data = 1 and Max_Data = 31
  * @param  Year    Specifies the RTC Date Year and it must be a number between Min_Data = 0 and Max_Data = 99
  * @retval None
  */
void RTC_DataConfig(uint8_t WeekDay, uint8_t Date, uint8_t Month, uint8_t Year)
{
  RTC_DateTypeDef  sdatestructure;

  sdatestructure.WeekDay = WeekDay;
  sdatestructure.Date = Date;
  sdatestructure.Month = Month;
  sdatestructure.Year = Year;

  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  Configure the current time.
  * @param  Hours   Specifies the RTC Time Hour.
  *                 This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected.
  *                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected
  * @param  Minutes Specifies the RTC Time Minutes and it must be a number between Min_Data = 0 and Max_Data = 59
  * @param  Seconds Specifies the RTC Time Seconds and it must be a number between Min_Data = 0 and Max_Data = 59
  * @retval None
  */
void RTC_TimeConfig(uint8_t Hours, uint8_t Minutes, uint8_t Seconds)
{
  RTC_TimeTypeDef  stimestructure;

  stimestructure.Hours = Hours;
  stimestructure.Minutes = Minutes;
  stimestructure.Seconds = Seconds;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
  * @brief  Alarm callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  SD_CardLogging= 1;
  if(semRun) osSemaphoreRelease(semRun);
}

/**
  * @brief  Get the current data and time value.
  * @param  None
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef RTC_GetCurrentDateTime(void)
{
  HAL_StatusTypeDef Status;
  /* Get the RTC current Time */
  Status = HAL_RTC_GetTime(&RtcHandle, &CurrentTime, RTC_FORMAT_BIN);
  if(Status == HAL_OK)  {
    /* Get the RTC current Date */
    Status = HAL_RTC_GetDate(&RtcHandle, &CurrentDate, RTC_FORMAT_BIN);
  }
  return Status;
}
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

/**
  * @brief This function provides accurate delay (in milliseconds) based
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    msgData_t msg;

  switch(GPIO_Pin){
#ifdef STM32_NUCLEO
    case SPI1_CMN_DEFAULT_IRQ_PIN:
#else
    case BNRG_SPI_EXTI_PIN:
#endif /* STM32_NUCLEO */
      HCI_Isr();
      msg.type  = PROCESS_EVENT;
      SendMsgToHost(&msg);
    break;
#ifdef STM32_NUCLEO
  case KEY_BUTTON_PIN:
    ButtonPressed = 1;
    if(semRun) osSemaphoreRelease(semRun);
    break;
#endif /* STM32_NUCLEO */

#ifdef STM32_NUCLEO
    case LSM6DSL_INT1_O_PIN:
    case LSM6DSL_INT2_O_PIN:
#elif STM32_SENSORTILE
  case LSM6DSM_INT2_PIN:
#endif /* STM32_NUCLEO */
    MEMSInterrupt=1;
    if(semRun) osSemaphoreRelease(semRun);
    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: SENSING1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

/**
 * @brief  Check if there are a valid Node Name Values in Memory and read them
 * @param  None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallNodeNameFromMemory(void)
{
  const char DefaultBoardName[7] = {NAME_BLUEMS};

  /* ReLoad the Node Name Values from RAM */
  unsigned char Success=0;

  /* Recall the node name Credential saved */
  MDM_ReCallGMD(GMD_NODE_NAME,(void *)&NodeName);

  if(NodeName[0] != 0x12) {
    NodeName[0]= 0x12;

    for(int i=0; i<7; i++) {
      NodeName[i+1]= DefaultBoardName[i];
    }

    MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
    NecessityToSaveMetaDataManager=1;
  }

  return Success;
}

void LedBlinkStart(void)
{
  ledTimer = LED_TIME_ON;
  LedOnTargetPlatform();
  if (!timLedId) {
    timLedId     = osTimerCreate (osTimer(TimerLedHandle),osTimerOnce, NULL);
  }
  if (timLedId){
    if  (osTimerStart (timLedId, ledTimer) != osOK){
        SENSING1_PRINTF("failed starting timer\n\r");
    }
  }
}

void LedBlinkStop(void)
{
  LedOffTargetPlatform();
  if (timLedId) {
    if  (osTimerStop (timLedId) != osOK){
      SENSING1_PRINTF("could not stop led timer\n\r");
    }
    if (osTimerDelete (timLedId) != osOK)  {
      SENSING1_PRINTF("could not delete led timer\n\r");
    }
  timLedId = NULL;
  ledTimer = (uint32_t)NULL;
  }
}

static void LedBlinkCb  (void const *arg)
{
  if (ledTimer == LED_TIME_ON){
    ledTimer = LED_TIME_OFF;
    LedOffTargetPlatform();
  }
  else{
    ledTimer = LED_TIME_ON;
    LedOnTargetPlatform();
  }
  if (timLedId){
    if  (osTimerStart (timLedId, ledTimer) != osOK){
        SENSING1_PRINTF("failed starting timer\n\r");
    }
  }
}
#ifdef STM32_SENSORTILE
#ifdef SENSING1_ENABLE_PRINTF
void CdcStart(void)
{
  if (!timCdcId) {
    timCdcId     = osTimerCreate (osTimer(TimerCdcHandle),osTimerPeriodic, NULL);
  }
  if (timCdcId){
    if  (osTimerStart (timCdcId, 500) != osOK){
        SENSING1_PRINTF("failed starting timer\n\r");
    }
  }
}

#ifdef NOT_USED
void CdcStop(void)
{
  if (timCdcId) {
    if  (osTimerStop (timCdcId) != osOK){
      SENSING1_PRINTF("could not stop timer\n\r");
    }
    if (osTimerDelete (timCdcId) != osOK)  {
      SENSING1_PRINTF("could not delete timer\n\r");
    }
  timCdcId = NULL;
  }
}
#endif
static void CdcCb (void const *arg)
{
  CDC_FlushBuff();
}
#endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
void vApplicationStackOverflowHook (void)
{
  while (1)
  {

  }
}
/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/