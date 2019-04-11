/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Initialization of the Target Platform
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

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
#include "DataLog_Manager.h"
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */

#ifdef STM32_SENSORTILE
  #ifdef SENSING1_ENABLE_PRINTF
    #include "usbd_core.h"
    #include "usbd_cdc.h"
    #include "usbd_cdc_interface.h"

    char PrintfBufferToWrite[256];

  #endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

#ifdef ASC_USE_USB_AUDIO
#include "usbd_desc.h"
#include "usbd_audio_if.h"
#endif /* ASC_USE_USB_AUDIO */

/* Imported variables ---------------------------------------------------------*/
#ifdef STM32_SENSORTILE
  #ifdef SENSING1_ENABLE_PRINTF
     extern USBD_DescriptorsTypeDef VCP_Desc;
  #endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

#ifdef ASC_USE_USB_AUDIO
extern USBD_DescriptorsTypeDef AUDIO_Desc;
extern USBD_AUDIO_ItfTypeDef  USBD_AUDIO_fops;
#endif /* ASC_USE_USB_AUDIO */

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

#ifdef STM32_SENSORTILE
  #ifdef SENSING1_ENABLE_PRINTF
    USBD_HandleTypeDef  USBD_Device;
  #endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

#ifdef ASC_USE_USB_AUDIO
/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUSBDDevice;
#endif /* ASC_USE_USB_AUDIO */

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];
uint16_t PCM_Buffer[AUDIO_CHANNELS*PCM_AUDIO_IN_SAMPLES];

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);
static void Init_MEMS_Mics(uint32_t AudioFreq);

static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  TargetType_t BoardType Nucleo/BlueCoin/SensorTile
  * @retval None
  */
void InitTargetPlatform(TargetType_t BoardType)
{
  TargetBoardFeatures.BoardType = BoardType;
#ifdef STM32_NUCLEO
  #ifdef SENSING1_ENABLE_PRINTF
  /* UART Initialization */
  if(UART_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    SENSING1_PRINTF("UART Initialized\r\n");
  }
  #endif /* SENSING1_ENABLE_PRINTF */

  /* I2C Initialization */
  if(I2C_Global_Init() != COMPONENT_OK) {
    Error_Handler();
  } else {
    SENSING1_PRINTF("I2C  Initialized\r\n");
  }
  
  /* Initialize the BlueNRG SPI driver */
  if(SPI_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    SENSING1_PRINTF("SPI  Initialized\r\n");
  }
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

#elif STM32_SENSORTILE

#ifdef SENSING1_ENABLE_SD_CARD_LOGGING
  /* Configure the SDCard */
  DATALOG_SD_Init();
  HAL_Delay(200);
  sprintf(DefaultDataFileName,"%s","STile");
#endif /* SENSING1_ENABLE_SD_CARD_LOGGING */
  
  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  
  #ifdef SENSING1_ENABLE_PRINTF
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Configure the CDC */
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  /* 10 seconds ... for having time to open the Terminal
   * for looking the SENSING1 Initialization phase */
  HAL_Delay(10000);
  #endif /* SENSING1_ENABLE_PRINTF */
#endif /* STM32_NUCLEO */
  
  /* Initialize LED */
#ifdef STM32_NUCLEO  
  BSP_LED_Init(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Init( LED1 );  
#endif /* STM32_NUCLEO */

  SENSING1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
#ifdef STM32_SENSORTILE
        "\tSTM32476RG-SensorTile board"
#elif STM32_NUCLEO
        "\tSTM32L476RG-Nucleo board"
#endif /* STM32_SENSORTILE */
          "\r\n\n",
          SENSING1_PACKAGENAME,
          SENSING1_VERSION_MAJOR,SENSING1_VERSION_MINOR,SENSING1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();

#ifdef STM32_SENSORTILE
  /* Inialize the Gas Gouge if the battery is present */
  if(BSP_GG_Init(&TargetBoardFeatures.HandleGGComponent) == COMPONENT_OK){
    SENSING1_PRINTF("OK Gas Gouge Component\r\n");
  } else {
    SENSING1_PRINTF("Battery not present\r\n");
  }
#endif /* STM32_SENSORTILE */

  /* Default Microphones' Audio Volume */
  TargetBoardFeatures.AudioVolume = AUDIO_VOLUME_VALUE;

  SENSING1_PRINTF("\n\r");
}

void enableMotionSensors (void)
{
    if(TargetBoardFeatures.HandleAccSensor) {
    if(BSP_ACCELERO_Sensor_Enable( TargetBoardFeatures.HandleAccSensor )==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Accelero Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleGyroSensor) {
    if(BSP_GYRO_Sensor_Enable( TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Gyroscope Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleMagSensor) {
    if(BSP_MAGNETO_Sensor_Enable( TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Magneto Sensor\r\n");
    }
  }
}
void disableMotionSensors (void)
{
  if(TargetBoardFeatures.HandleAccSensor) {
    if(BSP_ACCELERO_Sensor_Disable( TargetBoardFeatures.HandleAccSensor )==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Accelero Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleGyroSensor) {
    if(BSP_GYRO_Sensor_Disable( TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Gyroscope Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleMagSensor) {
    if(BSP_MAGNETO_Sensor_Disable( TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Magneto Sensor\r\n");
    }
  }
}
void enableEnvSensors (void)
{
  
  if(TargetBoardFeatures.HandleHumSensor) {
    if(BSP_HUMIDITY_Sensor_Enable( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Humidity Sensor --> ");
#ifdef ONE_SHOT
      if(BSP_HUMIDITY_Set_One_Shot( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
        SENSING1_PRINTF("Set One Shot Humidity Sensor\r\n");
      }
#endif
    }
  }

  if(TargetBoardFeatures.HandleTempSensors[0]){
    if(BSP_TEMPERATURE_Sensor_Enable( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Temperature Sensor1 --> ");
#ifdef ONE_SHOT
      if(BSP_TEMPERATURE_Set_One_Shot( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
        SENSING1_PRINTF("Set One Shot Temperature Sensor1\r\n");
      }
#endif
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[1]){
    if(BSP_TEMPERATURE_Sensor_Enable( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Temperature Sensor2 --> ");
#ifdef ONE_SHOT
      if(BSP_TEMPERATURE_Set_One_Shot( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
        SENSING1_PRINTF("Set One Shot Temperature Sensor2\r\n");
      }
#endif
    }
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    if(BSP_PRESSURE_Sensor_Enable( TargetBoardFeatures.HandlePressSensor)==COMPONENT_OK) {
      SENSING1_PRINTF("Enabled Pressure Sensor --> ");
    }
  }
  
  TargetBoardFeatures.EnvSensorEnabled= 1;
}
void disableEnvSensors (void)
{
  if(TargetBoardFeatures.HandleHumSensor) {
    if(BSP_HUMIDITY_Sensor_Disable( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Humidity Sensor\r\n");
    }
  }

  if(TargetBoardFeatures.HandleTempSensors[0]){
    if(BSP_TEMPERATURE_Sensor_Disable( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Temperature Sensor1\r\n");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[1]){
    if(BSP_TEMPERATURE_Sensor_Disable( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Temperature Sensor2\r\n");
    }
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    if(BSP_PRESSURE_Sensor_Disable( TargetBoardFeatures.HandlePressSensor)==COMPONENT_OK) {
      SENSING1_PRINTF("Disabled Pressure Sensor\r\n");
    }
  }
  
  TargetBoardFeatures.EnvSensorEnabled= 0;
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{  
  /* Accelero */
  if (BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK){
    SENSING1_PRINTF("OK Accelero Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Accelero Sensor\n\r");
    while(1);
  }
  /* set default range to 2G */
  Set2GAccelerometerFullScale();
  /* For accelero HW features */
  InitHWFeatures();

  /* Gyro */
  if(BSP_GYRO_Init( GYRO_SENSORS_AUTO, &TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK){
    SENSING1_PRINTF("OK Gyroscope Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Gyroscope Sensor\n\r");
    while(1);
  }

  if(BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK){
    SENSING1_PRINTF("OK Magneto Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Magneto Sensor\n\r");
    while(1);
  }

  /* Humidity */  
  if(BSP_HUMIDITY_Init( HUMIDITY_SENSORS_AUTO, &TargetBoardFeatures.HandleHumSensor )==COMPONENT_OK){
    SENSING1_PRINTF("OK Humidity Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Humidity Sensor\n\r");
  }

  /* Temperature1 */
  if(BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     SENSING1_PRINTF("OK Temperature Sensor1\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    SENSING1_PRINTF("Error Temperature Sensor1\n\r");
  }

  /* Temperature2 */
#ifdef STM32_NUCLEO
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     SENSING1_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    SENSING1_PRINTF("Error Temperature Sensor2\n\r");
  }
#elif STM32_SENSORTILE
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     SENSING1_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    SENSING1_PRINTF("Error Temperature Sensor2\n\r");
  }
#endif /* STM32_NUCLEO */
  
  /* Pressure */
  if(BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &TargetBoardFeatures.HandlePressSensor )==COMPONENT_OK){
    SENSING1_PRINTF("OK Pressure Sensor\n\r");
  } else {
    SENSING1_PRINTF("Error Pressure Sensor\n\r");
  }
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
static void Init_MEMS_Mics(uint32_t AudioFreq)
{
  uint8_t ret;
  
  ret= BSP_AUDIO_IN_Init(AudioFreq, 16, AUDIO_CHANNELS);
  
  if(ret != AUDIO_OK) {
    SENSING1_PRINTF("\nError Audio Init\r\n");
    
    while(1) {
      ;
    }
  } else {
    SENSING1_PRINTF("\nOK Audio Init\t(Audio Freq.= %ld)\r\n", AudioFreq);
  }
  
  /* Set the volume level */
  ret= BSP_AUDIO_IN_SetVolume(TargetBoardFeatures.AudioVolume);
  
  if(ret != AUDIO_OK) {
    SENSING1_PRINTF("Error Audio Volume\r\n\n");
    
    while(1) {
      ;
    }
  } else {
    SENSING1_PRINTF("OK Audio Volume\t(Volume= %ld)\r\n", TargetBoardFeatures.AudioVolume);
  }

  /* Number of Microphones */
  TargetBoardFeatures.NumMicSensors=AUDIO_CHANNELS;
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void InitMics(uint32_t AudioFreq)
{
  Init_MEMS_Mics(AudioFreq);
  BSP_AUDIO_IN_Record(PCM_Buffer,0);
}

/** @brief DeInitialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
void DeInitMics(void)
{
  uint8_t ret= AUDIO_OK;
  
  BSP_AUDIO_IN_Stop();  
  ret= BSP_AUDIO_IN_DeInit();
  
  if(ret != AUDIO_OK)
  {
    SENSING1_PRINTF("Error Audio DeInit\r\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    SENSING1_PRINTF("OK Audio DeInit\r\n");
  }
}

#ifdef ASC_USE_USB_AUDIO

/** @brief Initialize USB Audio
 * @param None
 * @retval None
 */
void InitUSBAudio(void)
{

#if (defined(STM32_SENSORTILE) && defined(ASC_USE_USB_AUDIO))
  #error "USB Audio on SensorTile is not supported"
#endif

  /* USB Initialization */
  /* Enable USB power */
  HAL_PWREx_EnableVddUSB();
  /* Init Device Library */
  USBD_Init(&hUSBDDevice, &AUDIO_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&hUSBDDevice, &USBD_AUDIO);
  /* Add Interface callbacks for AUDIO Class */
  USBD_AUDIO_RegisterInterface(&hUSBDDevice, &USBD_AUDIO_fops);
  /* Start USB*/
  USBD_Start(&hUSBDDevice);

  SENSING1_PRINTF("\nOK USB Audio Init\t(Audio Freq.= %ld)\r\n", USBD_AUDIO_FREQ);

}

/** @brief DeInitialize USB Audio
 * @param None
 * @retval None
 */
void DeInitUSBAudio(void)
{
  USBD_StatusTypeDef ret = USBD_FAIL;

  /* Stop USB */
  USBD_Stop(&hUSBDDevice);

  /* DeInit Device Library */
  ret = USBD_DeInit(&hUSBDDevice);

  if (ret != USBD_OK)
  {
    SENSING1_PRINTF("Error USB Audio DeInit\r\n");
    while(1);
  }
  else
  {
    SENSING1_PRINTF("OK USB Audio DeInit\r\n");
  }
}

#endif /* ASC_USE_USB_AUDIO */

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_On(LED2);
#elif STM32_SENSORTILE
  BSP_LED_On( LED1 );
#endif /* STM32_NUCLEO */
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_Off(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Off( LED1 );
#endif /* STM32_NUCLEO */
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_Toggle(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Toggle( LED1 );
#endif /* STM32_NUCLEO */
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}

/**
 * @brief User function for Erasing the MDM on Flash
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MDM_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MDM_FLASH_ADD);
  EraseInitStruct.NbPages     = 2;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success=0;
    Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief User function for Saving the MDM  on the Flash
 * @param void * InitMetaDataVector Pointer to the MDM beginning
 * @param void * EndMetaDataVector Pointer to the MDM end
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector)
{
  uint32_t Success=1;

  /* Store in Flash Memory */
  uint32_t Address = MDM_FLASH_ADD;
  uint64_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  for(WriteIndex =((uint64_t *) InitMetaDataVector); WriteIndex<((uint64_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 8;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      Error_Handler();
      Success =0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
