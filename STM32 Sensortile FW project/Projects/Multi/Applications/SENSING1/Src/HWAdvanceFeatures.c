/**
  ******************************************************************************
  * @file    HWAdvanceFeatures.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   DS3/DSM HW Advance Features API
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
#include "HWAdvanceFeatures.h"
#include "TargetFeatures.h"
#include "sensor_service.h"


#ifdef STM32_NUCLEO
    #define INT_PIN         INT1_PIN
    #define INT_PIN_WAKE_UP INT2_PIN
#elif STM32_SENSORTILE
  #define INT_PIN         INT2_PIN
  #define INT_PIN_WAKE_UP INT2_PIN
#endif /* STM32_NUCLEO */


/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PedometerStepCount;

/* Exported variables ---------------------------------------------------------*/
uint32_t HWAdvanceFeaturesStatus=0;

/* Private Variables -------------------------------------------------------------*/
static float DefaultAccODR;
static uint8_t MultipleAccEventEnabled= 0;

/**
  * @brief  This function Reads the default Acceleration Output Data Rate
  * @param  None
  * @retval None
  */
void InitHWFeatures(void){
   /* Read the Default Output Data Rate for Accelerometer */
  BSP_ACCELERO_Get_ODR(TargetBoardFeatures.HandleAccSensor,&DefaultAccODR);
}

/**
  * @brief  This function disables all the HW's Features
  * @param  None
  * @retval None
  */
void DisableHWFeatures(void)
{
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    DisableHWPedometer();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    DisableHWFreeFall();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    DisableHWDoubleTap();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    DisableHWSingleTap();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    DisableHWWakeUp();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    DisableHWTilt();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    DisableHWOrientation6D();
  }
}



/**
  * @brief  This function enables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void EnableHWOrientation6D(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Free Fall detection */
  if(BSP_ACCELERO_Enable_6D_Orientation_Ext(TargetBoardFeatures.HandleAccSensor,INT_PIN)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Enabling 6D Orientation\r\n");
  } else {
    SENSING1_PRINTF("Enabled 6D Orientation\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_6DORIENTATION);
  }
}

/**
  * @brief  This function disables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void DisableHWOrientation6D(void)
{
  /* Disable Free Fall detection */
  if(BSP_ACCELERO_Disable_6D_Orientation_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling 6D Orientation\r\n");
  } else {
    SENSING1_PRINTF("Disabled 6D Orientation\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_6DORIENTATION);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function eturns the HW's 6D Orientation result
  * @param  None
  * @retval AccEventType 6D Orientation Found
  */
AccEventType GetHWOrientation6D(void)
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  AccEventType OrientationResult = ACC_NOT_USED;

  if ( BSP_ACCELERO_Get_6D_Orientation_XL_Ext( TargetBoardFeatures.HandleAccSensor, &xl ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error getting 6D orientation XL axis from LSM6DS3\r\n");
  }

  if ( BSP_ACCELERO_Get_6D_Orientation_XH_Ext( TargetBoardFeatures.HandleAccSensor, &xh ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error getting 6D orientation XH axis from LSM6DS3\r\n");
  }

  if ( BSP_ACCELERO_Get_6D_Orientation_YL_Ext( TargetBoardFeatures.HandleAccSensor, &yl ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error getting 6D orientation YL axis from LSM6DS3\r\n");
  }

  if ( BSP_ACCELERO_Get_6D_Orientation_YH_Ext( TargetBoardFeatures.HandleAccSensor, &yh ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error getting 6D orientation YH axis from LSM6DS3\r\n");
  }

  if ( BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( TargetBoardFeatures.HandleAccSensor, &zl ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error getting 6D orientation ZL axis from LSM6DS3\r\n");
  }

  if ( BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( TargetBoardFeatures.HandleAccSensor, &zh ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error getting 6D orientation ZH axis from LSM6DS3\r\n");
  }

  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_RIGTH;
  } else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_TOP;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_BOTTOM;
  } else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_LEFT;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    OrientationResult = ACC_6D_OR_UP;
  } else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ){
    OrientationResult = ACC_6D_OR_DOWN;
  } else {
    SENSING1_PRINTF("None of the 6D orientation axes is set in LSM6DS3\r\n");
  }
  return OrientationResult;
}
/**
  * @brief  This function enables the HW's Tilt Detection
  * @param  None
  * @retval None
  */
void EnableHWTilt(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Tilt detection */
  if(BSP_ACCELERO_Enable_Tilt_Detection_Ext(TargetBoardFeatures.HandleAccSensor,INT_PIN)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Enabling Tilt Detection\r\n");
  } else {
    SENSING1_PRINTF("Enabled Tilt\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_TILT);
  }
}

/**
  * @brief  This function disables the HW's Tilt Detection
  * @param  None
  * @retval None
  */
void DisableHWTilt(void)
{
  /* Disable Tilt detection */
  if(BSP_ACCELERO_Disable_Tilt_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling Tilt Detection\r\n");
  } else {
    SENSING1_PRINTF("Disabled Tilt\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_TILT);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}


/**
  * @brief  This function enables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void EnableHWWakeUp(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Wake up detection */
  if(BSP_ACCELERO_Enable_Wake_Up_Detection_Ext(TargetBoardFeatures.HandleAccSensor,INT_PIN_WAKE_UP)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Enabling Wake Up Detection\r\n");
  } else {
    SENSING1_PRINTF("Enabled Wake Up\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_WAKE_UP);
  }
}

/**
  * @brief  This function disables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void DisableHWWakeUp(void)
{
  /* Disable Wake up detection */
  if(BSP_ACCELERO_Disable_Wake_Up_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling Wake Up Detection\r\n");
  } else {
    SENSING1_PRINTF("Disabled Wake Up\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_WAKE_UP);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void EnableHWFreeFall(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Free Fall detection */
  if(BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(TargetBoardFeatures.HandleAccSensor,INT_PIN)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Enabling Free Fall Detection\r\n");
  } else {
    SENSING1_PRINTF("Enabled Free Fall\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_FREE_FALL);
  }
  
#ifdef STM32_NUCLEO
  if(BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(TargetBoardFeatures.HandleAccSensor,LSM6DSL_ACC_GYRO_FF_THS_250mg)==COMPONENT_ERROR) {
#elif STM32_SENSORTILE
  if(BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(TargetBoardFeatures.HandleAccSensor,LSM6DSM_ACC_GYRO_FF_THS_219mg)==COMPONENT_ERROR) {
#endif /* STM32_NUCLEO */
    SENSING1_PRINTF("Error setting Free Fall Treshold\r\n");
  }
}

/**
  * @brief  This function disables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void DisableHWFreeFall(void)
{
  /* Disable Free Fall detection */
  if(BSP_ACCELERO_Disable_Free_Fall_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling Free Fall Detection\r\n");
  } else {
    SENSING1_PRINTF("Disabled Free Fall\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_FREE_FALL);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWDoubleTap(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Double Tap detection */
  if(BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor,INT_PIN)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Enabling Double Tap Detection\r\n");
  } else {
    SENSING1_PRINTF("Enabled Double Tap\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
  }
  
#ifdef STM32_NUCLEO
  if(BSP_ACCELERO_Set_Tap_Threshold_Ext(TargetBoardFeatures.HandleAccSensor,LSM6DSL_TAP_THRESHOLD_MID)==COMPONENT_ERROR) {
#elif STM32_SENSORTILE
  if(BSP_ACCELERO_Set_Tap_Threshold_Ext(TargetBoardFeatures.HandleAccSensor,LSM6DSM_TAP_THRESHOLD_MID)==COMPONENT_ERROR) {
#endif /* STM32_NUCLEO */
    SENSING1_PRINTF("Error setting Double Tap Treshold\r\n");
  }
}

/**
  * @brief  This function disables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWDoubleTap(void)
{
  /* Disable Double Tap detection */
  if(BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling Double Tap Detection\r\n");
  } else {
    SENSING1_PRINTF("Disabled Double Tap\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWSingleTap(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Single Tap detection */
  if(BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor,INT_PIN)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Enabling Single Tap Detection\r\n");
  } else {
    SENSING1_PRINTF("Enabled Sigle Tap\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
  }
}

/**
  * @brief  This function disables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWSingleTap(void)
{
  /* Disable Single Tap detection */
  if(BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling Single Tap Detection\r\n");
  } else {
    SENSING1_PRINTF("Disabled Sigle Tap\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's pedometer
  * @param  None
  * @retval None
  */
void EnableHWPedometer(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

#ifdef STM32_SENSORTILE
  if(BSP_ACCELERO_Enable_Pedometer_Ext(TargetBoardFeatures.HandleAccSensor, INT_PIN)==COMPONENT_ERROR) {
#else /* STM32_SENSORTILE */
  if(BSP_ACCELERO_Enable_Pedometer_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
#endif /* STM32_SENSORTILE */
    SENSING1_PRINTF("Error Enabling Pedometer\r\n");
  } else {
    SENSING1_PRINTF("Enabled Pedometer\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_PEDOMETER);
  }
}

/**
  * @brief  This function disables the HW's pedometer
  * @param  None
  * @retval None
  */
void DisableHWPedometer(void)
{
  if(BSP_ACCELERO_Disable_Pedometer_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
    SENSING1_PRINTF("Error Disabling Pedometer\r\n");
  } else {
    SENSING1_PRINTF("Disabled Pedometer\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_PEDOMETER);
  }

  /* Set the Output Data Rate to Default value */
  BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function resets the HW's pedometer steps counter
  * @param  None
  * @retval None
  */
void ResetHWPedometer(void)
{
  if ( BSP_ACCELERO_Reset_Step_Counter_Ext( TargetBoardFeatures.HandleAccSensor ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error Resetting Pedometer's Counter\r\n");
  } else {
    SENSING1_PRINTF("Reset Pedometer's Counter\r\n");
  }
}

/**
  * @brief  This function retunrs the HW's pedometer steps counter value
  * @param  None
  * @retval uint16_t Steps Counter
  */
uint16_t GetStepHWPedometer(void)
{
  uint16_t step_count=0;
  if(BSP_ACCELERO_Get_Step_Count_Ext( TargetBoardFeatures.HandleAccSensor, &step_count ) == COMPONENT_ERROR ){
    SENSING1_PRINTF("Error Reading Pedometer's Counter\r\n");
  } else {
    SENSING1_PRINTF("Pedometer's Counter=%u\r\n",step_count);
  }
  return step_count;
}

/**
  * @brief  This function enables the multiple HW's events
  * @param  None
  * @retval None
  */
void EnableHWMultipleEvents(void)
{
  DisableHWFeatures();
  
  MultipleAccEventEnabled= 1;
  
  /* Do not change the enable sequenze of the HW events */
  /* It depends on the ODR value (from minor value to max value) */
  EnableHWPedometer();
  EnableHWTilt();
  EnableHWFreeFall();
  EnableHWSingleTap();
  EnableHWDoubleTap();
  EnableHWOrientation6D();
  
  PedometerStepCount=0;
  AccEvent_Notify(PedometerStepCount, 3);
   
  W2ST_ON_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}

/**
  * @brief  This function disables the multiple HW's events
  * @param  None
  * @retval None
  */
void DisableHWMultipleEvents(void)
{
  DisableHWFeatures();
  MultipleAccEventEnabled= 0;
  W2ST_OFF_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
