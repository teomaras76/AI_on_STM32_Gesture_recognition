/**
  ******************************************************************************
  * @file    PowerControl.c
  * @author  Microcontroller Division Team
  * @version V1.0.0
  * @date    30-Nov-2018  
  * @brief   This file provide the power managing to reduce the power consump
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
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "PowerControl.h"
#include "cmsis_os.h"
#include "TargetFeatures.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RTC_ASYNCH_PREDIV    0x0
#define RTC_SYNCH_PREDIV     0x7FFF  

/* Imported Variables -------------------------------------------------------------*/
/* Imported for IT handler */
extern RTC_HandleTypeDef RtcHandle; 

/* Private variables ---------------------------------------------------------*/


static powerState_t minPowerMode =  RUN;
static int PowerCtrlLockToken =  0;

powerState_t GetMinPowerMode(void)
{
  return minPowerMode;
}

int SetMinPowerMode(powerState_t powerMode)
{
  minPowerMode = powerMode ;
  return 0;
}

int PowerCtrlLock(void)
{
  return ++PowerCtrlLockToken;
}
int PowerCtrlUnLock(void)
{
  PowerCtrlLockToken -= (PowerCtrlLockToken) ? 1:0;
  return PowerCtrlLockToken;
}
int PowerCtrlGetState(void)
{
  return PowerCtrlLockToken;
}

void vApplicationIdleHook( void )
{
   __WFI();
}

int initPowerController(void)
{
    SetMinPowerMode (RUN);

  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess(); 
    
  /* Ensure that MSI is wake-up system clock */ 
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

  /* Configure RTC */
  RtcHandle.Instance = RTC;
  /* Set the RTC time base to 1s */
  /* Configure RTC prescaler and RTC data registers as follow:
    - Hour Format = Format 24
    - Asynch Prediv = Value according to source clock
    - Synch Prediv = Value according to source clock
    - OutPut = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType = Open Drain */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  return 0 ;
}

uint32_t timeInSeconds( uint32_t inBCD)
{
  uint32_t timeOut;
  timeOut  = ((inBCD&RTC_TR_SU_Msk )>>RTC_TR_SU_Pos );         // seconds
  timeOut += ((inBCD&RTC_TR_ST_Msk )>>RTC_TR_ST_Pos )*10;      // tens of seconds
  timeOut += ((inBCD&RTC_TR_MNU_Msk)>>RTC_TR_MNU_Pos)*60;      // minutes 
  timeOut += ((inBCD&RTC_TR_MNT_Msk)>>RTC_TR_MNT_Pos)*60*10;   // tens of minutes 
  timeOut += ((inBCD&RTC_TR_HU_Msk )>>RTC_TR_HU_Pos )*60*60;   // hours 
  timeOut += ((inBCD&RTC_TR_HT_Msk )>>RTC_TR_HT_Pos )*10*60*60;// tens of hours
  timeOut += ((inBCD&RTC_TR_PM_Msk )>>RTC_TR_PM_Pos )*12*60*60;// PM
  return timeOut;
}

void wakeIn( uint32_t waketime)
{
  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);
  if (waketime < 32000)
  {
     /* resolution of 1/2048 = 0,48828125 ms */
    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, (waketime << 11)/1000, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
  }
  else
  {
    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle,0xFFFF, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    /* later, would set ALARM A or B with 1 second resolution instead */
  }
/*  SENSING1_PRINTF("(%d)",waketime);*/
}

extern __IO uint32_t uwTick; // ?? needed

void vPortSuppressTicksAndSleep ( TickType_t xExpectedIdleTime )
{
  eSleepModeStatus eSleepStatus;
  int32_t tickDif,secDif;
  uint32_t secIn,secOut;
  uint32_t tickIn,tickOut;
  uint32_t missedTicks;
  volatile uint32_t DR,TRin,TRout;

#ifdef DISABLE_PM
  minPowerMode = IDLE_WFI;
#endif
  
  if (( minPowerMode < IDLE_WFI_TICK_SUPRESS ) || (PowerCtrlLockToken))
  {
    return;
  }

  tickIn = RTC->SSR; 
  HAL_SuspendTick();
  TRin = RTC->TR & RTC_TR_RESERVED_MASK;
  DR     = RTC->DR & RTC_DR_RESERVED_MASK;; // unlock shadow registers
  UNUSED(DR);

  /* Ensure it is still ok to enter the sleep mode. */
  eSleepStatus = eTaskConfirmSleepModeStatus();

  if( eSleepStatus != eAbortSleep )
  {
    if( eSleepStatus != eNoTasksWaitingTimeout )
    {
        /* It is necessary to configure an interrupt to bring the
        microcontroller out of its low power state at a fixed time in the
        future. */
        /* Configure an interrupt to bring the microcontroller out of its low
        power state at the time the kernel next needs to execute.  The
        interrupt must be generated from a source that remains operational
        when the microcontroller is in a low power state. */
        wakeIn(xExpectedIdleTime-1);
    }
    __asm volatile ( "cpsid i" );
    __asm volatile ( "dsb" );
    __asm volatile ( "isb" );
    if (minPowerMode >= IDLE_SLEEP_STOP ) {
      /* Enter STOP 2 mode */
      HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
      /* ... STOP2 mode ... */
      SystemClock_Config();
    }
    else{
      __WFI();
    }
    __asm volatile ( "cpsie i" );
    __asm volatile ( "dsb" );
    __asm volatile ( "isb" );

  }
  /* Determine how long the microcontroller was actually in a low power
  state for, which will be less than xExpectedIdleTime if the
  microcontroller was brought out of low power mode by an interrupt
  other than that configured by the vSetWakeTimeInterrupt() call.
  Note that the scheduler is suspended before
  portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
  portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
  execute until this function completes. */
  tickOut = RTC->SSR; 
  TRout   = RTC->TR & RTC_TR_RESERVED_MASK;
  DR      = RTC->DR & RTC_DR_RESERVED_MASK; // unlock shadow registers & check day wrap around
  UNUSED(DR);
    
  secIn    = timeInSeconds (TRin);
  secOut   = timeInSeconds (TRout);
  secDif   = secOut - secIn ;
  tickDif  = tickIn - tickOut ; // sub second register is a decreasing counter
  if (secDif)
  {
    secDif  +=( secOut < secIn ) ? 24*60*60  : 0 ;
    tickDif += RTC_SYNCH_PREDIV;
    tickDif += (secDif-1)<<15; 
  }
  missedTicks = (int)(((float)tickDif)/32.768f);
  uwTick += missedTicks;
  /* Correct the kernels tick count to account for the time the
  microcontroller spent in its low power state. */
  vTaskStepTick( missedTicks);

  /* Restart the timer that is generating the tick interrupt. */
  HAL_ResumeTick();
  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);
#ifdef DEBUG_PM
  SENSING1_PRINTF( "zz %d ms\r\n",missedTicks );
#endif
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
