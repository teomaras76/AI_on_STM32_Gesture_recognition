/**
  ******************************************************************************
  * @file    PowerControl.h 
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Header for main.c module
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
#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
int initPowerController(void);

/* Exported defines and variables  ------------------------------------------------------- */
typedef enum
{
  RUN                   = 0,
  IDLE_WFI              = 1,
  IDLE_WFI_TICK_SUPRESS = 2,
  IDLE_SLEEP_STOP       = 3,
  STAND_BY              = 4,
  SHUTDOWNN             = 5
}powerState_t;

/*exported for IT handler */
extern RTC_HandleTypeDef RtcHandle; 

int  SetMinPowerMode(powerState_t);
powerState_t GetMinPowerMode(void);
int PowerCtrlLock(void);
int PowerCtrlUnLock(void);
int PowerCtrlGetState(void);

#endif /* __POWERCONTROL */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
