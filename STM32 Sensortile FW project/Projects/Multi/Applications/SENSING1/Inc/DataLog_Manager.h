 /**
 ******************************************************************************
 * @file    DataLog_Manager.h
 * @author  Central Lab
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   Header for DataLog_Manager.c
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
#ifndef _DATA_LOG_MANAGER_H_
#define _DATA_LOG_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported Functions Prototypes ---------------------------------------------*/
extern void SdCardMemsRecordingRun(uint32_t OnlyForAnnotation);
extern void SD_CardLoggingMemsStart(uint32_t OnlyForAnnotation);
extern void SD_CardLoggingMemsStop(void);

extern void SdCardAudioRecordingRun(void);
extern void SD_CardLoggingAudioStart(void);
extern void AudioProcess_SD_Recording(void);
extern void SD_CardLoggingAudioStop(void);

extern void SaveDataAnnotation(uint8_t *Annotation);
extern void DATALOG_SD_Init(void);
extern void DATALOG_SD_DeInit(void);

/* Exported Variables --------------------------------------------------------*/
extern volatile uint8_t writeAudio_flag;
extern uint32_t SD_LogAudio_Enabled;
extern uint32_t SD_LogMems_Enabled;
extern uint32_t SD_Card_FeaturesMask;
/* Data File Name. 12 == 11 Max Chars from BLE + termination char */
extern char DefaultDataFileName[12];
extern uint32_t RoundedInertialWakeUpTimer;
extern uint32_t RoundedEnvironmentalFreq;
extern uint32_t RoundCounterEnvironmental;
extern uint16_t SampleRateIneFeatures;


/* Exported Defines ----------------------------------------------------------*/
#define SD_CARD_LOGGING_STOP     (0)
#define SD_CARD_LOGGING_START    (1)
#define SD_CARD_LOGGING_NO_SD    (2)
#define SD_CARD_LOGGING_IO_ERROR (3)
#define SD_CARD_LOGGING_UPDATE   (4)

#ifdef __cplusplus
}
#endif

#endif /* _DATA_LOG_MANAGER_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

