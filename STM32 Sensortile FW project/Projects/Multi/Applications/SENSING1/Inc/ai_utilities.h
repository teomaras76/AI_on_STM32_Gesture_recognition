/**
  ******************************************************************************
  * @file    ai_utilities.h 
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Utilities used for AI algorithms
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


#ifndef __AI_UTILITIES_H_
#define __AI_UTILITIES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include "TargetFeatures.h"
#include "ai_platform.h"
#include "network_data.h"


/* Exported macro ------------------------------------------------------------*/
__STATIC_INLINE ai_u32 aiBufferSize(const ai_buffer* buffer)
{
    return buffer->height * buffer->width * buffer->channels;
}

static void aiLogErr(const ai_error err, const char *fct)
{
  if (fct) {
    SENSING1_PRINTF ("E: AI error (%s) - type=%d code=%d\r\n", fct,err.type, err.code);
  } else {
    SENSING1_PRINTF("E: AI error - type=%d code=%d\r\n", err.type, err.code);
  }
}
#ifdef SENSING1_ENABLE_PRINTF
__STATIC_INLINE const char* aiBufferFormatToStr(uint32_t val)
{
    if (val == AI_BUFFER_FORMAT_NONE)
        return "AI_BUFFER_FORMAT_NONE";
    else if (val == AI_BUFFER_FORMAT_FLOAT)
        return "AI_BUFFER_FORMAT_FLOAT";
    else if (val == AI_BUFFER_FORMAT_U8)
        return "AI_BUFFER_FORMAT_U8";
    else if (val == AI_BUFFER_FORMAT_Q15)
        return "AI_BUFFER_FORMAT_Q15";
    else if (val == AI_BUFFER_FORMAT_Q7)
        return "AI_BUFFER_FORMAT_Q7";
    else
        return "UNKNOWN";
}

__STATIC_INLINE void aiPrintLayoutBuffer(const char *msg,
        const ai_buffer* buffer)
{
    SENSING1_PRINTF("%s HWC layout:%d,%d,%ld (s:%ld f:%s)\r\n",
      msg, buffer->height, buffer->width, buffer->channels,
      aiBufferSize(buffer),
      aiBufferFormatToStr(buffer->format));
}

__STATIC_INLINE void aiPrintNetworkInfo(const ai_network_report* report)
{
  SENSING1_PRINTF("Network configuration...\r\n");
  SENSING1_PRINTF(" Model name         : %s\r\n", report->model_name);
  SENSING1_PRINTF(" Model signature    : %s\r\n", report->model_signature);
  SENSING1_PRINTF(" Model datetime     : %s\r\n", report->model_datetime);
  SENSING1_PRINTF(" Compile datetime   : %s\r\n", report->compile_datetime);
  SENSING1_PRINTF(" Runtime revision   : %s (%d.%d.%d)\r\n", report->runtime_revision,
    report->runtime_version.major,
    report->runtime_version.minor,
    report->runtime_version.micro);
  SENSING1_PRINTF(" Tool revision      : %s (%d.%d.%d)\r\n", report->tool_revision,
    report->tool_version.major,
    report->tool_version.minor,
    report->tool_version.micro);
  SENSING1_PRINTF("Network info...\r\n");
  SENSING1_PRINTF("  signature         : 0x%lx\r\n", report->signature);
  SENSING1_PRINTF("  nodes             : %ld\r\n", report->n_nodes);
  SENSING1_PRINTF("  complexity        : %ld MACC\r\n", report->n_macc);
  SENSING1_PRINTF("  activation        : %ld bytes\r\n", aiBufferSize(&report->activations));
  SENSING1_PRINTF("  weights           : %ld bytes\r\n", aiBufferSize(&report->weights));
  SENSING1_PRINTF("  inputs/outputs    : %u/%u\r\n", report->n_inputs, report->n_outputs);
  aiPrintLayoutBuffer("  IN tensor format  :", &report->inputs);
  aiPrintLayoutBuffer("  OUT tensor format :", &report->outputs);
}
#endif /* SENSING1_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif
#endif /* __AI_UTILITIES_H_ */
