/**
 ******************************************************************************
 * @file    har_Processing.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   This file includes activity recognition interface functions
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
#include "TargetFeatures.h"
#include "gesture_Processing.h"
#include "gesture_Preprocessing.h"
#include "gesture_Postprocessing.h"
#ifdef TEST_IGN_WSDM 
  #include "gesture_ProcessingTest.h"
#endif
#include "ai_platform.h"
#include "ai_utilities.h"
#include "network.h"
#include "network_data.h"

/* Matteo */
#include "main.h"
#include "ff_gen_drv.h"
#include "DataLog_Manager.h"

/* Imported Variable -------------------------------------------------------------*/

/* exported Variable -------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Matteo - changed to 128 sequence */
//#define AI_NETWORK_IN_1_HEIGHT  (24)
#define AI_NETWORK_IN_1_HEIGHT  (128)
#define AI_NETWORK_IN_1_WIDTH   ((ai_int)(AI_NETWORK_IN_1_SIZE/AI_NETWORK_IN_1_HEIGHT))
#define AI_NETWORK_IN_1_FORMAT   AI_BUFFER_FORMAT_FLOAT
#define AI_NETWORK_OUT_1_FORMAT  AI_BUFFER_FORMAT_FLOAT

//#ifdef NN_GMP
//  #define WINDOW_STEP           (16)
//  #define N_OVERLAPPING_WIN     ((ai_int)(AI_NETWORK_IN_1_HEIGHT / WINDOW_STEP) + 1)
//#elif (defined(NN_IGN))
//  #define N_OVERLAPPING_WIN     (1)
//#elif (defined(NN_IGN_WSDM))
//  #define N_OVERLAPPING_WIN     (1)
//#endif

#define WINDOW_STEP           (128)
#define N_OVERLAPPING_WIN     (1)

/* Declaration of network objects from network.h
-> ai_handle is a type void which points a memory space
-> ai_u8 is an unsigned int on 1 byte
*/
AI_ALIGNED(4)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

static ai_handle network = AI_HANDLE_NULL;

static ai_buffer ai_input[AI_NETWORK_IN_NUM]    = { AI_NETWORK_IN_1 };
static ai_buffer ai_output[AI_NETWORK_OUT_NUM]  = { AI_NETWORK_OUT_1 };

static uint8_t last_prediction = AR_ID_NONE;
static Gesture_output_t ActivityCode = Gesture_NOACTIVITY;

static ai_network_report report;

/* handling samples in a sliding window */
static ai_size n_sample = 0;
//#pragma location=0x10000000
static ai_float window_buffer[N_OVERLAPPING_WIN * AI_NETWORK_IN_1_SIZE] = {0}; 


extern char mySDPath[4];
extern char AI_file_name[64];
extern FIL MyFileAI;
extern char myPath[];
extern char *MonthName[];



// List of all Gestures to be added later
//#define AR_ID_STILL             (uint8_t)(0x01)
//#define AR_ID_ROUND_CCW         (uint8_t)(0x02)
//#define AR_ID_ROUND_CW          (uint8_t)(0x03)
//#define AR_ID_CROSS_RIGHT       (uint8_t)(0x04)
//#define AR_ID_CROSS_LEFT        (uint8_t)(0x05)
//#define AR_ID_RIGHT             (uint8_t)(0x06)
//#define AR_ID_LEFT              (uint8_t)(0x07)
//#define AR_ID_UP                (uint8_t)(0x00)
//#define AR_ID_DOWN              (uint8_t)(0x01)
__STATIC_INLINE Gesture_output_t map2GestureClasses(uint8_t prediction)
{
  switch(prediction){
    case(0x00): return (Gesture_NOACTIVITY);
    case(AR_ID_STILL): return (Gesture_STATIONARY);
    case(AR_ID_ROUND_CCW)   : return (Gesture_WALKING);
    case(AR_ID_ROUND_CW)   : return (Gesture_JOGGING);
    case(AR_ID_CROSS_RIGHT)   : return (Gesture_DRIVING);
    case(AR_ID_CROSS_LEFT)   : return (Gesture_BIKING);
    case(AR_ID_RIGHT)   : return (Gesture_NOACTIVITY);
    case(AR_ID_LEFT)   : return (Gesture_NOACTIVITY);
    case(AR_ID_UP)   : return (Gesture_NOACTIVITY);
    case(AR_ID_DOWN)   : return (Gesture_NOACTIVITY);

//#if defined(NN_IGN_WSDM)
//    case(AR_ID_CROSS_RIGHT)    : return (Gesture_STAIRS);
//#else
//    case(AR_ID_CROSS_LEFT)    : return (Gesture_BIKING);
//    case(AR_ID_RIGHT)   : return (Gesture_DRIVING);
//#endif    
    default:return (Gesture_NOACTIVITY);
  }
}

__STATIC_INLINE int aiCheckNetwork(const ai_network_report* report)
{
    if (!report)
        return -1;

    if (aiBufferSize(&report->activations)
            != AI_NETWORK_DATA_ACTIVATIONS_SIZE) {
        SENSING1_PRINTF("E: defined activation buffer size is not coherent (expected=%d)\r\n",
                AI_NETWORK_DATA_ACTIVATIONS_SIZE);
        return -1;
    }

    if (aiBufferSize(&report->weights)
            != AI_NETWORK_DATA_WEIGHTS_SIZE) {
        SENSING1_PRINTF("E: defined weights buffer size is not coherent (expected=%d)\r\n",
                AI_NETWORK_DATA_WEIGHTS_SIZE);
        return -1;
    }

    if ((AI_NETWORK_IN_NUM != report->n_inputs) ||
            (AI_NETWORK_OUT_NUM != report->n_outputs) ||
            (report->n_inputs != 1) || (report->n_outputs != 1)) {
        SENSING1_PRINTF("E: only one input and one output is supported\r\n");
        return -1;
    }

    if ((ai_input[0].format != AI_NETWORK_IN_1_FORMAT) ||
            (ai_output[0].format != AI_NETWORK_OUT_1_FORMAT)) {
        SENSING1_PRINTF("E: input or output format unconsistancy\r\n");

        return -1;
    }

    if (AI_NETWORK_IN_1_WIDTH != ai_input[0].width){
        SENSING1_PRINTF("E: input width unconsistancy\r\n");
        return -1;
    }

    if (AI_NETWORK_IN_1_HEIGHT != ai_input[0].height){
        SENSING1_PRINTF("E: input height unconsistancy\r\n");
        return -1;
    }
 
    return 0;
}

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run activity recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Activity Recognition Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/

/* Matteo - Buffer for NN input for datalog purpose */
char NN_input_buffer[6000];             // 128 samples, 6 axis, 6 characters for one value >> buffer of 4608

#ifdef NN_GMP
Gesture_output_t Gesture_run(SensorAxes_t ACC_Value, SensorAxes_t GYR_Value)
{
  static ai_float out[AI_NETWORK_OUT_1_SIZE];

  ai_i32 batch;
  Gesture_input_t iDataIN;
  //Gesture_input_t iDataInPreProc;
  int debug_check = 0;

  if (AI_HANDLE_NULL == network) {
      SENSING1_PRINTF("E: network handle is NULL\r\n");
      return Gesture_NOACTIVITY;
  }
  
///* Matteo */
//#ifdef AI_LOG_INFERENCE
//  //char NN_input_buffer[]="";
//  uint32_t Pos=0;
//  //int i=0;
//  int j=0;
//#endif
  
/* Matteo - Changed scale for acc from G to mG, Gyro in mdps */
//  iDataIN.AccX = (float)ACC_Value_Raw.AXIS_X * TargetBoardFeatures.AccSensiMultInG;
//  iDataIN.AccY = (float)ACC_Value_Raw.AXIS_Y * TargetBoardFeatures.AccSensiMultInG;
//  iDataIN.AccZ = (float)ACC_Value_Raw.AXIS_Z * TargetBoardFeatures.AccSensiMultInG;
  iDataIN.AccX = (float)ACC_Value.AXIS_X;
  iDataIN.AccY = (float)ACC_Value.AXIS_Y;
  iDataIN.AccZ = (float)ACC_Value.AXIS_Z;
  iDataIN.GyrX = (float)GYR_Value.AXIS_X;
  iDataIN.GyrY = (float)GYR_Value.AXIS_Y;
  iDataIN.GyrZ = (float)GYR_Value.AXIS_Z;
  
  // Matteo - No need to rotate gravity for gesture
  //iDataInPreProc = gravity_rotate(&iDataIN);
  
  /* add samples to each active window */
  ai_size n_window = n_sample / WINDOW_STEP, pos = n_sample % WINDOW_STEP;
  for (ai_size i = 0; i < N_OVERLAPPING_WIN; ++i) {
    /* avoid partial buffers at start */
    if (n_window < i) continue;

    ai_int win_idx = (n_window - i) % N_OVERLAPPING_WIN;
    ai_int index = pos + i * WINDOW_STEP;
    ai_int win_offset = win_idx * AI_NETWORK_IN_1_SIZE;

    if (index < AI_NETWORK_IN_1_HEIGHT) {
      ai_size j = win_offset + index * AI_NETWORK_IN_1_WIDTH ;
      //window_buffer[j++] = iDataInPreProc.AccX;
      //window_buffer[j++] = iDataInPreProc.AccY;
      //window_buffer[j]   = iDataInPreProc.AccZ;
      /* Matteo - added Gyro data to buffer */
      window_buffer[j++] = iDataIN.AccX;
      window_buffer[j++] = iDataIN.AccY;
      window_buffer[j++] = iDataIN.AccZ;
      window_buffer[j++] = iDataIN.GyrX;
      window_buffer[j++] = iDataIN.GyrY;
      window_buffer[j]   = iDataIN.GyrZ;
    }

    /* if buffer is full, run the network */
    if (index == (AI_NETWORK_IN_1_HEIGHT - 1)) {
      ai_input[0].data  = AI_HANDLE_PTR(&window_buffer[win_offset]);
      ai_output[0].data = AI_HANDLE_PTR(out);
      /* Matteo */
      LedToggleTargetPlatform();
      batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
      if (batch != 1) {
        aiLogErr(ai_network_get_error(network),"ai_network_run");
      }
      last_prediction = gesture_postProc(out);
      debug_check++; 
      /* Matteo */
      LedToggleTargetPlatform();
      ActivityCode = map2GestureClasses(last_prediction);
      #ifdef AI_LOG_INFERENCE
      /* Matteo */
      uint32_t Pos=0;
      int k=0;
      int j=0;
        for (k = 0; k < 128; ++k){
          for (j = 0; j < 6; ++j){
            //Pos += sprintf(Pos+NN_input_buffer, "%02d,", ((uint32_t*) (ai_input[0].data))[6*k+j]);
            Pos += sprintf(Pos+NN_input_buffer, "%5.0f,", ((float*) (ai_input[0].data))[6*k+j]);
          }
          //Pos += sprintf(Pos+NN_input_buffer+Pos,"%c",'\n');
        }
        //SD_CardLogging_AI_inference(NN_input_buffer, Pos, last_prediction);
      #endif
      //LedToggleTargetPlatform();
    }
  }
  ++n_sample;
  
  //ActivityCode = map2GestureClasses(last_prediction);
  
    
  return ActivityCode;
}
//#elif (defined (NN_IGN) || defined(NN_IGN_WSDM))
//Gesture_output_t Gesture_run(SensorAxesRaw_t ACC_Value_Raw, SensorAxesRaw_t GYR_Value_Ra)
//{
//  static ai_float out[AI_NETWORK_OUT_1_SIZE];
//  ai_i32 batch;
//  HAR_input_t iDataIN;
//  HAR_input_t iDataInPreProc;
//  
//  if (AI_HANDLE_NULL == network) {
//      SENSING1_PRINTF("E: network handle is NULL\r\n");
//      return HAR_NOACTIVITY;
//  }
//  
//#ifdef TEST_IGN_WSDM  
//  Gesture_GetTestSamples(&iDataIN);
//#else
//  {
//    float factor = TargetBoardFeatures.AccSensiMultInG;
//#ifdef NN_IGN_WSDM     
//    factor *= FROM_G_TO_MS_2 ;
//#endif     
//    iDataIN.AccX = (float)ACC_Value_Raw.AXIS_X * factor;
//    iDataIN.AccY = (float)ACC_Value_Raw.AXIS_Y * factor;
//    iDataIN.AccZ = (float)ACC_Value_Raw.AXIS_Z * factor;
//  }
//#endif  
//
//  /* preprocessing */ 
//  iDataInPreProc = gravity_rotate(&iDataIN);
//  
//  /* add samples to each active window */
//  window_buffer[n_sample++] = iDataInPreProc.AccX;
//  window_buffer[n_sample++] = iDataInPreProc.AccY;
//  window_buffer[n_sample++] = iDataInPreProc.AccZ;
//  
//  if  ( n_sample >= AI_NETWORK_IN_1_SIZE )
//  {
//    ai_input[0].data  = AI_HANDLE_PTR(&window_buffer);
//    ai_output[0].data = AI_HANDLE_PTR(out);
//    batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
//    if (batch != 1) {
//        aiLogErr(ai_network_get_error(network),"ai_network_run");
//    }
//    last_prediction = gesture_postProc(out);
//    n_sample         = 0;
//  }
//  ActivityCode = map2GestureClasses(last_prediction);
//  
//  return ActivityCode;
//
//}
#endif
/**
* @brief  Initialises MotionAR algorithm
* @param  None
* @retval 0 if initilazed OK, a negative value otherwise
*/

int8_t Gesture_Initialize(void)
{
  ai_error err;
  if (network != AI_HANDLE_NULL){
	SENSING1_PRINTF("\r\nAI Network already initialized...\r\n");
	return -1;
  }

  last_prediction = AR_ID_NONE;

  SENSING1_PRINTF("\r\nAI Network (AI platform API %d.%d.%d)...\r\n",
          AI_PLATFORM_API_MAJOR,
          AI_PLATFORM_API_MINOR,
          AI_PLATFORM_API_MICRO);

  /* enabling CRC clock for using AI libraries (for checking if STM32 
  microprocessor is used)*/
  __HAL_RCC_CRC_CLK_ENABLE();
    
  /* create an instance of the network */
  SENSING1_PRINTF("Creating the network...\r\n");
  err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  if (err.type) {
      aiLogErr(err, "ai_network_create");
      return -3;
  }

  /* Query the created network to get relevant info from it */
  if (ai_network_get_info(network, &report) ) {
#ifdef SENSING1_ENABLE_PRINTF
      aiPrintNetworkInfo(&report);
#endif
  } else {
      err = ai_network_get_error(network);
      aiLogErr(err, "ai_network_get_info");
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return -4;
  }

  if (aiCheckNetwork(&report))
  {
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return -5;
  }

  /* initialize the instance */
  SENSING1_PRINTF("Initializing the network...\r\n");

  /* build params structure to provide the references of the
   * activation and weight buffers */
  
  const ai_network_params params = {
     AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
     AI_NETWORK_DATA_ACTIVATIONS(activations) };

  if (!ai_network_init(network, &params)) {
      err = ai_network_get_error(network);
      aiLogErr(err, "ai_network_init");
      ai_network_destroy(&network);
      network = AI_HANDLE_NULL;
      return -6;
  }
  ActivityCode = Gesture_NOACTIVITY;

#ifdef NN_GMP
  SENSING1_PRINTF("Initialized NN_GMP HAR\r\n");
#elif defined (NN_IGN)
  SENSING1_PRINTF("Initialized NN_IGN HAR\r\n");
#elif defined (NN_IGN_WSDM)
  #ifdef TEST_IGN_WSDM  
  HAR_GetTestSamples_Init();
  #endif
  SENSING1_PRINTF("Initialized NN_IGN_WSDM HAR\r\n");
#endif  

    return 0;
}
void Gesture_DeInitialize(void)
{
    ai_error err;

    SENSING1_PRINTF("Releasing the network...\r\n");
    if (network == AI_HANDLE_NULL)
        return;
    if (ai_network_destroy(network) != AI_HANDLE_NULL) {
        err = ai_network_get_error(network);
        aiLogErr(err, "ai_network_destroy");
    }
    network = AI_HANDLE_NULL;
    __HAL_RCC_CRC_CLK_DISABLE();

#ifdef TEST_IGN_WSDM  
   HAR_GetTestSamples_DeInit();
#endif

}

/**
 * @brief  get latest activity code computed by Recognition Algorithm
 * @param  None
 * @retval activity index
 */
Gesture_output_t Gesture_get_Activity_Code(void)
{
  return ActivityCode;
}

#ifdef AI_LOG_INFERENCE
static void SD_CardLogging_AI_inference(char* NN_input, uint32_t NN_input_size, Gesture_output_t Y_log)
{


  char myBuffer[256]="";
  uint32_t CharPos=0;
  //static char AI_file_name[64];
  //char Log[2];
  uint32_t byteswritten;
  //static char *MonthName[]={"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
//  FIL MyFileAI;
//  FATFS myFATAFS;
//  char mySDPath[4];
  
  
  
//  sprintf(AI_file_name, "AI_inference_test");
  RTC_GetCurrentDateTime();
//  sprintf(AI_file_name, "%s-Ann_%02d_%s_%02d_%02dh_%02dm_%02ds.csv",
//                       AI_file_name,
//                       CurrentDate.Date,
//                       MonthName[CurrentDate.Month-1],
//                       CurrentDate.Year,
//                       CurrentTime.Hours,
//                       CurrentTime.Minutes,
//                       CurrentTime.Seconds);
  //CharPos = sprintf(myBuffer,"%c",'\n');
  CharPos = sprintf(myBuffer, "%02d:%02d:%02d.%03ld,",
                         CurrentTime.Hours,
                         CurrentTime.Minutes,
                         CurrentTime.Seconds,
                         999- (CurrentTime.SubSeconds*1000)/(CurrentTime.SecondFraction));
  CharPos += sprintf(myBuffer+CharPos,"Output,");
  CharPos += sprintf(myBuffer+CharPos,"%02d",
                         Y_log);
  CharPos += sprintf(myBuffer+CharPos,"%c",'\n');
  //CharPos += sprintf(myBuffer+CharPos,"NN input buffer,");
  //CharPos += sprintf(myBuffer+CharPos,"%c",'\n');
  //if (f_mount(&myFATAFS,mySDPath,1)==FR_OK){
  //char myPath[] = "Test_AI_log.csv\0";
  //f_open(&MyFileAI, AI_file_name, FA_CREATE_ALWAYS | FA_WRITE);
  //f_open(&MyFileAI, myPath, FA_OPEN_APPEND);
  //char myDataTest[] = "Hello World!\0";
  //f_write(&MyFileAI, myBuffer, sizeof(myBuffer), &byteswritten);
  //sprintf(Log, "%02d", Y_log);
  f_write(&MyFileAI, myBuffer, CharPos, &byteswritten);  
  f_write(&MyFileAI, NN_input, NN_input_size, &byteswritten);
  
  CharPos = sprintf(myBuffer,"%c",'\n');
  f_write(&MyFileAI, myBuffer, CharPos, &byteswritten); 
  
  f_sync(&MyFileAI);
  HAL_Delay(1000);
  
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
