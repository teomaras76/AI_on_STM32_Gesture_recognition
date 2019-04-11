/**
 ******************************************************************************
 * @file    asc_preprocessing.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    30-Nov-2018
 * @brief   Preprocessing for Audio Scene Classification algorithm
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
#include "asc_preprocessing.h"
#include "mel_filters_lut_30.h"
#include "asym_hann_win_1024.h"


/* Private variables ---------------------------------------------------------*/
static arm_rfft_fast_instance_f32 SFast;


/**
 * @brief      LogMel Spectrum Calculation when all columns are populated
 * @param      pSpectrogram  Mel-scaled power spectrogram
 * @retval     none
 */
void ASC_LogMelSpectrogram(float32_t *pSpectrogram)
{
  float32_t max_mel_energy = 0.0f;
  uint32_t i;

  /* Find MelEnergy Scaling factor */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    max_mel_energy = (max_mel_energy > pSpectrogram[i]) ? max_mel_energy : pSpectrogram[i];
  }

  /* Scale Mel Energies */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    pSpectrogram[i] /= max_mel_energy;
  }

  /* Convert power spectrogram to decibel */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    pSpectrogram[i] = 10.0f * log10f(pSpectrogram[i]);
  }

  /* Threshold output to -80.0 dB */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    pSpectrogram[i] = (pSpectrogram[i] < -80.0f) ? (-80.0f) : (pSpectrogram[i]);
  }
}

/**
 * @brief      Compute a mel-scaled spectrogram column
 * @param      pInSignal        Input signal (1024 audio samples)
 * @param[in]  col              Spectrogram column index
 * @param      pOutSpectrogram  Log Mel spectrogram (30x32)
 * @retval     none
 */
void ASC_MelColumn(float32_t *pInSignal, uint32_t col, float32_t *pOutSpectrogram)
{
  uint32_t i;
  uint32_t j;
  uint16_t mel_offset = 0;
  uint16_t startIdx, stopIdx;

  /* @note large buffers declared as static to avoid stack usage */
  static float32_t buffer[PROC_BUFFER_SIZE];
  static float32_t pwr_spectr[PROC_BUFFER_SIZE / 2];

  /* Apply Hann window */
  /* @note: OK to typecast because hannWin content is not modified */
  arm_mult_f32(pInSignal, (float32_t *) hannWin, pInSignal, PROC_BUFFER_SIZE);

  /* FFT */
  arm_rfft_fast_init_f32(&SFast, (uint16_t) PROC_BUFFER_SIZE);
  arm_rfft_fast_f32(&SFast, pInSignal, buffer, 0);

  /* Power Spectrum */
  arm_cmplx_mag_squared_f32(buffer, pwr_spectr, PROC_BUFFER_SIZE  / 2);

  /* Mel Filter Banks Application */
  memset(buffer, 0, NMELS * sizeof(buffer[0]));
  for (i = 0 ; i < NMELS; i ++) {
    startIdx = melFiltersStartIndices[i];
    stopIdx = melFiltersStopIndices[i];
    for (j = startIdx ; j <= stopIdx ; j++) {
      buffer[i] += pwr_spectr[j] * (*(melFilterLut + mel_offset));
      mel_offset++;
    }
  }

  /* Reshape and copy into output spectrogram column */
  for (i = 0; i < NMELS; i++) {
    pOutSpectrogram[i * SPECTROGRAM_COLS + col] = buffer[i];
  }

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
