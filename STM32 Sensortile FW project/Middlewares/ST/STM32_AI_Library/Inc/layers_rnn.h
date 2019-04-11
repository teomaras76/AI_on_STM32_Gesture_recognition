/**
  ******************************************************************************
  * @file    layers_rnn.h
  * @author  AST Embedded Analytics Research Platform
  * @date    18-May-2018
  * @brief   header file of RNN layers
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

#ifndef __LAYERS_RNN_H_
#define __LAYERS_RNN_H_
#pragma once

#include "layers_common.h"

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_lstm
 * @ingroup layers
 * @brief LSTM layer with generic nonlinearities and peephole connections
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_lstm_ {
  AI_LAYER_COMMON_FIELDS_DECLARE
  ai_size n_units;       /**< size of the hidden RNN state */
  func_nl_el activation_nl;  /**< activation nonlinearity (input to cell) */
  func_nl_el recurrent_nl;   /**< recurrent nonlinearity (hidden to cell) */
  func_nl_el out_nl;    /**< output nonlinearity (cell to hidden) */
  ai_bool go_backwards; /**< process reversed input */
  ai_bool reverse_seq; /**< reverse output sequence */
} ai_layer_lstm;


/*!
 * @struct ai_layer_gru
 * @ingroup layers
 * @brief Gated Recurrent Unit (GRU) layer with generic nonlinearities
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_gru_ {
  AI_LAYER_COMMON_FIELDS_DECLARE
  ai_size n_units;       /**< size of the hidden RNN state */
  func_nl_el activation_nl;  /**< activation nonlinearity (input to cell) */
  func_nl_el recurrent_nl;   /**< recurrent nonlinearity (hidden to cell) */
  ai_bool reset_after;
  ai_bool go_backwards; /**< process reversed input */
  ai_bool reverse_seq; /**< reverse output sequence */
} ai_layer_gru;

/*!
 * @brief Computes the activations of a Long-Short Term Memory (LSTM) layer.
 * @ingroup layers
 *
 * Implements a Long-Short Term Layer with peephole connections:
 * \f{eqnarray*}{
 *    i_t &=& \sigma_a(x_t W_{xi} + h_{t-1} W_{hi}
 *            + w_{ci} \odot c_{t-1} + b_i)\\
 *    f_t &=& \sigma_a(x_t W_{xf} + h_{t-1} W_{hf}
 *            + w_{cf} \odot c_{t-1} + b_f)\\
 *    c_t &=& f_t \odot c_{t - 1}
 *            + i_t \odot \sigma_r(x_t W_{xc} + h_{t-1} W_{hc} + b_c)\\
 *    o_t &=& \sigma_a(x_t W_{xo} + h_{t-1} W_{ho} + w_{co} \odot c_t + b_o)\\
 *    h_t &=& o_t \odot \sigma_o(c_t)
 * \f}
 * where \f$\sigma_a\f$ is the activation nonlinearity, \f$\sigma_r\f$ is the
 * recurrent nonlinearity and \f$\sigma_o\f$ is the out nonlinearity. The
 * \f$W_x\f$, \f$W_h\f$ and \f$W_c\f$ weights are sliced from the kernel,
 * recurrent and peephole weights.
 *
 * @param layer the LSTM layer
 */
AI_INTERNAL_API
void forward_lstm(ai_layer * layer);

/*!
 * @brief Computes the activations of a Gated Recurrent Unit (GRU) layer.
 * @ingroup layers
 *
 * Implements a Gated Recurrent Unit with the formula:
 * \f{eqnarray*}{
 *    r_t &=& \sigma_a(x_t W_{xr} + h_{t - 1} W_{hr} + b_r) \\
 *    z_t &=& \sigma_a(x_t W_{xz} + h_{t - 1} W_{hz} + b_z) \\
 *    c_t &=& \sigma_r(x_t W_{xc} + r_t \odot (h_{t - 1} W_{hc} + b_{hc}) + b_c)
 *            \qquad \textnormal{when reset after is true} \\
 *    c_t &=& \sigma_r(x_t W_{xc} + (r_t \odot h_{t - 1}) W_{hc} + b_{hc} + b_c)
 *            \qquad \textnormal{when reset after is false (default)} \\
 *    h_t &=& (1 - z_t) \odot h_{t - 1} + z_t \odot c_t
 * \f}
 * where \f$\sigma_a\f$ is the activation nonlinearity and \f$\sigma_r\f$ is
 * the recurrent nonlinearity. The weights are sliced from the kernel and
 * recurrent weights.
 *
 * @param layer the GRU layer
 */
AI_INTERNAL_API
void forward_gru(ai_layer * layer);


AI_API_DECLARE_END

#endif /* __LAYERS_RNN_H_ */
