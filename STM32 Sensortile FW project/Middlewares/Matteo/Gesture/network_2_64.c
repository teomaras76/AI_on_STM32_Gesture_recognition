/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Mar 15 17:32:41 2019
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */



#include "network.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "layers.h"

#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#define AI_TOOLS_VERSION_MAJOR 3
#define AI_TOOLS_VERSION_MINOR 3
#define AI_TOOLS_VERSION_MICRO 0

#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 1
#define AI_TOOLS_API_VERSION_MICRO 0

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "4b6b62085c5ed0d18e0dfeb04a824e62"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     "(rev-)"
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Fri Mar 15 17:32:41 2019"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

/**  Forward network declaration section  *************************************/
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/**  Forward network arrays declarations  *************************************/
AI_STATIC ai_array input_0_output_array;   /* Array #0 */
AI_STATIC ai_array conv2d_1_output_array;   /* Array #1 */
AI_STATIC ai_array conv2d_2_output_array;   /* Array #2 */
AI_STATIC ai_array conv2d_3_output_array;   /* Array #3 */
AI_STATIC ai_array conv2d_4_output_array;   /* Array #4 */
AI_STATIC ai_array conv2d_5_output_array;   /* Array #5 */
AI_STATIC ai_array conv2d_6_output_array;   /* Array #6 */
AI_STATIC ai_array conv2d_7_output_array;   /* Array #7 */
AI_STATIC ai_array permute_1_output_in_array;   /* Array #8 */
AI_STATIC ai_array permute_1_output_out_array;   /* Array #9 */
AI_STATIC ai_array batch_normalization_8_output_array;   /* Array #10 */
AI_STATIC ai_array gru_1_output_array;   /* Array #11 */
AI_STATIC ai_array batch_normalization_9_output_array;   /* Array #12 */
AI_STATIC ai_array gru_2_output_array;   /* Array #13 */
AI_STATIC ai_array batch_normalization_10_output_array;   /* Array #14 */
AI_STATIC ai_array gru_3_output_array;   /* Array #15 */
AI_STATIC ai_array batch_normalization_11_output_array;   /* Array #16 */
AI_STATIC ai_array gru_4_output_array;   /* Array #17 */
AI_STATIC ai_array batch_normalization_12_output_array;   /* Array #18 */
AI_STATIC ai_array dense_1_output_array;   /* Array #19 */
AI_STATIC ai_array dense_1_nl_output_array;   /* Array #20 */


/**  Forward network tensors declarations  ************************************/
AI_STATIC ai_tensor input_0_output;   /* Tensor #0 */
AI_STATIC ai_tensor conv2d_1_output;   /* Tensor #1 */
AI_STATIC ai_tensor conv2d_2_output;   /* Tensor #2 */
AI_STATIC ai_tensor conv2d_3_output;   /* Tensor #3 */
AI_STATIC ai_tensor conv2d_4_output;   /* Tensor #4 */
AI_STATIC ai_tensor conv2d_5_output;   /* Tensor #5 */
AI_STATIC ai_tensor conv2d_6_output;   /* Tensor #6 */
AI_STATIC ai_tensor conv2d_7_output;   /* Tensor #7 */
AI_STATIC ai_tensor permute_1_output_in;   /* Tensor #8 */
AI_STATIC ai_tensor permute_1_output_out;   /* Tensor #9 */
AI_STATIC ai_tensor batch_normalization_8_output;   /* Tensor #10 */
AI_STATIC ai_tensor gru_1_output;   /* Tensor #11 */
AI_STATIC ai_tensor batch_normalization_9_output;   /* Tensor #12 */
AI_STATIC ai_tensor gru_2_output;   /* Tensor #13 */
AI_STATIC ai_tensor batch_normalization_10_output;   /* Tensor #14 */
AI_STATIC ai_tensor gru_3_output;   /* Tensor #15 */
AI_STATIC ai_tensor batch_normalization_11_output;   /* Tensor #16 */
AI_STATIC ai_tensor gru_4_output;   /* Tensor #17 */
AI_STATIC ai_tensor batch_normalization_12_output;   /* Tensor #18 */
AI_STATIC ai_tensor dense_1_output;   /* Tensor #19 */
AI_STATIC ai_tensor dense_1_nl_output;   /* Tensor #20 */


/**  Forward network tensor chain declarations  *******************************/
AI_STATIC_CONST ai_tensor_chain conv2d_1_chain;   /* Chain #0 */
AI_STATIC_CONST ai_tensor_chain conv2d_2_chain;   /* Chain #1 */
AI_STATIC_CONST ai_tensor_chain conv2d_3_chain;   /* Chain #2 */
AI_STATIC_CONST ai_tensor_chain conv2d_4_chain;   /* Chain #3 */
AI_STATIC_CONST ai_tensor_chain conv2d_5_chain;   /* Chain #4 */
AI_STATIC_CONST ai_tensor_chain conv2d_6_chain;   /* Chain #5 */
AI_STATIC_CONST ai_tensor_chain conv2d_7_chain;   /* Chain #6 */
AI_STATIC_CONST ai_tensor_chain permute_1_chain;   /* Chain #7 */
AI_STATIC_CONST ai_tensor_chain batch_normalization_8_chain;   /* Chain #8 */
AI_STATIC_CONST ai_tensor_chain gru_1_chain;   /* Chain #9 */
AI_STATIC_CONST ai_tensor_chain batch_normalization_9_chain;   /* Chain #10 */
AI_STATIC_CONST ai_tensor_chain gru_2_chain;   /* Chain #11 */
AI_STATIC_CONST ai_tensor_chain batch_normalization_10_chain;   /* Chain #12 */
AI_STATIC_CONST ai_tensor_chain gru_3_chain;   /* Chain #13 */
AI_STATIC_CONST ai_tensor_chain batch_normalization_11_chain;   /* Chain #14 */
AI_STATIC_CONST ai_tensor_chain gru_4_chain;   /* Chain #15 */
AI_STATIC_CONST ai_tensor_chain batch_normalization_12_chain;   /* Chain #16 */
AI_STATIC_CONST ai_tensor_chain dense_1_chain;   /* Chain #17 */
AI_STATIC_CONST ai_tensor_chain dense_1_nl_chain;   /* Chain #18 */


/**  Subgraph network operators tensor chain declarations  *********************************/


/**  Subgraph network operators declarations  *********************************/


/**  Forward network layers declarations  *************************************/
AI_STATIC ai_layer_conv2d conv2d_1_layer; /* Layer #0 */
AI_STATIC ai_layer_conv2d conv2d_2_layer; /* Layer #1 */
AI_STATIC ai_layer_conv2d conv2d_3_layer; /* Layer #2 */
AI_STATIC ai_layer_conv2d conv2d_4_layer; /* Layer #3 */
AI_STATIC ai_layer_conv2d conv2d_5_layer; /* Layer #4 */
AI_STATIC ai_layer_conv2d conv2d_6_layer; /* Layer #5 */
AI_STATIC ai_layer_conv2d conv2d_7_layer; /* Layer #6 */
AI_STATIC ai_layer_permute permute_1_layer; /* Layer #7 */
AI_STATIC ai_layer_bn batch_normalization_8_layer; /* Layer #8 */
AI_STATIC ai_layer_gru gru_1_layer; /* Layer #9 */
AI_STATIC ai_layer_bn batch_normalization_9_layer; /* Layer #10 */
AI_STATIC ai_layer_gru gru_2_layer; /* Layer #11 */
AI_STATIC ai_layer_bn batch_normalization_10_layer; /* Layer #12 */
AI_STATIC ai_layer_gru gru_3_layer; /* Layer #13 */
AI_STATIC ai_layer_bn batch_normalization_11_layer; /* Layer #14 */
AI_STATIC ai_layer_gru gru_4_layer; /* Layer #15 */
AI_STATIC ai_layer_bn batch_normalization_12_layer; /* Layer #16 */
AI_STATIC ai_layer_dense dense_1_layer; /* Layer #17 */
AI_STATIC ai_layer_sm dense_1_nl_layer; /* Layer #18 */


/**  Arrays declarations section  *********************************************/
AI_ARRAY_OBJ_DECLARE(
  input_0_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 768,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 4608,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_2_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 9216,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 9216,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 9216,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 9216,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 6144,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_7_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 6144,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  permute_1_output_in_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 6144,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  permute_1_output_out_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 6144,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_8_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 6144,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  gru_1_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1024,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_9_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1024,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  gru_2_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1024,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_10_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1024,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  gru_3_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1024,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_11_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1024,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  gru_4_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_12_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  dense_1_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 10,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  dense_1_nl_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 10,
  AI_STATIC)


/**  Activations tensors declaration section  *********************************/
AI_TENSOR_OBJ_DECLARE(
  input_0_output,
  AI_SHAPE_INIT(6, 128, 1, 1),
  AI_STRIDE_INIT(512, 4, 4, 4),
  &input_0_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_output,
  AI_SHAPE_INIT(6, 64, 12, 1),
  AI_STRIDE_INIT(3072, 48, 4, 4),
  &conv2d_1_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_2_output,
  AI_SHAPE_INIT(6, 64, 24, 1),
  AI_STRIDE_INIT(6144, 96, 4, 4),
  &conv2d_2_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_3_output,
  AI_SHAPE_INIT(6, 64, 24, 1),
  AI_STRIDE_INIT(6144, 96, 4, 4),
  &conv2d_3_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_4_output,
  AI_SHAPE_INIT(6, 32, 48, 1),
  AI_STRIDE_INIT(6144, 192, 4, 4),
  &conv2d_4_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_5_output,
  AI_SHAPE_INIT(6, 32, 48, 1),
  AI_STRIDE_INIT(6144, 192, 4, 4),
  &conv2d_5_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_6_output,
  AI_SHAPE_INIT(6, 16, 64, 1),
  AI_STRIDE_INIT(4096, 256, 4, 4),
  &conv2d_6_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_7_output,
  AI_SHAPE_INIT(6, 16, 64, 1),
  AI_STRIDE_INIT(4096, 256, 4, 4),
  &conv2d_7_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  permute_1_output_in,
  AI_SHAPE_INIT(16, 6, 64, 1),
  AI_STRIDE_INIT(1536, 256, 4, 4),
  &permute_1_output_in_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  permute_1_output_out,
  AI_SHAPE_INIT(16, 1, 384, 1),
  AI_STRIDE_INIT(1536, 1536, 4, 4),
  &permute_1_output_out_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  batch_normalization_8_output,
  AI_SHAPE_INIT(16, 1, 384, 1),
  AI_STRIDE_INIT(1536, 1536, 4, 4),
  &batch_normalization_8_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  gru_1_output,
  AI_SHAPE_INIT(16, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &gru_1_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  batch_normalization_9_output,
  AI_SHAPE_INIT(16, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_9_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  gru_2_output,
  AI_SHAPE_INIT(16, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &gru_2_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  batch_normalization_10_output,
  AI_SHAPE_INIT(16, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_10_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  gru_3_output,
  AI_SHAPE_INIT(16, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &gru_3_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  batch_normalization_11_output,
  AI_SHAPE_INIT(16, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_11_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  gru_4_output,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &gru_4_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  batch_normalization_12_output,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_12_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_1_output,
  AI_SHAPE_INIT(1, 1, 10, 1),
  AI_STRIDE_INIT(40, 40, 4, 4),
  &dense_1_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_1_nl_output,
  AI_SHAPE_INIT(1, 1, 10, 1),
  AI_STRIDE_INIT(40, 40, 4, 4),
  &dense_1_nl_output_array,
  AI_STATIC)





/* Layer #0: "conv2d_1" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 60,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_1_weights,
  AI_SHAPE_INIT(12, 1, 5, 1),
  AI_STRIDE_INIT(20, 20, 4, 4),
  &conv2d_1_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_1_bias,
  AI_SHAPE_INIT(1, 1, 12, 1),
  AI_STRIDE_INIT(48, 48, 4, 4),
  &conv2d_1_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&input_0_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_1_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_1_weights, &conv2d_1_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_1_layer, 0,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_2_layer, AI_STATIC,
  .tensors = &conv2d_1_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 1), 
)

/* Layer #1: "conv2d_2" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_2_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 864,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_2_weights,
  AI_SHAPE_INIT(24, 1, 3, 12),
  AI_STRIDE_INIT(144, 144, 48, 4),
  &conv2d_2_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_2_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 24,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_2_bias,
  AI_SHAPE_INIT(1, 1, 24, 1),
  AI_STRIDE_INIT(96, 96, 4, 4),
  &conv2d_2_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_2_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_1_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_2_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_2_weights, &conv2d_2_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_2_layer, 3,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_3_layer, AI_STATIC,
  .tensors = &conv2d_2_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 1), 
)

/* Layer #2: "conv2d_3" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1728,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_3_weights,
  AI_SHAPE_INIT(24, 1, 3, 24),
  AI_STRIDE_INIT(288, 288, 96, 4),
  &conv2d_3_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_3_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 24,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_3_bias,
  AI_SHAPE_INIT(1, 1, 24, 1),
  AI_STRIDE_INIT(96, 96, 4, 4),
  &conv2d_3_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_3_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_2_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_3_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_3_weights, &conv2d_3_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_3_layer, 6,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_4_layer, AI_STATIC,
  .tensors = &conv2d_3_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 1), 
)

/* Layer #3: "conv2d_4" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 3456,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_4_weights,
  AI_SHAPE_INIT(48, 1, 3, 24),
  AI_STRIDE_INIT(288, 288, 96, 4),
  &conv2d_4_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_4_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 48,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_4_bias,
  AI_SHAPE_INIT(1, 1, 48, 1),
  AI_STRIDE_INIT(192, 192, 4, 4),
  &conv2d_4_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_4_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_3_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_4_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_4_weights, &conv2d_4_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_4_layer, 9,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_5_layer, AI_STATIC,
  .tensors = &conv2d_4_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
)

/* Layer #4: "conv2d_5" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 6912,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_5_weights,
  AI_SHAPE_INIT(48, 1, 3, 48),
  AI_STRIDE_INIT(576, 576, 192, 4),
  &conv2d_5_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_5_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 48,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_5_bias,
  AI_SHAPE_INIT(1, 1, 48, 1),
  AI_STRIDE_INIT(192, 192, 4, 4),
  &conv2d_5_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_5_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_4_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_5_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_5_weights, &conv2d_5_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_5_layer, 12,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_6_layer, AI_STATIC,
  .tensors = &conv2d_5_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 1), 
)

/* Layer #5: "conv2d_6" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 9216,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_6_weights,
  AI_SHAPE_INIT(64, 1, 3, 48),
  AI_STRIDE_INIT(576, 576, 192, 4),
  &conv2d_6_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_6_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_6_bias,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &conv2d_6_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_6_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_5_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_6_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_6_weights, &conv2d_6_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_6_layer, 15,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &conv2d_7_layer, AI_STATIC,
  .tensors = &conv2d_6_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
)

/* Layer #6: "conv2d_7" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_7_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_7_weights,
  AI_SHAPE_INIT(64, 1, 3, 64),
  AI_STRIDE_INIT(768, 768, 256, 4),
  &conv2d_7_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_7_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_7_bias,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &conv2d_7_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_7_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_6_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_7_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_7_weights, &conv2d_7_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_7_layer, 18,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &permute_1_layer, AI_STATIC,
  .tensors = &conv2d_7_chain, 
  .groups = 1, 
  .nl_func = nl_func_tanh_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 1), 
)

/* Layer #7: "permute_1" (Permute) */
  


AI_TENSOR_CHAIN_OBJ_DECLARE(
  permute_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_7_output),
  AI_TENSOR_LIST_ENTRY(&permute_1_output_in),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  permute_1_layer, 22,
  PERMUTE_TYPE,
  permute, forward_permute,
  &AI_NET_OBJ_INSTANCE, &batch_normalization_8_layer, AI_STATIC,
  .tensors = &permute_1_chain, 
  .out_mapping = AI_SHAPE_INIT(AI_SHAPE_WIDTH, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_IN_CHANNEL), 
)

/* Layer #8: "batch_normalization_8" (ScaleBias) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_8_scale_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 384,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_8_scale,
  AI_SHAPE_INIT(1, 1, 384, 1),
  AI_STRIDE_INIT(1536, 1536, 4, 4),
  &batch_normalization_8_scale_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_8_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 384,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_8_bias,
  AI_SHAPE_INIT(1, 1, 384, 1),
  AI_STRIDE_INIT(1536, 1536, 4, 4),
  &batch_normalization_8_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  batch_normalization_8_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&permute_1_output_out),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_8_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_8_scale, &batch_normalization_8_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  batch_normalization_8_layer, 24,
  BN_TYPE,
  bn, forward_bn,
  &AI_NET_OBJ_INSTANCE, &gru_1_layer, AI_STATIC,
  .tensors = &batch_normalization_8_chain, 
)

/* Layer #9: "gru_1" (GRU) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  gru_1_kernel_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 73728,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_1_kernel,
  AI_SHAPE_INIT(192, 1, 1, 384),
  AI_STRIDE_INIT(1536, 1536, 1536, 4),
  &gru_1_kernel_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  gru_1_recurrent_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_1_recurrent,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_1_recurrent_array,
  AI_STATIC)

/* Weight tensor #3 */
AI_ARRAY_OBJ_DECLARE(
  gru_1_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 192,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_1_bias,
  AI_SHAPE_INIT(1, 1, 192, 1),
  AI_STRIDE_INIT(768, 768, 4, 4),
  &gru_1_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  gru_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&batch_normalization_8_output),
  AI_TENSOR_LIST_ENTRY(&gru_1_output),
  AI_TENSOR_LIST_ENTRY(&gru_1_kernel, &gru_1_recurrent, &gru_1_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  gru_1_layer, 25,
  GRU_TYPE,
  gru, forward_gru,
  &AI_NET_OBJ_INSTANCE, &batch_normalization_9_layer, AI_STATIC,
  .tensors = &gru_1_chain, 
  .n_units = 64, 
  .activation_nl = ai_math_tanh, 
  .recurrent_nl = ai_math_hard_sigmoid, 
  .reset_after = false, 
)

/* Layer #10: "batch_normalization_9" (ScaleBias) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_9_scale_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_9_scale,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_9_scale_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_9_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_9_bias,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_9_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  batch_normalization_9_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&gru_1_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_9_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_9_scale, &batch_normalization_9_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  batch_normalization_9_layer, 26,
  BN_TYPE,
  bn, forward_bn,
  &AI_NET_OBJ_INSTANCE, &gru_2_layer, AI_STATIC,
  .tensors = &batch_normalization_9_chain, 
)

/* Layer #11: "gru_2" (GRU) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  gru_2_kernel_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_2_kernel,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_2_kernel_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  gru_2_recurrent_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_2_recurrent,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_2_recurrent_array,
  AI_STATIC)

/* Weight tensor #3 */
AI_ARRAY_OBJ_DECLARE(
  gru_2_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 192,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_2_bias,
  AI_SHAPE_INIT(1, 1, 192, 1),
  AI_STRIDE_INIT(768, 768, 4, 4),
  &gru_2_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  gru_2_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&batch_normalization_9_output),
  AI_TENSOR_LIST_ENTRY(&gru_2_output),
  AI_TENSOR_LIST_ENTRY(&gru_2_kernel, &gru_2_recurrent, &gru_2_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  gru_2_layer, 27,
  GRU_TYPE,
  gru, forward_gru,
  &AI_NET_OBJ_INSTANCE, &batch_normalization_10_layer, AI_STATIC,
  .tensors = &gru_2_chain, 
  .n_units = 64, 
  .activation_nl = ai_math_tanh, 
  .recurrent_nl = ai_math_hard_sigmoid, 
  .reset_after = false, 
)

/* Layer #12: "batch_normalization_10" (ScaleBias) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_10_scale_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_10_scale,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_10_scale_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_10_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_10_bias,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_10_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  batch_normalization_10_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&gru_2_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_10_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_10_scale, &batch_normalization_10_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  batch_normalization_10_layer, 28,
  BN_TYPE,
  bn, forward_bn,
  &AI_NET_OBJ_INSTANCE, &gru_3_layer, AI_STATIC,
  .tensors = &batch_normalization_10_chain, 
)

/* Layer #13: "gru_3" (GRU) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  gru_3_kernel_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_3_kernel,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_3_kernel_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  gru_3_recurrent_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_3_recurrent,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_3_recurrent_array,
  AI_STATIC)

/* Weight tensor #3 */
AI_ARRAY_OBJ_DECLARE(
  gru_3_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 192,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_3_bias,
  AI_SHAPE_INIT(1, 1, 192, 1),
  AI_STRIDE_INIT(768, 768, 4, 4),
  &gru_3_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  gru_3_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&batch_normalization_10_output),
  AI_TENSOR_LIST_ENTRY(&gru_3_output),
  AI_TENSOR_LIST_ENTRY(&gru_3_kernel, &gru_3_recurrent, &gru_3_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  gru_3_layer, 29,
  GRU_TYPE,
  gru, forward_gru,
  &AI_NET_OBJ_INSTANCE, &batch_normalization_11_layer, AI_STATIC,
  .tensors = &gru_3_chain, 
  .n_units = 64, 
  .activation_nl = ai_math_tanh, 
  .recurrent_nl = ai_math_hard_sigmoid, 
  .reset_after = false, 
)

/* Layer #14: "batch_normalization_11" (ScaleBias) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_11_scale_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_11_scale,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_11_scale_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_11_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_11_bias,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_11_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  batch_normalization_11_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&gru_3_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_11_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_11_scale, &batch_normalization_11_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  batch_normalization_11_layer, 30,
  BN_TYPE,
  bn, forward_bn,
  &AI_NET_OBJ_INSTANCE, &gru_4_layer, AI_STATIC,
  .tensors = &batch_normalization_11_chain, 
)

/* Layer #15: "gru_4" (GRU) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  gru_4_kernel_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_4_kernel,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_4_kernel_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  gru_4_recurrent_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 12288,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_4_recurrent,
  AI_SHAPE_INIT(192, 1, 1, 64),
  AI_STRIDE_INIT(256, 256, 256, 4),
  &gru_4_recurrent_array,
  AI_STATIC)

/* Weight tensor #3 */
AI_ARRAY_OBJ_DECLARE(
  gru_4_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 192,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  gru_4_bias,
  AI_SHAPE_INIT(1, 1, 192, 1),
  AI_STRIDE_INIT(768, 768, 4, 4),
  &gru_4_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  gru_4_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&batch_normalization_11_output),
  AI_TENSOR_LIST_ENTRY(&gru_4_output),
  AI_TENSOR_LIST_ENTRY(&gru_4_kernel, &gru_4_recurrent, &gru_4_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  gru_4_layer, 31,
  GRU_TYPE,
  gru, forward_gru,
  &AI_NET_OBJ_INSTANCE, &batch_normalization_12_layer, AI_STATIC,
  .tensors = &gru_4_chain, 
  .n_units = 64, 
  .activation_nl = ai_math_tanh, 
  .recurrent_nl = ai_math_hard_sigmoid, 
  .reset_after = false, 
)

/* Layer #16: "batch_normalization_12" (ScaleBias) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_12_scale_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_12_scale,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_12_scale_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  batch_normalization_12_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 64,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  batch_normalization_12_bias,
  AI_SHAPE_INIT(1, 1, 64, 1),
  AI_STRIDE_INIT(256, 256, 4, 4),
  &batch_normalization_12_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  batch_normalization_12_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&gru_4_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_12_output),
  AI_TENSOR_LIST_ENTRY(&batch_normalization_12_scale, &batch_normalization_12_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  batch_normalization_12_layer, 32,
  BN_TYPE,
  bn, forward_bn,
  &AI_NET_OBJ_INSTANCE, &dense_1_layer, AI_STATIC,
  .tensors = &batch_normalization_12_chain, 
)

/* Layer #17: "dense_1" (Dense) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 640,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  dense_1_weights,
  AI_SHAPE_INIT(1, 1, 10, 64),
  AI_STRIDE_INIT(2560, 2560, 256, 4),
  &dense_1_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 10,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  dense_1_bias,
  AI_SHAPE_INIT(1, 1, 10, 1),
  AI_STRIDE_INIT(40, 40, 4, 4),
  &dense_1_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&batch_normalization_12_output),
  AI_TENSOR_LIST_ENTRY(&dense_1_output),
  AI_TENSOR_LIST_ENTRY(&dense_1_weights, &dense_1_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_layer, 34,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_1_nl_layer, AI_STATIC,
  .tensors = &dense_1_chain, 
)

/* Layer #18: "dense_1_nl" (Nonlinearity) */
  


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_nl_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&dense_1_output),
  AI_TENSOR_LIST_ENTRY(&dense_1_nl_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_nl_layer, 34,
  SM_TYPE,
  sm, forward_sm,
  &AI_NET_OBJ_INSTANCE, &dense_1_nl_layer, AI_STATIC,
  .tensors = &dense_1_nl_chain, 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 789000, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 73732, 1,
                     NULL),
  &input_0_output, &dense_1_nl_output,
  &conv2d_1_layer, 0)


AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT( activations )
  AI_FLAG_SET(net_ctx->flags, AI_NETWORK_FLAG_OUT_COPY);

  {
    /* Updating activations (byte) offsets */
    input_0_output_array.data = NULL;
  input_0_output_array.data_start = NULL;
  conv2d_1_output_array.data = activations + 0;
  conv2d_1_output_array.data_start = activations + 0;
  conv2d_2_output_array.data = activations + 36864;
  conv2d_2_output_array.data_start = activations + 36864;
  conv2d_3_output_array.data = activations + 0;
  conv2d_3_output_array.data_start = activations + 0;
  conv2d_4_output_array.data = activations + 36864;
  conv2d_4_output_array.data_start = activations + 36864;
  conv2d_5_output_array.data = activations + 0;
  conv2d_5_output_array.data_start = activations + 0;
  conv2d_6_output_array.data = activations + 36864;
  conv2d_6_output_array.data_start = activations + 36864;
  conv2d_7_output_array.data = activations + 0;
  conv2d_7_output_array.data_start = activations + 0;
  permute_1_output_in_array.data = activations + 24576;
  permute_1_output_in_array.data_start = activations + 24576;
  permute_1_output_out_array.data = activations + 24576;
  permute_1_output_out_array.data_start = activations + 24576;
  batch_normalization_8_output_array.data = activations + 24576;
  batch_normalization_8_output_array.data_start = activations + 24576;
  gru_1_output_array.data = activations + 0;
  gru_1_output_array.data_start = activations + 0;
  batch_normalization_9_output_array.data = activations + 0;
  batch_normalization_9_output_array.data_start = activations + 0;
  gru_2_output_array.data = activations + 4096;
  gru_2_output_array.data_start = activations + 4096;
  batch_normalization_10_output_array.data = activations + 4096;
  batch_normalization_10_output_array.data_start = activations + 4096;
  gru_3_output_array.data = activations + 0;
  gru_3_output_array.data_start = activations + 0;
  batch_normalization_11_output_array.data = activations + 0;
  batch_normalization_11_output_array.data_start = activations + 0;
  gru_4_output_array.data = activations + 4096;
  gru_4_output_array.data_start = activations + 4096;
  batch_normalization_12_output_array.data = activations + 4096;
  batch_normalization_12_output_array.data_start = activations + 4096;
  dense_1_output_array.data = activations + 0;
  dense_1_output_array.data_start = activations + 0;
  dense_1_nl_output_array.data = activations + 0;
  dense_1_nl_output_array.data_start = activations + 0;
  
  }
  return true;
}

AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT( weights )

  {
    /* Updating weights (byte) offsets */
    conv2d_1_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_1_weights_array.data = weights + 0;
  conv2d_1_weights_array.data_start = weights + 0;
  conv2d_1_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_1_bias_array.data = weights + 240;
  conv2d_1_bias_array.data_start = weights + 240;
  conv2d_2_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_2_weights_array.data = weights + 288;
  conv2d_2_weights_array.data_start = weights + 288;
  conv2d_2_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_2_bias_array.data = weights + 3744;
  conv2d_2_bias_array.data_start = weights + 3744;
  conv2d_3_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_3_weights_array.data = weights + 3840;
  conv2d_3_weights_array.data_start = weights + 3840;
  conv2d_3_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_3_bias_array.data = weights + 10752;
  conv2d_3_bias_array.data_start = weights + 10752;
  conv2d_4_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_4_weights_array.data = weights + 10848;
  conv2d_4_weights_array.data_start = weights + 10848;
  conv2d_4_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_4_bias_array.data = weights + 24672;
  conv2d_4_bias_array.data_start = weights + 24672;
  conv2d_5_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_5_weights_array.data = weights + 24864;
  conv2d_5_weights_array.data_start = weights + 24864;
  conv2d_5_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_5_bias_array.data = weights + 52512;
  conv2d_5_bias_array.data_start = weights + 52512;
  conv2d_6_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_6_weights_array.data = weights + 52704;
  conv2d_6_weights_array.data_start = weights + 52704;
  conv2d_6_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_6_bias_array.data = weights + 89568;
  conv2d_6_bias_array.data_start = weights + 89568;
  conv2d_7_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_7_weights_array.data = weights + 89824;
  conv2d_7_weights_array.data_start = weights + 89824;
  conv2d_7_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_7_bias_array.data = weights + 138976;
  conv2d_7_bias_array.data_start = weights + 138976;
  batch_normalization_8_scale_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_8_scale_array.data = weights + 139232;
  batch_normalization_8_scale_array.data_start = weights + 139232;
  batch_normalization_8_bias_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_8_bias_array.data = weights + 140768;
  batch_normalization_8_bias_array.data_start = weights + 140768;
  gru_1_kernel_array.format |= AI_FMT_FLAG_CONST;
  gru_1_kernel_array.data = weights + 142304;
  gru_1_kernel_array.data_start = weights + 142304;
  gru_1_recurrent_array.format |= AI_FMT_FLAG_CONST;
  gru_1_recurrent_array.data = weights + 437216;
  gru_1_recurrent_array.data_start = weights + 437216;
  gru_1_bias_array.format |= AI_FMT_FLAG_CONST;
  gru_1_bias_array.data = weights + 486368;
  gru_1_bias_array.data_start = weights + 486368;
  batch_normalization_9_scale_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_9_scale_array.data = weights + 487136;
  batch_normalization_9_scale_array.data_start = weights + 487136;
  batch_normalization_9_bias_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_9_bias_array.data = weights + 487392;
  batch_normalization_9_bias_array.data_start = weights + 487392;
  gru_2_kernel_array.format |= AI_FMT_FLAG_CONST;
  gru_2_kernel_array.data = weights + 487648;
  gru_2_kernel_array.data_start = weights + 487648;
  gru_2_recurrent_array.format |= AI_FMT_FLAG_CONST;
  gru_2_recurrent_array.data = weights + 536800;
  gru_2_recurrent_array.data_start = weights + 536800;
  gru_2_bias_array.format |= AI_FMT_FLAG_CONST;
  gru_2_bias_array.data = weights + 585952;
  gru_2_bias_array.data_start = weights + 585952;
  batch_normalization_10_scale_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_10_scale_array.data = weights + 586720;
  batch_normalization_10_scale_array.data_start = weights + 586720;
  batch_normalization_10_bias_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_10_bias_array.data = weights + 586976;
  batch_normalization_10_bias_array.data_start = weights + 586976;
  gru_3_kernel_array.format |= AI_FMT_FLAG_CONST;
  gru_3_kernel_array.data = weights + 587232;
  gru_3_kernel_array.data_start = weights + 587232;
  gru_3_recurrent_array.format |= AI_FMT_FLAG_CONST;
  gru_3_recurrent_array.data = weights + 636384;
  gru_3_recurrent_array.data_start = weights + 636384;
  gru_3_bias_array.format |= AI_FMT_FLAG_CONST;
  gru_3_bias_array.data = weights + 685536;
  gru_3_bias_array.data_start = weights + 685536;
  batch_normalization_11_scale_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_11_scale_array.data = weights + 686304;
  batch_normalization_11_scale_array.data_start = weights + 686304;
  batch_normalization_11_bias_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_11_bias_array.data = weights + 686560;
  batch_normalization_11_bias_array.data_start = weights + 686560;
  gru_4_kernel_array.format |= AI_FMT_FLAG_CONST;
  gru_4_kernel_array.data = weights + 686816;
  gru_4_kernel_array.data_start = weights + 686816;
  gru_4_recurrent_array.format |= AI_FMT_FLAG_CONST;
  gru_4_recurrent_array.data = weights + 735968;
  gru_4_recurrent_array.data_start = weights + 735968;
  gru_4_bias_array.format |= AI_FMT_FLAG_CONST;
  gru_4_bias_array.data = weights + 785120;
  gru_4_bias_array.data_start = weights + 785120;
  batch_normalization_12_scale_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_12_scale_array.data = weights + 785888;
  batch_normalization_12_scale_array.data_start = weights + 785888;
  batch_normalization_12_bias_array.format |= AI_FMT_FLAG_CONST;
  batch_normalization_12_bias_array.data = weights + 786144;
  batch_normalization_12_bias_array.data_start = weights + 786144;
  dense_1_weights_array.format |= AI_FMT_FLAG_CONST;
  dense_1_weights_array.data = weights + 786400;
  dense_1_weights_array.data_start = weights + 786400;
  dense_1_bias_array.format |= AI_FMT_FLAG_CONST;
  dense_1_bias_array.data = weights + 788960;
  dense_1_bias_array.data_start = weights + 788960;
  
  }

  return true;
}

/**  PUBLIC APIs SECTION  *****************************************************/

AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if ( report && net_ctx )
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = {AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR,
                            AI_TOOLS_API_VERSION_MICRO, 0x0},

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 8194738,
      .n_inputs          = AI_NETWORK_IN_NUM,
      .inputs            = AI_BUFFER_OBJ_INIT(
                              AI_BUFFER_FORMAT_FLOAT,
                              6,
                              128,
                              1,
                              1, NULL),
      .n_outputs         = AI_NETWORK_OUT_NUM,
      .outputs           = AI_BUFFER_OBJ_INIT(
                              AI_BUFFER_FORMAT_FLOAT,
                              1,
                              1,
                              10,
                              1, NULL),
      .activations       = net_ctx->activations,
      .weights           = net_ctx->params,
      .n_nodes           = 0,
      .signature         = net_ctx->signature,
    };

    AI_FOR_EACH_NODE_DO(node, net_ctx->input_node)
    {
      r.n_nodes++;
    }

    *report = r;

    return ( r.n_nodes>0 ) ? true : false;
  }
  
  return false;
}

AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if ( !net_ctx ) return false;

  ai_bool ok = true;
  ok &= network_configure_weights(net_ctx, &params->params);
  ok &= network_configure_activations(net_ctx, &params->activations);
  
  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}

#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

