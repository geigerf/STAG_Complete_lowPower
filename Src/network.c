/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Thu Aug 27 14:34:14 2020
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
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
#define AI_TOOLS_VERSION_MAJOR 5
#define AI_TOOLS_VERSION_MINOR 0
#define AI_TOOLS_VERSION_MICRO 0


#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 3
#define AI_TOOLS_API_VERSION_MICRO 0

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "dee9fa1ba4c6971b5ff1764102c2c7bc"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     "(rev-5.0.0)"
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Thu Aug 27 14:34:14 2020"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

/**  Forward network declaration section  *************************************/
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/**  Forward network array declarations  **************************************/
AI_STATIC ai_array node_99_scratch0_array;   /* Array #0 */
AI_STATIC ai_array node_65_scratch0_array;   /* Array #1 */
AI_STATIC ai_array output_C_array;   /* Array #2 */
AI_STATIC ai_array output_B_array;   /* Array #3 */
AI_STATIC ai_array node_99_bias_array;   /* Array #4 */
AI_STATIC ai_array node_99_weights_array;   /* Array #5 */
AI_STATIC ai_array node_95_bias_array;   /* Array #6 */
AI_STATIC ai_array node_95_weights_array;   /* Array #7 */
AI_STATIC ai_array node_92_bias_array;   /* Array #8 */
AI_STATIC ai_array node_92_weights_array;   /* Array #9 */
AI_STATIC ai_array node_88_bias_array;   /* Array #10 */
AI_STATIC ai_array node_88_weights_array;   /* Array #11 */
AI_STATIC ai_array node_86_bias_array;   /* Array #12 */
AI_STATIC ai_array node_86_weights_array;   /* Array #13 */
AI_STATIC ai_array node_83_bias_array;   /* Array #14 */
AI_STATIC ai_array node_83_weights_array;   /* Array #15 */
AI_STATIC ai_array node_79_bias_array;   /* Array #16 */
AI_STATIC ai_array node_79_weights_array;   /* Array #17 */
AI_STATIC ai_array node_76_bias_array;   /* Array #18 */
AI_STATIC ai_array node_76_weights_array;   /* Array #19 */
AI_STATIC ai_array node_72_bias_array;   /* Array #20 */
AI_STATIC ai_array node_72_weights_array;   /* Array #21 */
AI_STATIC ai_array node_69_bias_array;   /* Array #22 */
AI_STATIC ai_array node_69_weights_array;   /* Array #23 */
AI_STATIC ai_array node_65_bias_array;   /* Array #24 */
AI_STATIC ai_array node_65_weights_array;   /* Array #25 */
AI_STATIC ai_array input_output_array;   /* Array #26 */
AI_STATIC ai_array node_65_output_array;   /* Array #27 */
AI_STATIC ai_array node_69_output_array;   /* Array #28 */
AI_STATIC ai_array node_72_output_array;   /* Array #29 */
AI_STATIC ai_array node_74_output_array;   /* Array #30 */
AI_STATIC ai_array node_75_output_array;   /* Array #31 */
AI_STATIC ai_array node_76_output_array;   /* Array #32 */
AI_STATIC ai_array node_79_output_array;   /* Array #33 */
AI_STATIC ai_array node_81_output_array;   /* Array #34 */
AI_STATIC ai_array node_82_output_array;   /* Array #35 */
AI_STATIC ai_array node_88_output_array;   /* Array #36 */
AI_STATIC ai_array node_83_output_array;   /* Array #37 */
AI_STATIC ai_array node_86_output_array;   /* Array #38 */
AI_STATIC ai_array node_90_output_array;   /* Array #39 */
AI_STATIC ai_array node_91_output_array;   /* Array #40 */
AI_STATIC ai_array node_92_output_array;   /* Array #41 */
AI_STATIC ai_array node_95_output_array;   /* Array #42 */
AI_STATIC ai_array node_97_output_array;   /* Array #43 */
AI_STATIC ai_array node_98_output_array;   /* Array #44 */
AI_STATIC ai_array node_99_output_array;   /* Array #45 */
AI_STATIC ai_array output_output_array;   /* Array #46 */


/**  Forward network tensor declarations  *************************************/
AI_STATIC ai_tensor node_99_scratch0;   /* Tensor #0 */
AI_STATIC ai_tensor node_65_scratch0;   /* Tensor #1 */
AI_STATIC ai_tensor output_C;   /* Tensor #2 */
AI_STATIC ai_tensor output_B;   /* Tensor #3 */
AI_STATIC ai_tensor node_99_bias;   /* Tensor #4 */
AI_STATIC ai_tensor node_99_weights;   /* Tensor #5 */
AI_STATIC ai_tensor node_95_bias;   /* Tensor #6 */
AI_STATIC ai_tensor node_95_weights;   /* Tensor #7 */
AI_STATIC ai_tensor node_92_bias;   /* Tensor #8 */
AI_STATIC ai_tensor node_92_weights;   /* Tensor #9 */
AI_STATIC ai_tensor node_88_bias;   /* Tensor #10 */
AI_STATIC ai_tensor node_88_weights;   /* Tensor #11 */
AI_STATIC ai_tensor node_86_bias;   /* Tensor #12 */
AI_STATIC ai_tensor node_86_weights;   /* Tensor #13 */
AI_STATIC ai_tensor node_83_bias;   /* Tensor #14 */
AI_STATIC ai_tensor node_83_weights;   /* Tensor #15 */
AI_STATIC ai_tensor node_79_bias;   /* Tensor #16 */
AI_STATIC ai_tensor node_79_weights;   /* Tensor #17 */
AI_STATIC ai_tensor node_76_bias;   /* Tensor #18 */
AI_STATIC ai_tensor node_76_weights;   /* Tensor #19 */
AI_STATIC ai_tensor node_72_bias;   /* Tensor #20 */
AI_STATIC ai_tensor node_72_weights;   /* Tensor #21 */
AI_STATIC ai_tensor node_69_bias;   /* Tensor #22 */
AI_STATIC ai_tensor node_69_weights;   /* Tensor #23 */
AI_STATIC ai_tensor node_65_bias;   /* Tensor #24 */
AI_STATIC ai_tensor node_65_weights;   /* Tensor #25 */
AI_STATIC ai_tensor input_output;   /* Tensor #26 */
AI_STATIC ai_tensor node_65_output;   /* Tensor #27 */
AI_STATIC ai_tensor node_69_output;   /* Tensor #28 */
AI_STATIC ai_tensor node_72_output;   /* Tensor #29 */
AI_STATIC ai_tensor node_74_output;   /* Tensor #30 */
AI_STATIC ai_tensor node_75_output;   /* Tensor #31 */
AI_STATIC ai_tensor node_76_output;   /* Tensor #32 */
AI_STATIC ai_tensor node_79_output;   /* Tensor #33 */
AI_STATIC ai_tensor node_81_output;   /* Tensor #34 */
AI_STATIC ai_tensor node_82_output;   /* Tensor #35 */
AI_STATIC ai_tensor node_88_output;   /* Tensor #36 */
AI_STATIC ai_tensor node_83_output;   /* Tensor #37 */
AI_STATIC ai_tensor node_86_output;   /* Tensor #38 */
AI_STATIC ai_tensor node_90_output;   /* Tensor #39 */
AI_STATIC ai_tensor node_91_output;   /* Tensor #40 */
AI_STATIC ai_tensor node_92_output;   /* Tensor #41 */
AI_STATIC ai_tensor node_95_output;   /* Tensor #42 */
AI_STATIC ai_tensor node_97_output;   /* Tensor #43 */
AI_STATIC ai_tensor node_98_output;   /* Tensor #44 */
AI_STATIC ai_tensor node_99_output;   /* Tensor #45 */
AI_STATIC ai_tensor output_output;   /* Tensor #46 */


/**  Forward network tensor chain declarations  *******************************/
AI_STATIC_CONST ai_tensor_chain node_65_chain;   /* Chain #0 */
AI_STATIC_CONST ai_tensor_chain node_69_chain;   /* Chain #1 */
AI_STATIC_CONST ai_tensor_chain node_72_chain;   /* Chain #2 */
AI_STATIC_CONST ai_tensor_chain node_74_chain;   /* Chain #3 */
AI_STATIC_CONST ai_tensor_chain node_75_chain;   /* Chain #4 */
AI_STATIC_CONST ai_tensor_chain node_76_chain;   /* Chain #5 */
AI_STATIC_CONST ai_tensor_chain node_79_chain;   /* Chain #6 */
AI_STATIC_CONST ai_tensor_chain node_81_chain;   /* Chain #7 */
AI_STATIC_CONST ai_tensor_chain node_82_chain;   /* Chain #8 */
AI_STATIC_CONST ai_tensor_chain node_88_chain;   /* Chain #9 */
AI_STATIC_CONST ai_tensor_chain node_83_chain;   /* Chain #10 */
AI_STATIC_CONST ai_tensor_chain node_86_chain;   /* Chain #11 */
AI_STATIC_CONST ai_tensor_chain node_90_chain;   /* Chain #12 */
AI_STATIC_CONST ai_tensor_chain node_91_chain;   /* Chain #13 */
AI_STATIC_CONST ai_tensor_chain node_92_chain;   /* Chain #14 */
AI_STATIC_CONST ai_tensor_chain node_95_chain;   /* Chain #15 */
AI_STATIC_CONST ai_tensor_chain node_97_chain;   /* Chain #16 */
AI_STATIC_CONST ai_tensor_chain node_98_chain;   /* Chain #17 */
AI_STATIC_CONST ai_tensor_chain node_99_chain;   /* Chain #18 */
AI_STATIC_CONST ai_tensor_chain output_chain;   /* Chain #19 */


/**  Forward network layer declarations  **************************************/
AI_STATIC ai_layer_conv2d_nl_pool node_65_layer; /* Layer #0 */
AI_STATIC ai_layer_conv2d node_69_layer; /* Layer #1 */
AI_STATIC ai_layer_conv2d node_72_layer; /* Layer #2 */
AI_STATIC ai_layer_eltwise node_74_layer; /* Layer #3 */
AI_STATIC ai_layer_nl node_75_layer; /* Layer #4 */
AI_STATIC ai_layer_conv2d node_76_layer; /* Layer #5 */
AI_STATIC ai_layer_conv2d node_79_layer; /* Layer #6 */
AI_STATIC ai_layer_eltwise node_81_layer; /* Layer #7 */
AI_STATIC ai_layer_nl node_82_layer; /* Layer #8 */
AI_STATIC ai_layer_conv2d node_88_layer; /* Layer #9 */
AI_STATIC ai_layer_conv2d node_83_layer; /* Layer #10 */
AI_STATIC ai_layer_conv2d node_86_layer; /* Layer #11 */
AI_STATIC ai_layer_eltwise node_90_layer; /* Layer #12 */
AI_STATIC ai_layer_nl node_91_layer; /* Layer #13 */
AI_STATIC ai_layer_conv2d node_92_layer; /* Layer #14 */
AI_STATIC ai_layer_conv2d node_95_layer; /* Layer #15 */
AI_STATIC ai_layer_eltwise node_97_layer; /* Layer #16 */
AI_STATIC ai_layer_nl node_98_layer; /* Layer #17 */
AI_STATIC ai_layer_conv2d_nl_pool node_99_layer; /* Layer #18 */
AI_STATIC ai_layer_gemm output_layer; /* Layer #19 */


/**  Array declarations section  **********************************************/
AI_ARRAY_OBJ_DECLARE(
    node_99_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_65_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 1536,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    output_C_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 17,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    output_B_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 544,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_99_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_99_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 1024,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_95_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_95_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 9216,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_92_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_92_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 9216,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_88_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_88_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 512,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_86_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_86_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 9216,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_83_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_83_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4608,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_79_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 16,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_79_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2304,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_76_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 16,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_76_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2304,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_72_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 16,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_72_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2304,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_69_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 16,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_69_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2304,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_65_bias_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 16,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_65_weights_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 144,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    input_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
    NULL, NULL, 1024,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_65_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_69_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_72_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_74_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_75_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_76_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_79_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_81_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_82_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 4096,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_88_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_83_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_86_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_90_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_91_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_92_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_95_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_97_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_98_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 2048,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    node_99_output_array, AI_ARRAY_FORMAT_FLOAT,
    NULL, NULL, 32,
     AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
    output_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
    NULL, NULL, 17,
     AI_STATIC)




/**  Tensor declarations section  *********************************************/
AI_TENSOR_OBJ_DECLARE(
  node_99_scratch0, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_99_scratch0_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_65_scratch0, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 32, 3), AI_STRIDE_INIT(4, 4, 4, 64, 2048),
  1, &node_65_scratch0_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  output_C, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 17, 1, 1), AI_STRIDE_INIT(4, 4, 4, 68, 68),
  1, &output_C_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  output_B, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 17), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &output_B_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_99_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_99_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_99_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 32, 1, 1, 32), AI_STRIDE_INIT(4, 4, 128, 128, 128),
  1, &node_99_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_95_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_95_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_95_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 32, 3, 3, 32), AI_STRIDE_INIT(4, 4, 128, 384, 1152),
  1, &node_95_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_92_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_92_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_92_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 32, 3, 3, 32), AI_STRIDE_INIT(4, 4, 128, 384, 1152),
  1, &node_92_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_88_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_88_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_88_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 16, 1, 1, 32), AI_STRIDE_INIT(4, 4, 64, 64, 64),
  1, &node_88_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_86_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_86_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_86_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 32, 3, 3, 32), AI_STRIDE_INIT(4, 4, 128, 384, 1152),
  1, &node_86_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_83_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_83_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_83_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 16, 3, 3, 32), AI_STRIDE_INIT(4, 4, 64, 192, 576),
  1, &node_83_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_79_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &node_79_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_79_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 16, 3, 3, 16), AI_STRIDE_INIT(4, 4, 64, 192, 576),
  1, &node_79_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_76_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &node_76_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_76_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 16, 3, 3, 16), AI_STRIDE_INIT(4, 4, 64, 192, 576),
  1, &node_76_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_72_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &node_72_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_72_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 16, 3, 3, 16), AI_STRIDE_INIT(4, 4, 64, 192, 576),
  1, &node_72_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_69_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &node_69_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_69_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 16, 3, 3, 16), AI_STRIDE_INIT(4, 4, 64, 192, 576),
  1, &node_69_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_65_bias, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &node_65_bias_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_65_weights, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 3, 3, 16), AI_STRIDE_INIT(4, 4, 4, 12, 36),
  1, &node_65_weights_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  input_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 1, 32, 32), AI_STRIDE_INIT(4, 4, 4, 4, 128),
  1, &input_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_65_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_65_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_69_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_69_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_72_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_72_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_74_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_74_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_75_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_75_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_76_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_76_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_79_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_79_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_81_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_81_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_82_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 16, 16, 16), AI_STRIDE_INIT(4, 4, 4, 64, 1024),
  1, &node_82_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_88_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_88_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_83_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_83_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_86_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_86_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_90_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_90_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_91_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_91_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_92_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_92_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_95_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_95_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_97_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_97_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_98_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &node_98_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  node_99_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &node_99_output_array, NULL)
AI_TENSOR_OBJ_DECLARE(
  output_output, AI_STATIC,
  0x0, 0x0, AI_SHAPE_INIT(4, 1, 17, 1, 1), AI_STRIDE_INIT(4, 4, 4, 68, 68),
  1, &output_output_array, NULL)


/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_65_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&input_output),
  AI_TENSOR_LIST_ENTRY(&node_65_output),
  AI_TENSOR_LIST_ENTRY(&node_65_weights, &node_65_bias, NULL),
  AI_TENSOR_LIST_ENTRY(&node_65_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  node_65_layer, 0,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &node_69_layer, AI_STATIC,
  .tensors = &node_65_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .pool_size = AI_SHAPE_2D_INIT(3, 3), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .pool_func = pool_func_mp_array_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_69_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_65_output),
  AI_TENSOR_LIST_ENTRY(&node_69_output),
  AI_TENSOR_LIST_ENTRY(&node_69_weights, &node_69_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_69_layer, 3,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_72_layer, AI_STATIC,
  .tensors = &node_69_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_72_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_69_output),
  AI_TENSOR_LIST_ENTRY(&node_72_output),
  AI_TENSOR_LIST_ENTRY(&node_72_weights, &node_72_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_72_layer, 5,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_74_layer, AI_STATIC,
  .tensors = &node_72_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_74_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_72_output, &node_65_output),
  AI_TENSOR_LIST_ENTRY(&node_74_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_74_layer, 6,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &node_75_layer, AI_STATIC,
  .tensors = &node_74_chain, 
  .operation = ai_sum, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_75_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_74_output),
  AI_TENSOR_LIST_ENTRY(&node_75_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_75_layer, 7,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &node_76_layer, AI_STATIC,
  .tensors = &node_75_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_76_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_75_output),
  AI_TENSOR_LIST_ENTRY(&node_76_output),
  AI_TENSOR_LIST_ENTRY(&node_76_weights, &node_76_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_76_layer, 8,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_79_layer, AI_STATIC,
  .tensors = &node_76_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_79_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_76_output),
  AI_TENSOR_LIST_ENTRY(&node_79_output),
  AI_TENSOR_LIST_ENTRY(&node_79_weights, &node_79_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_79_layer, 10,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_81_layer, AI_STATIC,
  .tensors = &node_79_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_81_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_79_output, &node_75_output),
  AI_TENSOR_LIST_ENTRY(&node_81_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_81_layer, 11,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &node_82_layer, AI_STATIC,
  .tensors = &node_81_chain, 
  .operation = ai_sum, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_82_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_81_output),
  AI_TENSOR_LIST_ENTRY(&node_82_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_82_layer, 12,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &node_88_layer, AI_STATIC,
  .tensors = &node_82_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_88_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_82_output),
  AI_TENSOR_LIST_ENTRY(&node_88_output),
  AI_TENSOR_LIST_ENTRY(&node_88_weights, &node_88_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_88_layer, 16,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_83_layer, AI_STATIC,
  .tensors = &node_88_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_83_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_82_output),
  AI_TENSOR_LIST_ENTRY(&node_83_output),
  AI_TENSOR_LIST_ENTRY(&node_83_weights, &node_83_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_83_layer, 13,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_86_layer, AI_STATIC,
  .tensors = &node_83_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_86_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_83_output),
  AI_TENSOR_LIST_ENTRY(&node_86_output),
  AI_TENSOR_LIST_ENTRY(&node_86_weights, &node_86_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_86_layer, 15,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_90_layer, AI_STATIC,
  .tensors = &node_86_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_90_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_86_output, &node_88_output),
  AI_TENSOR_LIST_ENTRY(&node_90_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_90_layer, 17,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &node_91_layer, AI_STATIC,
  .tensors = &node_90_chain, 
  .operation = ai_sum, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_91_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_90_output),
  AI_TENSOR_LIST_ENTRY(&node_91_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_91_layer, 18,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &node_92_layer, AI_STATIC,
  .tensors = &node_91_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_92_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_91_output),
  AI_TENSOR_LIST_ENTRY(&node_92_output),
  AI_TENSOR_LIST_ENTRY(&node_92_weights, &node_92_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_92_layer, 19,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_95_layer, AI_STATIC,
  .tensors = &node_92_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_95_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_92_output),
  AI_TENSOR_LIST_ENTRY(&node_95_output),
  AI_TENSOR_LIST_ENTRY(&node_95_weights, &node_95_bias, NULL),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_95_layer, 21,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &node_97_layer, AI_STATIC,
  .tensors = &node_95_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_97_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_95_output, &node_91_output),
  AI_TENSOR_LIST_ENTRY(&node_97_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_97_layer, 22,
  ELTWISE_TYPE,
  eltwise, forward_eltwise,
  &AI_NET_OBJ_INSTANCE, &node_98_layer, AI_STATIC,
  .tensors = &node_97_chain, 
  .operation = ai_sum, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_98_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_97_output),
  AI_TENSOR_LIST_ENTRY(&node_98_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  node_98_layer, 23,
  NL_TYPE,
  nl, forward_relu,
  &AI_NET_OBJ_INSTANCE, &node_99_layer, AI_STATIC,
  .tensors = &node_98_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_99_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_98_output),
  AI_TENSOR_LIST_ENTRY(&node_99_output),
  AI_TENSOR_LIST_ENTRY(&node_99_weights, &node_99_bias, NULL),
  AI_TENSOR_LIST_ENTRY(&node_99_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  node_99_layer, 24,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &output_layer, AI_STATIC,
  .tensors = &node_99_chain, 
  .groups = 1, 
  .nl_func = NULL, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .pool_size = AI_SHAPE_2D_INIT(8, 8), 
  .pool_stride = AI_SHAPE_2D_INIT(8, 8), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .pool_func = pool_func_ap_array_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  output_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_ENTRY(&node_99_output),
  AI_TENSOR_LIST_ENTRY(&output_output),
  AI_TENSOR_LIST_ENTRY(&output_B, &output_C),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  output_layer, 27,
  GEMM_TYPE,
  gemm, forward_gemm,
  &AI_NET_OBJ_INSTANCE, &output_layer, AI_STATIC,
  .tensors = &output_chain, 
  .alpha = 1.0, 
  .beta = 1.0, 
  .tA = 0, 
  .tB = 1, 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 175940, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 49152, 1,
                     NULL),
  AI_TENSOR_LIST_IO_ENTRY(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_ENTRY(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &output_output),
  &node_65_layer, 0, NULL)



AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT(activations)
  AI_UNUSED(net_ctx)

  {
    /* Updating activations (byte) offsets */
    node_99_scratch0_array.data = AI_PTR(activations + 0);
    node_99_scratch0_array.data_start = AI_PTR(activations + 0);
    node_65_scratch0_array.data = AI_PTR(activations + 0);
    node_65_scratch0_array.data_start = AI_PTR(activations + 0);
    input_output_array.data = AI_PTR(NULL);
    input_output_array.data_start = AI_PTR(NULL);
    node_65_output_array.data = AI_PTR(activations + 16384);
    node_65_output_array.data_start = AI_PTR(activations + 16384);
    node_69_output_array.data = AI_PTR(activations + 0);
    node_69_output_array.data_start = AI_PTR(activations + 0);
    node_72_output_array.data = AI_PTR(activations + 32768);
    node_72_output_array.data_start = AI_PTR(activations + 32768);
    node_74_output_array.data = AI_PTR(activations + 0);
    node_74_output_array.data_start = AI_PTR(activations + 0);
    node_75_output_array.data = AI_PTR(activations + 16384);
    node_75_output_array.data_start = AI_PTR(activations + 16384);
    node_76_output_array.data = AI_PTR(activations + 0);
    node_76_output_array.data_start = AI_PTR(activations + 0);
    node_79_output_array.data = AI_PTR(activations + 32768);
    node_79_output_array.data_start = AI_PTR(activations + 32768);
    node_81_output_array.data = AI_PTR(activations + 0);
    node_81_output_array.data_start = AI_PTR(activations + 0);
    node_82_output_array.data = AI_PTR(activations + 16384);
    node_82_output_array.data_start = AI_PTR(activations + 16384);
    node_88_output_array.data = AI_PTR(activations + 0);
    node_88_output_array.data_start = AI_PTR(activations + 0);
    node_83_output_array.data = AI_PTR(activations + 8192);
    node_83_output_array.data_start = AI_PTR(activations + 8192);
    node_86_output_array.data = AI_PTR(activations + 16384);
    node_86_output_array.data_start = AI_PTR(activations + 16384);
    node_90_output_array.data = AI_PTR(activations + 8192);
    node_90_output_array.data_start = AI_PTR(activations + 8192);
    node_91_output_array.data = AI_PTR(activations + 0);
    node_91_output_array.data_start = AI_PTR(activations + 0);
    node_92_output_array.data = AI_PTR(activations + 8192);
    node_92_output_array.data_start = AI_PTR(activations + 8192);
    node_95_output_array.data = AI_PTR(activations + 16384);
    node_95_output_array.data_start = AI_PTR(activations + 16384);
    node_97_output_array.data = AI_PTR(activations + 8192);
    node_97_output_array.data_start = AI_PTR(activations + 8192);
    node_98_output_array.data = AI_PTR(activations + 8192);
    node_98_output_array.data_start = AI_PTR(activations + 8192);
    node_99_output_array.data = AI_PTR(activations + 16384);
    node_99_output_array.data_start = AI_PTR(activations + 16384);
    output_output_array.data = AI_PTR(NULL);
    output_output_array.data_start = AI_PTR(NULL);
    
  }
  return true;
}



AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT(weights)
  AI_UNUSED(net_ctx)

  {
    /* Updating weights (byte) offsets */
    
    output_C_array.format |= AI_FMT_FLAG_CONST;
    output_C_array.data = AI_PTR(weights + 175872);
    output_C_array.data_start = AI_PTR(weights + 175872);
    output_B_array.format |= AI_FMT_FLAG_CONST;
    output_B_array.data = AI_PTR(weights + 173696);
    output_B_array.data_start = AI_PTR(weights + 173696);
    node_99_bias_array.format |= AI_FMT_FLAG_CONST;
    node_99_bias_array.data = AI_PTR(weights + 173568);
    node_99_bias_array.data_start = AI_PTR(weights + 173568);
    node_99_weights_array.format |= AI_FMT_FLAG_CONST;
    node_99_weights_array.data = AI_PTR(weights + 169472);
    node_99_weights_array.data_start = AI_PTR(weights + 169472);
    node_95_bias_array.format |= AI_FMT_FLAG_CONST;
    node_95_bias_array.data = AI_PTR(weights + 169344);
    node_95_bias_array.data_start = AI_PTR(weights + 169344);
    node_95_weights_array.format |= AI_FMT_FLAG_CONST;
    node_95_weights_array.data = AI_PTR(weights + 132480);
    node_95_weights_array.data_start = AI_PTR(weights + 132480);
    node_92_bias_array.format |= AI_FMT_FLAG_CONST;
    node_92_bias_array.data = AI_PTR(weights + 132352);
    node_92_bias_array.data_start = AI_PTR(weights + 132352);
    node_92_weights_array.format |= AI_FMT_FLAG_CONST;
    node_92_weights_array.data = AI_PTR(weights + 95488);
    node_92_weights_array.data_start = AI_PTR(weights + 95488);
    node_88_bias_array.format |= AI_FMT_FLAG_CONST;
    node_88_bias_array.data = AI_PTR(weights + 95360);
    node_88_bias_array.data_start = AI_PTR(weights + 95360);
    node_88_weights_array.format |= AI_FMT_FLAG_CONST;
    node_88_weights_array.data = AI_PTR(weights + 93312);
    node_88_weights_array.data_start = AI_PTR(weights + 93312);
    node_86_bias_array.format |= AI_FMT_FLAG_CONST;
    node_86_bias_array.data = AI_PTR(weights + 93184);
    node_86_bias_array.data_start = AI_PTR(weights + 93184);
    node_86_weights_array.format |= AI_FMT_FLAG_CONST;
    node_86_weights_array.data = AI_PTR(weights + 56320);
    node_86_weights_array.data_start = AI_PTR(weights + 56320);
    node_83_bias_array.format |= AI_FMT_FLAG_CONST;
    node_83_bias_array.data = AI_PTR(weights + 56192);
    node_83_bias_array.data_start = AI_PTR(weights + 56192);
    node_83_weights_array.format |= AI_FMT_FLAG_CONST;
    node_83_weights_array.data = AI_PTR(weights + 37760);
    node_83_weights_array.data_start = AI_PTR(weights + 37760);
    node_79_bias_array.format |= AI_FMT_FLAG_CONST;
    node_79_bias_array.data = AI_PTR(weights + 37696);
    node_79_bias_array.data_start = AI_PTR(weights + 37696);
    node_79_weights_array.format |= AI_FMT_FLAG_CONST;
    node_79_weights_array.data = AI_PTR(weights + 28480);
    node_79_weights_array.data_start = AI_PTR(weights + 28480);
    node_76_bias_array.format |= AI_FMT_FLAG_CONST;
    node_76_bias_array.data = AI_PTR(weights + 28416);
    node_76_bias_array.data_start = AI_PTR(weights + 28416);
    node_76_weights_array.format |= AI_FMT_FLAG_CONST;
    node_76_weights_array.data = AI_PTR(weights + 19200);
    node_76_weights_array.data_start = AI_PTR(weights + 19200);
    node_72_bias_array.format |= AI_FMT_FLAG_CONST;
    node_72_bias_array.data = AI_PTR(weights + 19136);
    node_72_bias_array.data_start = AI_PTR(weights + 19136);
    node_72_weights_array.format |= AI_FMT_FLAG_CONST;
    node_72_weights_array.data = AI_PTR(weights + 9920);
    node_72_weights_array.data_start = AI_PTR(weights + 9920);
    node_69_bias_array.format |= AI_FMT_FLAG_CONST;
    node_69_bias_array.data = AI_PTR(weights + 9856);
    node_69_bias_array.data_start = AI_PTR(weights + 9856);
    node_69_weights_array.format |= AI_FMT_FLAG_CONST;
    node_69_weights_array.data = AI_PTR(weights + 640);
    node_69_weights_array.data_start = AI_PTR(weights + 640);
    node_65_bias_array.format |= AI_FMT_FLAG_CONST;
    node_65_bias_array.data = AI_PTR(weights + 576);
    node_65_bias_array.data_start = AI_PTR(weights + 576);
    node_65_weights_array.format |= AI_FMT_FLAG_CONST;
    node_65_weights_array.data = AI_PTR(weights + 0);
    node_65_weights_array.data_start = AI_PTR(weights + 0);
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
      
      .n_macc            = 4761904,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .activations       = AI_STRUCT_INIT,
      .params            = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if ( !ai_platform_api_get_network_report(network, &r) ) return false;

    *report = r;
    return true;
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

