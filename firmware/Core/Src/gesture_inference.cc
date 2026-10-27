#include "gesture_inference.h"

#include "app_config.h"
#include "g_gesture_model_data.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/schema/schema_generated.h"

namespace {

constexpr size_t kTensorArenaBytes = APP_TENSOR_ARENA_BYTES;
constexpr int kClassCount = 6;

alignas(16) uint8_t tensor_arena[kTensorArenaBytes];
tflite::MicroInterpreter* interpreter = nullptr;
const TfLiteTensor* output_tensor = nullptr;

}  // namespace

extern "C" uint8_t GestureInference_Init(void) {
  static tflite::MicroMutableOpResolver<12> resolver;
  static bool resolver_initialized = false;
  if (!resolver_initialized) {
    if (resolver.AddExpandDims() != kTfLiteOk ||
        resolver.AddConv2D() != kTfLiteOk ||
        resolver.AddMul() != kTfLiteOk ||
        resolver.AddAdd() != kTfLiteOk ||
        resolver.AddReshape() != kTfLiteOk ||
        resolver.AddMaxPool2D() != kTfLiteOk ||
        resolver.AddMean() != kTfLiteOk ||
        resolver.AddFullyConnected() != kTfLiteOk ||
        resolver.AddSoftmax() != kTfLiteOk ||
        resolver.AddShape() != kTfLiteOk ||
        resolver.AddStridedSlice() != kTfLiteOk ||
        resolver.AddPack() != kTfLiteOk) {
      return 0U;
    }
    resolver_initialized = true;
  }

  const tflite::Model* model = tflite::GetModel(g_gesture_model_data);
  if (model == nullptr || model->version() != TFLITE_SCHEMA_VERSION) {
    return 0U;
  }

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaBytes);
  interpreter = &static_interpreter;
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    interpreter = nullptr;
    return 0U;
  }

  TfLiteTensor* input_tensor = interpreter->input(0);
  output_tensor = interpreter->output(0);
  if (input_tensor == nullptr || output_tensor == nullptr ||
      input_tensor->type != kTfLiteInt8 || output_tensor->type != kTfLiteInt8 ||
      input_tensor->bytes != 100U * 6U || output_tensor->bytes != kClassCount) {
    interpreter = nullptr;
    output_tensor = nullptr;
    return 0U;
  }
  return 1U;
}

extern "C" uint8_t GestureInference_Run(const int8_t* input,
                                         int* class_index,
                                         uint8_t* confidence_percent) {
  if (interpreter == nullptr || output_tensor == nullptr || input == nullptr ||
      class_index == nullptr || confidence_percent == nullptr) {
    return 0U;
  }

  TfLiteTensor* input_tensor = interpreter->input(0);
  for (size_t i = 0U; i < 100U * 6U; ++i) {
    input_tensor->data.int8[i] = input[i];
  }
  if (interpreter->Invoke() != kTfLiteOk) {
    return 0U;
  }

  const int8_t* scores = output_tensor->data.int8;
  int best_index = 0;
  for (int i = 1; i < kClassCount; ++i) {
    if (scores[i] > scores[best_index]) {
      best_index = i;
    }
  }
  *class_index = best_index;
  /* The output tensor uses scale 1/256 and zero point -128. */
  *confidence_percent = static_cast<uint8_t>(
      (static_cast<int>(scores[best_index]) + 128) * 100 / 256);
  return 1U;
}

extern "C" uint8_t GestureInference_GetScores(int8_t* scores) {
  if (interpreter == nullptr || output_tensor == nullptr || scores == nullptr) {
    return 0U;
  }
  for (int i = 0; i < kClassCount; ++i) {
    scores[i] = output_tensor->data.int8[i];
  }
  return 1U;
}

extern "C" uint32_t GestureInference_GetArenaUsed(void) {
  if (interpreter == nullptr) {
    return 0U;
  }
  return static_cast<uint32_t>(interpreter->arena_used_bytes());
}
