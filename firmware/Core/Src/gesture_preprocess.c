#include "gesture_preprocess.h"

const char *const gesture_class_names[GESTURE_CLASS_COUNT] = {
  "lift",
  "move_left",
  "move_right",
  "neutral",
  "tilt_left",
  "tilt_right"
};

/*
 * Placeholder normalization and quantization values.  The released repository
 * does not ship trained parameters.
 *
 * After training, copy normalization_mean / normalization_std from
 * runs/<name>/report.json and INPUT_SCALE / INPUT_ZERO_POINT from the INT8
 * quantization block.  Until then the firmware still builds and boots, but
 * inference will not be meaningful.
 */
static const float normalization_mean[GESTURE_FEATURE_COUNT] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static const float normalization_std[GESTURE_FEATURE_COUNT] = {
  1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

#define ACCEL_LSB_PER_G 16384.0f
#define GYRO_LSB_PER_DPS 131.0f
/* Optional static X-axis mount trim in g.  Calibrate it for your own board or
 * leave at 0.0f. */
#define ACCEL_X_TRIM_G 0.0f
#define INPUT_SCALE 1.0f
#define INPUT_ZERO_POINT 0

static int8_t Gesture_Quantize(float normalized_value)
{
  float scaled = normalized_value / INPUT_SCALE;
  int32_t integer_value = (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
  integer_value += INPUT_ZERO_POINT;
  if (integer_value > 127) integer_value = 127;
  if (integer_value < -128) integer_value = -128;
  return (int8_t)integer_value;
}

static void Gesture_PreprocessSample(const MPU6050_RawData *raw,
                                     int8_t *output)
{
  const float physical[GESTURE_FEATURE_COUNT] = {
    (float)raw->accel_x / ACCEL_LSB_PER_G + ACCEL_X_TRIM_G,
    (float)raw->accel_y / ACCEL_LSB_PER_G,
    (float)raw->accel_z / ACCEL_LSB_PER_G,
    (float)raw->gyro_x / GYRO_LSB_PER_DPS,
    (float)raw->gyro_y / GYRO_LSB_PER_DPS,
    (float)raw->gyro_z / GYRO_LSB_PER_DPS
  };
  uint32_t feature;
  for (feature = 0U; feature < GESTURE_FEATURE_COUNT; ++feature)
  {
    output[feature] = Gesture_Quantize(
        (physical[feature] - normalization_mean[feature]) /
        normalization_std[feature]);
  }
}

void Gesture_PreprocessWindow(const MPU6050_RawData *raw_samples,
                              int8_t *quantized_output)
{
  uint32_t sample;
  for (sample = 0U; sample < GESTURE_WINDOW_SIZE; ++sample)
  {
    Gesture_PreprocessSample(raw_samples + sample,
                             quantized_output + sample * GESTURE_FEATURE_COUNT);
  }
}

