#ifndef GESTURE_PREPROCESS_H
#define GESTURE_PREPROCESS_H

#include <stdint.h>
#include "mpu6050.h"

#define GESTURE_WINDOW_SIZE 100U
#define GESTURE_FEATURE_COUNT 6U
#define GESTURE_CLASS_COUNT 6U

/* Must match data/processed_physical_6class/metadata.json. */
extern const char *const gesture_class_names[GESTURE_CLASS_COUNT];

void Gesture_PreprocessWindow(const MPU6050_RawData *raw_samples,
                              int8_t *quantized_output);

#endif /* GESTURE_PREPROCESS_H */
