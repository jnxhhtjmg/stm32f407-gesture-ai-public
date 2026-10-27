#ifndef GESTURE_MODEL_DATA_H
#define GESTURE_MODEL_DATA_H

#include <stdint.h>

/*
 * Placeholder.  The released repository does not ship trained weights.
 * Train a model with ../training and regenerate this pair with:
 *
 *   python ../training/generate_model_c_array.py \
 *     <your_model_int8.tflite> <output_dir> \
 *     --header-dir Core/Inc --source-dir Core/Src
 */
extern const unsigned char g_gesture_model_data[];
extern const unsigned int g_gesture_model_data_len;

#define GESTURE_MODEL_SHA256 "not-generated"
#define GESTURE_MODEL_BYTES 0u

#endif
