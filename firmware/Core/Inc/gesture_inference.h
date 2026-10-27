#ifndef GESTURE_INFERENCE_H
#define GESTURE_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t GestureInference_Init(void);
uint8_t GestureInference_Run(const int8_t *input,
                             int *class_index,
                             uint8_t *confidence_percent);
uint8_t GestureInference_GetScores(int8_t *scores);
uint32_t GestureInference_GetArenaUsed(void);

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_INFERENCE_H */
