/**
 * @file    gesture_event.h
 * @brief   Motion-onset segmentation and latch-until-neutral state machine.
 *
 * The model consumes a complete 2 s gesture (200 raw samples at 100 Hz,
 * decimated to 100 samples at 50 Hz).  Running it on an arbitrary sliding
 * window lets the return phase of a round-trip gesture look like the opposite
 * class, which is the main source of left/right flips.  This module freezes a
 * 200-sample buffer at the motion onset and only releases a new gesture after
 * the sensor has been quiet for a while.
 */
#ifndef GESTURE_EVENT_H
#define GESTURE_EVENT_H

#include <stdint.h>

#include "app_config.h"
#include "mpu6050.h"

#define GESTURE_EVENT_BUFFER_SIZE   APP_SEQUENCE_RAW_SAMPLES
#define GESTURE_EVENT_READY_SAMPLES APP_SEQUENCE_RAW_SAMPLES

typedef enum
{
  GESTURE_EVENT_IDLE = 0,
  GESTURE_EVENT_ARMED,
  GESTURE_EVENT_READY,
  GESTURE_EVENT_INFER,
  GESTURE_EVENT_LATCHED
} GestureEventState;

typedef struct
{
  volatile GestureEventState state;
  uint16_t count;
  uint16_t quiet_count;
  uint16_t motion_count;
  MPU6050_RawData baseline;
  MPU6050_RawData pre[APP_ONSET_CONSECUTIVE];
  MPU6050_RawData samples[GESTURE_EVENT_BUFFER_SIZE];
} GestureEvent;

typedef enum
{
  GESTURE_EVENT_FLAG_NONE = 0,
  GESTURE_EVENT_FLAG_ONSET = 1U << 0,
  GESTURE_EVENT_FLAG_READY = 1U << 1,
  GESTURE_EVENT_FLAG_RELEASED = 1U << 2
} GestureEventFlags;

void GestureEvent_Init(GestureEvent *event, const MPU6050_RawData *baseline);

/**
 * @brief Feed one calibrated sample.
 * @return Bit mask of GESTURE_EVENT_FLAG_* transitions.
 */
uint8_t GestureEvent_Update(GestureEvent *event, const MPU6050_RawData *sample);

/** Complete 200-sample sequence captured from the motion onset. */
const MPU6050_RawData *GestureEvent_GetSamples(const GestureEvent *event);
uint16_t GestureEvent_GetCount(const GestureEvent *event);
void GestureEvent_SetState(GestureEvent *event, GestureEventState state);
void GestureEvent_Reset(GestureEvent *event);

#endif /* GESTURE_EVENT_H */
