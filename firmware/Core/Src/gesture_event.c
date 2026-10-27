#include "gesture_event.h"

#include "app_config.h"

#define ACCEL_LSB_PER_G   16384.0f
#define GYRO_LSB_PER_DPS  131.0f

static uint8_t sample_is_moving(const GestureEvent *event,
                                const MPU6050_RawData *sample)
{
  const float dx = ((float)sample->accel_x - (float)event->baseline.accel_x) /
                   ACCEL_LSB_PER_G;
  const float dy = ((float)sample->accel_y - (float)event->baseline.accel_y) /
                   ACCEL_LSB_PER_G;
  const float dz = ((float)sample->accel_z - (float)event->baseline.accel_z) /
                   ACCEL_LSB_PER_G;
  const float gx = (float)sample->gyro_x / GYRO_LSB_PER_DPS;
  const float gy = (float)sample->gyro_y / GYRO_LSB_PER_DPS;
  const float gz = (float)sample->gyro_z / GYRO_LSB_PER_DPS;
  const float accel_sq = dx * dx + dy * dy + dz * dz;
  const float gyro_sq = gx * gx + gy * gy + gz * gz;

  return (gyro_sq > (APP_ONSET_GYRO_DPS * APP_ONSET_GYRO_DPS)) ||
         (accel_sq > (APP_ONSET_ACCEL_G * APP_ONSET_ACCEL_G));
}

static uint8_t sample_is_quiet(const GestureEvent *event,
                               const MPU6050_RawData *sample)
{
  const float dx = ((float)sample->accel_x - (float)event->baseline.accel_x) /
                   ACCEL_LSB_PER_G;
  const float dy = ((float)sample->accel_y - (float)event->baseline.accel_y) /
                   ACCEL_LSB_PER_G;
  const float dz = ((float)sample->accel_z - (float)event->baseline.accel_z) /
                   ACCEL_LSB_PER_G;
  const float gx = (float)sample->gyro_x / GYRO_LSB_PER_DPS;
  const float gy = (float)sample->gyro_y / GYRO_LSB_PER_DPS;
  const float gz = (float)sample->gyro_z / GYRO_LSB_PER_DPS;
  const float accel_sq = dx * dx + dy * dy + dz * dz;
  const float gyro_sq = gx * gx + gy * gy + gz * gz;

  return (gyro_sq < (APP_RELEASE_GYRO_DPS * APP_RELEASE_GYRO_DPS)) &&
         (accel_sq < (APP_RELEASE_ACCEL_G * APP_RELEASE_ACCEL_G));
}

void GestureEvent_Init(GestureEvent *event, const MPU6050_RawData *baseline)
{
  if ((event == 0) || (baseline == 0))
  {
    return;
  }
  event->state = GESTURE_EVENT_IDLE;
  event->count = 0U;
  event->quiet_count = 0U;
  event->motion_count = 0U;
  event->baseline = *baseline;
}

uint8_t GestureEvent_Update(GestureEvent *event, const MPU6050_RawData *sample)
{
  uint8_t flags = GESTURE_EVENT_FLAG_NONE;
  uint8_t moving;

  if ((event == 0) || (sample == 0))
  {
    return flags;
  }

  moving = sample_is_moving(event, sample);

  if (event->state == GESTURE_EVENT_IDLE)
  {
    uint8_t index;
    for (index = 1U; index < APP_ONSET_CONSECUTIVE; ++index)
    {
      event->pre[index - 1U] = event->pre[index];
    }
    event->pre[APP_ONSET_CONSECUTIVE - 1U] = *sample;
  }

  switch (event->state)
  {
    case GESTURE_EVENT_IDLE:
      if (moving != 0U)
      {
        ++event->motion_count;
        if (event->motion_count >= APP_ONSET_CONSECUTIVE)
        {
          uint8_t index;
          event->state = GESTURE_EVENT_ARMED;
          event->quiet_count = 0U;
          event->count = 0U;
          /* Keep the first sample of the confirmation run, not the fifth, so
           * the window origin matches the offline onset detector. */
          for (index = 0U; index < APP_ONSET_CONSECUTIVE; ++index)
          {
            event->samples[event->count++] = event->pre[index];
          }
          flags |= GESTURE_EVENT_FLAG_ONSET;
        }
      }
      else
      {
        event->motion_count = 0U;
      }
      break;

    case GESTURE_EVENT_ARMED:
      if (event->count < GESTURE_EVENT_BUFFER_SIZE)
      {
        event->samples[event->count++] = *sample;
      }
      if (event->count >= GESTURE_EVENT_READY_SAMPLES)
      {
        event->state = GESTURE_EVENT_READY;
        flags |= GESTURE_EVENT_FLAG_READY;
      }
      break;

    case GESTURE_EVENT_READY:
    case GESTURE_EVENT_INFER:
      /* The buffer is frozen while the inference task consumes it. */
      break;

    case GESTURE_EVENT_LATCHED:
      if (sample_is_quiet(event, sample) != 0U)
      {
        ++event->quiet_count;
        if (event->quiet_count >= APP_RELEASE_CONSECUTIVE)
        {
          event->state = GESTURE_EVENT_IDLE;
          event->motion_count = 0U;
          event->quiet_count = 0U;
          flags |= GESTURE_EVENT_FLAG_RELEASED;
        }
      }
      else
      {
        event->quiet_count = 0U;
      }
      break;

    default:
      event->state = GESTURE_EVENT_IDLE;
      break;
  }

  return flags;
}

const MPU6050_RawData *GestureEvent_GetSamples(const GestureEvent *event)
{
  return (event != 0) ? event->samples : 0;
}

uint16_t GestureEvent_GetCount(const GestureEvent *event)
{
  return (event != 0) ? event->count : 0U;
}

void GestureEvent_SetState(GestureEvent *event, GestureEventState state)
{
  if (event != 0)
  {
    event->state = state;
    if (state == GESTURE_EVENT_LATCHED)
    {
      event->quiet_count = 0U;
    }
  }
}

void GestureEvent_Reset(GestureEvent *event)
{
  if (event != 0)
  {
    event->state = GESTURE_EVENT_IDLE;
    event->count = 0U;
    event->quiet_count = 0U;
    event->motion_count = 0U;
  }
}
