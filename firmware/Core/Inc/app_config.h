/**
 * @file    app_config.h
 * @brief   Compile-time configuration for the gesture recognition firmware.
 *
 * Values that affect the sampling pipeline are grouped here so that the
 * acquisition, inference and host-side data formats stay in sync.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ------------------------------------------------------------------------- */
/* Identity                                                                   */
/* ------------------------------------------------------------------------- */
#define APP_FIRMWARE_NAME       "f407_gesture_ai"
#define APP_FIRMWARE_VERSION    "0.3.0"

/* ------------------------------------------------------------------------- */
/* Sampling and buses                                                         */
/* ------------------------------------------------------------------------- */
/* 100 Hz matches the training data and the F103 collector. */
#define APP_SAMPLE_RATE_HZ      100U

/* APB1 timer clock is 84 MHz on this board (168 MHz / 4, timer x2). */
#define APP_TIM6_INPUT_HZ       84000000U
#define APP_TIM6_PRESCALER      ((APP_TIM6_INPUT_HZ / 1000000U) - 1U)
#define APP_TIM6_PERIOD         ((1000000U / APP_SAMPLE_RATE_HZ) - 1U)

#define APP_UART_BAUD_RATE      115200U
/* Temporary: use polling TX until the DMA stream mapping is verified on the
 * board.  Set to 1 to use the ring buffer + DMA1 Stream 4 implementation. */
#define APP_UART_TX_DMA         1
#define APP_I2C_CLOCK_HZ        400000U

/* ------------------------------------------------------------------------- */
/* Buffers and task configuration                                             */
/* ------------------------------------------------------------------------- */
/* Must be a power of two. 4 KiB is ~0.7 s of CSV at 100 Hz. */
#define APP_UART_TX_BUFFER_SIZE 4096U

#define APP_STACK_DEFAULT_BYTES     4096U
#define APP_STACK_INFERENCE_BYTES   6144U
#define APP_TENSOR_ARENA_BYTES      (16U * 1024U)

/* ------------------------------------------------------------------------- */
/* Acquisition behaviour                                                      */
/* ------------------------------------------------------------------------- */
/* In idle mode print one raw IMU line every N samples. */
#define APP_IDLE_IMU_LOG_EVERY  10U

/* Event segmentation.  A gesture is accepted only after the motion onset and
 * is latched until the sensor is quiet again, so the return phase cannot flip
 * the label.  The buffer holds 2 s at 100 Hz and is decimated 2:1 before
 * inference, matching the 2 s / 50 Hz training windows (100x6 input). */
#define APP_SEQUENCE_RAW_SAMPLES    200U
#define APP_SEQUENCE_DOWNSAMPLE     2U
#define APP_SEQUENCE_SAMPLES        (APP_SEQUENCE_RAW_SAMPLES / APP_SEQUENCE_DOWNSAMPLE)

/* The model is trained on the F103 rig.  On the F407 the tilt gesture's first
 * 0.3 s still has an unambiguous gz sign, while the absolute ax offset shifts
 * both tilt classes toward tilt_right.  Use the sign of the first-phase gz as
 * a direction rule when the model reports a tilt class.  Remove this after
 * retraining on F407-collected tilt data. */
#define APP_TILT_GZ_RULE        0
#define APP_TILT_GZ_WINDOW      30U
#define APP_TILT_GZ_MIN_DPS     2

/* On the F407 a "lift" is a vertical acceleration plus pitch/roll, so the
 * model can report a tilt class.  The first 0.3 s of a lift has az well above
 * the boot baseline, while a pure left/right tilt keeps az at ~1 g. */
#define APP_LIFT_AZ_RULE        0
#define APP_LIFT_AZ_WINDOW      30U
#define APP_LIFT_AZ_DELTA       100
#define APP_LIFT_MIN_CONFIDENCE 80U
#define APP_ONSET_GYRO_DPS          3
#define APP_ONSET_ACCEL_G           0.03f
#define APP_RELEASE_GYRO_DPS        2
#define APP_RELEASE_ACCEL_G         0.03f
#define APP_ONSET_CONSECUTIVE       5U
#define APP_RELEASE_CONSECUTIVE     30U

/* MPU6050 gyro zero-rate calibration. */
#define APP_GYRO_CAL_SAMPLES        200U
#define APP_GYRO_CAL_INTERVAL_MS    5U
#define APP_GYRO_CAL_DISCARD        10U

#define APP_BUTTON_DEBOUNCE_MS  30U

/* Retry the sensor every 5 s if it was missing at boot.  Calibration alone
 * blocks for ~1 s, so a shorter interval would waste most of the task budget. */
#define APP_IMU_RETRY_MS        5000U

/* ------------------------------------------------------------------------- */
/* Feature switches                                                           */
/* ------------------------------------------------------------------------- */
#define APP_ENABLE_OLED         1
#define APP_RUN_BOOT_BENCHMARK  1
#define APP_ENABLE_STACK_LOG    1
#define APP_DEBUG_EVENTS        1

#endif /* APP_CONFIG_H */
