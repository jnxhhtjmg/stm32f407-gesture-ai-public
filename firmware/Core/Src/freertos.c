/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Application tasks for the gesture recognition demo.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "mpu6050.h"
#include "gesture_preprocess.h"
#include "gesture_event.h"
#include "gesture_inference.h"
#include "oled_ssd1306.h"
#include "gesture_proto.h"
#include "app_config.h"
#include "app_events.h"
#include "uart_tx.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static GestureEvent gesture_event;
static MPU6050_RawData imu_baseline;
static MPU6050_RawData downsampled_sequence[APP_SEQUENCE_SAMPLES];
static int8_t gesture_input[GESTURE_WINDOW_SIZE * GESTURE_FEATURE_COUNT];
static osThreadId_t inferenceTaskHandle;
static osMutexId_t uart_mutex;

static uint8_t inference_ready = 0U;
static uint8_t imu_ready = 0U;
static uint8_t oled_ready = 0U;

/* Written by the TIM6 ISR, consumed by StartDefaultTask. */
static volatile uint32_t s_sample_tick;
static volatile uint8_t s_sample_pending;
static volatile uint32_t s_sample_overruns;

/* Command channel (AA 55 + func + len + payload + CRC16). */
static uint8_t g_proto_capture_req = 0xFFU;  /* 0xFF = none, 0 = stop, 1 = start */
static uint8_t g_tx_frame[PROTO_MAX_FRAME];

static uint32_t s_imu_error_count;
static uint32_t s_imu_last_retry;

static void Proto_HandleCommands(uint8_t *capture_active);
static void Proto_SendStatus(uint8_t capture_active);
static void Proto_SendAck(uint8_t func, uint8_t code);
static void Proto_SendFrame(uint8_t func, const uint8_t *payload, uint8_t len);

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = APP_STACK_DEFAULT_BYTES,
  .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t inferenceTask_attributes = {
  .name = "inferenceTask",
  .stack_size = APP_STACK_INFERENCE_BYTES,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Debug_SendText(const char *text);
static void Debug_SendWhoAmI(uint8_t who_am_i);
static void Debug_SendClock(void);
static void Debug_SendStack(uint32_t free_bytes);
static void Debug_SendLatency(uint32_t cycles);
static void Debug_EnableCycleCounter(void);
static void Debug_ScanI2C2(void);
static void Debug_ScanI2C1(void);
static void Debug_SendBuffer(const char *buffer, uint16_t length);
static char *Debug_AppendText(char *cursor, const char *text);
static char *Debug_AppendU32(char *cursor, uint32_t value);
static char *Debug_AppendI32(char *cursor, int32_t value);
static char *Debug_AppendHex8(char *cursor, uint8_t value);

static uint8_t Imu_InitAndCalibrate(int16_t *bias_x, int16_t *bias_y,
                                    int16_t *bias_z,
                                    MPU6050_RawData *baseline);
static uint8_t Imu_ReadAccelBaseline(MPU6050_RawData *baseline);
static void Capture_Start(void);
static void Capture_Stop(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartInferenceTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  uart_mutex = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  inferenceTaskHandle = osThreadNew(StartInferenceTask, NULL,
                                    &inferenceTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Application */

/* ==========================================================================
 * 100 Hz sample tick
 * ========================================================================== */
void App_OnSampleTickFromIsr(void)
{
  BaseType_t woken = pdFALSE;

  s_sample_tick = HAL_GetTick();
  if (s_sample_pending != 0U)
  {
    /* The previous tick has not been consumed yet.  Keep the newest timestamp
     * and count the miss instead of building an unbounded backlog. */
    ++s_sample_overruns;
  }
  s_sample_pending = 1U;

  if (defaultTaskHandle != NULL)
  {
    vTaskNotifyGiveFromISR((TaskHandle_t)defaultTaskHandle, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

/* ==========================================================================
 * Command channel
 *   下行（设备 -> PC）: 0x01 状态, 0x7F 应答
 *   上行（PC -> 设备）: 0x81 查询状态, 0x87 运行控制
 *   帧: AA 55 | FUNC | LEN | PAYLOAD | CRC16_L CRC16_H
 * ========================================================================== */
static void Proto_SendFrame(uint8_t func, const uint8_t *payload, uint8_t len)
{
  uint16_t total = Proto_Pack(g_tx_frame, func, payload, len);
  if (total > 0U)
  {
    Debug_SendBuffer((const char *)g_tx_frame, total);
  }
}

static void Proto_SendAck(uint8_t func, uint8_t code)
{
  uint8_t payload[2];
  payload[0] = code;
  payload[1] = func;
  Proto_SendFrame(PROTO_F_ACK, payload, 2U);
}

static void Proto_SendStatus(uint8_t capture_active)
{
  uint8_t  payload[24];
  uint16_t i = 0U;
  uint32_t up = HAL_GetTick();
  uint32_t stack_free = (uint32_t)osThreadGetStackSpace(defaultTaskHandle);
  uint32_t overruns = s_sample_overruns;
  uint32_t uart_drops = UartTx_GetDropped();
  uint32_t imu_errors = s_imu_error_count;

  payload[i++] = capture_active;
  payload[i++] = inference_ready;
  payload[i++] = imu_ready;
  payload[i++] = (uint8_t)(up & 0xFFU);
  payload[i++] = (uint8_t)((up >> 8) & 0xFFU);
  payload[i++] = (uint8_t)((up >> 16) & 0xFFU);
  payload[i++] = (uint8_t)((up >> 24) & 0xFFU);
  payload[i++] = (uint8_t)(g_proto_stats.frames_ok & 0xFFU);
  payload[i++] = (uint8_t)((g_proto_stats.frames_ok >> 8) & 0xFFU);
  payload[i++] = (uint8_t)(g_proto_stats.frames_bad & 0xFFU);
  payload[i++] = (uint8_t)((g_proto_stats.frames_bad >> 8) & 0xFFU);
  payload[i++] = (uint8_t)(stack_free & 0xFFU);
  payload[i++] = (uint8_t)((stack_free >> 8) & 0xFFU);
  payload[i++] = (uint8_t)(overruns & 0xFFU);
  payload[i++] = (uint8_t)((overruns >> 8) & 0xFFU);
  payload[i++] = (uint8_t)(uart_drops & 0xFFU);
  payload[i++] = (uint8_t)((uart_drops >> 8) & 0xFFU);
  payload[i++] = (uint8_t)(imu_errors & 0xFFU);
  payload[i++] = (uint8_t)((imu_errors >> 8) & 0xFFU);
  payload[i++] = oled_ready;
  payload[i++] = (uint8_t)gesture_event.state;

  Proto_SendFrame(PROTO_F_STATUS, payload, (uint8_t)i);
}

static void Proto_HandleCommands(uint8_t *capture_active)
{
  uint8_t        func = 0U;
  uint8_t        len = 0U;
  const uint8_t *payload = 0;

  while (Proto_Pop(&func, &payload, &len) != 0)
  {
    switch (func)
    {
      case PROTO_F_REQ_STATUS:
        Proto_SendStatus((capture_active != 0) ? *capture_active : 0U);
        break;

      case PROTO_F_CTRL:
        if (len < 1U)
        {
          Proto_SendAck(func, PROTO_ACK_BAD_PARAM);
        }
        else if (payload[0] == PROTO_CTRL_CAPTURE_START)
        {
          g_proto_capture_req = 1U;   /* ack is sent after the mode switch */
        }
        else if (payload[0] == PROTO_CTRL_CAPTURE_STOP)
        {
          g_proto_capture_req = 0U;
        }
        else if (payload[0] == PROTO_CTRL_RESET)
        {
          Proto_StatsReset();
          Proto_SendAck(func, PROTO_ACK_OK);
        }
        else
        {
          Proto_SendAck(func, PROTO_ACK_BAD_PARAM);
        }
        break;

      default:
        Proto_SendAck(func, PROTO_ACK_BAD_FUNC);
        break;
    }
  }
}

/* ==========================================================================
 * Acquisition helpers
 * ========================================================================== */
static uint8_t Imu_ReadAccelBaseline(MPU6050_RawData *baseline)
{
  MPU6050_RawData sample;
  int32_t ax = 0;
  int32_t ay = 0;
  int32_t az = 0;
  uint16_t index;

  if (baseline == 0)
  {
    return 0U;
  }
  for (index = 0U; index < 50U; ++index)
  {
    if (MPU6050_ReadRaw(&hi2c2, &sample) != HAL_OK)
    {
      return 0U;
    }
    ax += sample.accel_x;
    ay += sample.accel_y;
    az += sample.accel_z;
    HAL_Delay(5U);
  }
  baseline->accel_x = (int16_t)(ax / 50);
  baseline->accel_y = (int16_t)(ay / 50);
  baseline->accel_z = (int16_t)(az / 50);
  baseline->gyro_x = 0;
  baseline->gyro_y = 0;
  baseline->gyro_z = 0;
  return 1U;
}

static uint8_t Imu_InitAndCalibrate(int16_t *bias_x, int16_t *bias_y,
                                    int16_t *bias_z,
                                    MPU6050_RawData *baseline)
{
  uint8_t who_am_i = 0U;

  if (MPU6050_Init(&hi2c2) != HAL_OK)
  {
    Debug_SendText("MPU6050 init failed\r\n");
    return 0U;
  }
  if (MPU6050_ReadWhoAmI(&hi2c2, &who_am_i) != HAL_OK)
  {
    Debug_SendText("MPU6050 WHO_AM_I read failed\r\n");
    return 0U;
  }

  Debug_SendWhoAmI(who_am_i);
  if (who_am_i != MPU6050_WHO_AM_I_VALUE)
  {
    Debug_SendText("MPU6050 unexpected WHO_AM_I\r\n");
    return 0U;
  }

  Debug_SendText("gyro calibration: keep sensor still\r\n");
  if (MPU6050_CalibrateGyro(&hi2c2,
                            APP_GYRO_CAL_SAMPLES,
                            APP_GYRO_CAL_INTERVAL_MS,
                            APP_GYRO_CAL_DISCARD,
                            bias_x, bias_y, bias_z) != HAL_OK)
  {
    Debug_SendText("gyro calibration failed\r\n");
    return 0U;
  }

  {
    char message[64];
    char *cursor = message;
    cursor = Debug_AppendText(cursor, "gyro_bias,");
    cursor = Debug_AppendI32(cursor, *bias_x);
    *cursor++ = ',';
    cursor = Debug_AppendI32(cursor, *bias_y);
    *cursor++ = ',';
    cursor = Debug_AppendI32(cursor, *bias_z);
    *cursor++ = '\r';
    *cursor++ = '\n';
    *cursor = '\0';
    Debug_SendBuffer(message, (uint16_t)(cursor - message));
  }

  if (Imu_ReadAccelBaseline(baseline) == 0U)
  {
    Debug_SendText("accel baseline failed\r\n");
    return 0U;
  }
  return 1U;
}

static void Capture_Start(void)
{
  Debug_SendText("capture,start\r\n");
  Debug_SendText("timestamp,ax,ay,az,gx,gy,gz\r\n");
  GestureEvent_Reset(&gesture_event);
}

static void Capture_Stop(void)
{
  Debug_SendText("capture,stop\r\n");
  GestureEvent_Reset(&gesture_event);
}

/* ==========================================================================
 * Default task: 100 Hz acquisition + command handling + button
 * ========================================================================== */
void StartDefaultTask(void *argument)
{
  MPU6050_RawData imu_data;
  int16_t gyro_bias_x = 0;
  int16_t gyro_bias_y = 0;
  int16_t gyro_bias_z = 0;
  uint32_t sample_index = 0U;
  uint32_t last_timestamp = 0U;
  char message[96];
  uint8_t capture_active = 0U;
  GPIO_PinState stable_button = HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
  GPIO_PinState last_button = stable_button;
  uint32_t last_button_change = HAL_GetTick();

  (void)argument;

  memset(&imu_baseline, 0, sizeof(imu_baseline));
  memset(&gesture_event, 0, sizeof(gesture_event));
  inference_ready = 0U;
  oled_ready = 0U;

  Debug_SendText(APP_FIRMWARE_NAME " v" APP_FIRMWARE_VERSION " started\r\n");
  Debug_SendClock();

  imu_ready = Imu_InitAndCalibrate(&gyro_bias_x, &gyro_bias_y, &gyro_bias_z,
                                   &imu_baseline);
  if (imu_ready != 0U)
  {
    GestureEvent_Init(&gesture_event, &imu_baseline);
  }
  else
  {
    Debug_ScanI2C2();
    s_imu_last_retry = HAL_GetTick();
  }

  if (GestureInference_Init() != 0U)
  {
    inference_ready = 1U;
    Debug_SendText("gesture inference ready\r\n");
  }
  else
  {
    Debug_SendText("gesture inference init failed\r\n");
  }

#if APP_ENABLE_OLED
  if (OLED_Init(&hi2c1) != 0U)
  {
    oled_ready = 1U;
    OLED_ShowGesture("neutral", 100U);
    Debug_SendText("oled ready\r\n");
  }
  else
  {
    Debug_SendText("oled,no_device\r\n");
    Debug_ScanI2C1();
  }
#endif

  Debug_EnableCycleCounter();

#if APP_RUN_BOOT_BENCHMARK
  if (inference_ready != 0U)
  {
    int bench_class = -1;
    uint8_t bench_confidence = 0U;
    uint32_t wall_start;
    uint32_t wall_end;
    uint32_t cycle_start;
    uint32_t cycle_end;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    char *cursor = message;

    memset(gesture_input, 0, sizeof(gesture_input));
    wall_start = HAL_GetTick();
    cycle_start = DWT->CYCCNT;
    (void)GestureInference_Run(gesture_input, &bench_class, &bench_confidence);
    cycle_end = DWT->CYCCNT;
    wall_end = HAL_GetTick();

    cursor = Debug_AppendText(cursor, "bench_ms,");
    cursor = Debug_AppendU32(cursor, wall_end - wall_start);
    cursor = Debug_AppendText(cursor, ",bench_us,");
    cursor = Debug_AppendU32(cursor,
                             (cycle_end - cycle_start) /
                                 (cycles_per_us != 0U ? cycles_per_us : 1U));
    cursor = Debug_AppendText(cursor, ",arena_used,");
    cursor = Debug_AppendU32(cursor, GestureInference_GetArenaUsed());
    *cursor++ = '\r';
    *cursor++ = '\n';
    *cursor = '\0';
    Debug_SendBuffer(message, (uint16_t)(cursor - message));
  }
#endif

  Debug_SendStack(osThreadGetStackSpace(defaultTaskHandle));

  /* Start the 100 Hz hardware tick only after the scheduler is running. */
  (void)HAL_TIM_Base_Start_IT(&htim6);
  Debug_SendText("sampling,100\r\n");

  for (;;)
  {
    uint32_t now = HAL_GetTick();
    uint32_t sample_tick = 0U;
    uint8_t have_sample = 0U;
    GPIO_PinState raw_button;

    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));

    taskENTER_CRITICAL();
    if (s_sample_pending != 0U)
    {
      sample_tick = s_sample_tick;
      s_sample_pending = 0U;
      have_sample = 1U;
    }
    taskEXIT_CRITICAL();

    /* Command channel. */
    Proto_HandleCommands(&capture_active);
    if (g_proto_capture_req != 0xFFU)
    {
      uint8_t want = g_proto_capture_req;
      g_proto_capture_req = 0xFFU;
      if ((want != 0U) && (imu_ready == 0U))
      {
        Proto_SendAck(PROTO_F_CTRL, PROTO_ACK_BUSY);
      }
      else if (want != capture_active)
      {
        capture_active = want;
        if (want != 0U)
        {
          Capture_Start();
        }
        else
        {
          Capture_Stop();
        }
        GestureEvent_Reset(&gesture_event);
        Proto_SendAck(PROTO_F_CTRL, PROTO_ACK_OK);
      }
    }

    /* Button: KEY0 is active low, debounced in task context. */
    raw_button = HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
    if (raw_button != last_button)
    {
      last_button = raw_button;
      last_button_change = now;
    }
    if ((now - last_button_change) >= APP_BUTTON_DEBOUNCE_MS &&
        raw_button != stable_button)
    {
      stable_button = raw_button;
      if (stable_button == GPIO_PIN_RESET)
      {
        if ((capture_active == 0U) && (imu_ready == 0U))
        {
          Debug_SendText("capture,unavailable\r\n");
        }
        else
        {
          capture_active = (capture_active == 0U) ? 1U : 0U;
          if (capture_active != 0U)
          {
            Capture_Start();
          }
          else
          {
            Capture_Stop();
          }
        }
      }
    }

    if (have_sample != 0U)
    {
      if (imu_ready != 0U)
      {
        if (MPU6050_ReadRaw(&hi2c2, &imu_data) == HAL_OK)
        {
          s_imu_error_count = 0U;
          imu_data.gyro_x = (int16_t)(imu_data.gyro_x - gyro_bias_x);
          imu_data.gyro_y = (int16_t)(imu_data.gyro_y - gyro_bias_y);
          imu_data.gyro_z = (int16_t)(imu_data.gyro_z - gyro_bias_z);

          if (capture_active != 0U)
          {
            char *cursor = message;
            if (sample_tick <= last_timestamp)
            {
              sample_tick = last_timestamp + 1U;
            }
            last_timestamp = sample_tick;

            cursor = Debug_AppendU32(cursor, sample_tick);
            *cursor++ = ',';
            cursor = Debug_AppendI32(cursor, imu_data.accel_x);
            *cursor++ = ',';
            cursor = Debug_AppendI32(cursor, imu_data.accel_y);
            *cursor++ = ',';
            cursor = Debug_AppendI32(cursor, imu_data.accel_z);
            *cursor++ = ',';
            cursor = Debug_AppendI32(cursor, imu_data.gyro_x);
            *cursor++ = ',';
            cursor = Debug_AppendI32(cursor, imu_data.gyro_y);
            *cursor++ = ',';
            cursor = Debug_AppendI32(cursor, imu_data.gyro_z);
            *cursor++ = '\r';
            *cursor++ = '\n';
            *cursor = '\0';
            Debug_SendBuffer(message, (uint16_t)(cursor - message));
          }
          else
          {
            if ((sample_index % APP_IDLE_IMU_LOG_EVERY) == 0U)
            {
              char *cursor = message;
              cursor = Debug_AppendText(cursor, "imu,");
              cursor = Debug_AppendI32(cursor, imu_data.accel_x);
              *cursor++ = ',';
              cursor = Debug_AppendI32(cursor, imu_data.accel_y);
              *cursor++ = ',';
              cursor = Debug_AppendI32(cursor, imu_data.accel_z);
              *cursor++ = ',';
              cursor = Debug_AppendI32(cursor, imu_data.gyro_x);
              *cursor++ = ',';
              cursor = Debug_AppendI32(cursor, imu_data.gyro_y);
              *cursor++ = ',';
              cursor = Debug_AppendI32(cursor, imu_data.gyro_z);
              *cursor++ = '\r';
              *cursor++ = '\n';
              *cursor = '\0';
              Debug_SendBuffer(message, (uint16_t)(cursor - message));
            }

            if (inference_ready != 0U)
            {
              uint8_t event_flags = GestureEvent_Update(&gesture_event, &imu_data);

              if ((event_flags & GESTURE_EVENT_FLAG_ONSET) != 0U)
              {
#if APP_DEBUG_EVENTS
                Debug_SendText("event,onset\r\n");
#endif
              }
              if ((event_flags & GESTURE_EVENT_FLAG_READY) != 0U)
              {
#if APP_DEBUG_EVENTS
                Debug_SendText("event,ready\r\n");
#endif
                GestureEvent_SetState(&gesture_event, GESTURE_EVENT_INFER);
                (void)osThreadFlagsSet(inferenceTaskHandle, 1U);
              }
              else if ((event_flags & GESTURE_EVENT_FLAG_RELEASED) != 0U)
              {
                Debug_SendText("gesture,neutral,100\r\n");
#if APP_ENABLE_OLED
                if (oled_ready != 0U)
                {
                  OLED_ShowGesture("neutral", 100U);
                }
#endif
              }
            }
          }
          ++sample_index;
        }
        else
        {
          ++s_imu_error_count;
          if (s_imu_error_count >= 10U)
          {
            imu_ready = 0U;
            s_imu_last_retry = now;
            Debug_SendText("imu,offline\r\n");
            if (capture_active != 0U)
            {
              capture_active = 0U;
              Capture_Stop();
            }
          }
        }
      }
    }

    /* Retry a missing sensor, but never while a capture is running. */
    if ((imu_ready == 0U) && (capture_active == 0U) &&
        ((now - s_imu_last_retry) >= APP_IMU_RETRY_MS))
    {
      s_imu_last_retry = now;
      imu_ready = Imu_InitAndCalibrate(&gyro_bias_x, &gyro_bias_y,
                                       &gyro_bias_z, &imu_baseline);
      if (imu_ready != 0U)
      {
        Debug_SendText("imu,online\r\n");
        GestureEvent_Init(&gesture_event, &imu_baseline);
      }
    }

    /* In case a DMA restart ever failed, retry from task context. */
    UartTx_Kick();
  }
}

/* ==========================================================================
 * Inference task
 * ========================================================================== */
void StartInferenceTask(void *argument)
{
  char message[96];
  uint32_t inference_count = 0U;

  (void)argument;

  for (;;)
  {
    (void)osThreadFlagsWait(1U, osFlagsWaitAny, osWaitForever);

    if (gesture_event.state != GESTURE_EVENT_INFER)
    {
      continue;
    }

    {
      const MPU6050_RawData *raw = GestureEvent_GetSamples(&gesture_event);
      uint16_t count = GestureEvent_GetCount(&gesture_event);
      int class_index = -1;
      uint8_t confidence = 0U;
      uint32_t elapsed_cycles = 0U;
      uint8_t inference_ok = 0U;

      if ((raw != 0) && (count >= APP_SEQUENCE_RAW_SAMPLES))
      {
        uint16_t index;
        uint32_t start_cycles;

        for (index = 0U; index < APP_SEQUENCE_SAMPLES; ++index)
        {
          const MPU6050_RawData *a = &raw[index * APP_SEQUENCE_DOWNSAMPLE];
          const MPU6050_RawData *b = &raw[index * APP_SEQUENCE_DOWNSAMPLE + 1U];

          downsampled_sequence[index].accel_x =
              (int16_t)(((int32_t)a->accel_x + b->accel_x) / 2);
          downsampled_sequence[index].accel_y =
              (int16_t)(((int32_t)a->accel_y + b->accel_y) / 2);
          downsampled_sequence[index].accel_z =
              (int16_t)(((int32_t)a->accel_z + b->accel_z) / 2);
          downsampled_sequence[index].gyro_x =
              (int16_t)(((int32_t)a->gyro_x + b->gyro_x) / 2);
          downsampled_sequence[index].gyro_y =
              (int16_t)(((int32_t)a->gyro_y + b->gyro_y) / 2);
          downsampled_sequence[index].gyro_z =
              (int16_t)(((int32_t)a->gyro_z + b->gyro_z) / 2);
        }

        Gesture_PreprocessWindow(downsampled_sequence, gesture_input);
        start_cycles = DWT->CYCCNT;
        inference_ok = GestureInference_Run(gesture_input, &class_index,
                                            &confidence);
        elapsed_cycles = DWT->CYCCNT - start_cycles;

#if APP_LIFT_AZ_RULE
        if (inference_ok != 0U)
        {
          int32_t az_sum = 0;
          uint16_t index;

          for (index = 0U; index < APP_LIFT_AZ_WINDOW; ++index)
          {
            az_sum += raw[index].accel_z;
          }

          if ((az_sum / (int32_t)APP_LIFT_AZ_WINDOW -
               (int32_t)imu_baseline.accel_z) > APP_LIFT_AZ_DELTA)
          {
            class_index = 0;  /* lift, regardless of the model's class */
            if (confidence < APP_LIFT_MIN_CONFIDENCE)
            {
              confidence = APP_LIFT_MIN_CONFIDENCE;
            }
          }
        }
#endif

#if APP_TILT_GZ_RULE
        if ((inference_ok != 0U) &&
            ((class_index == 4) || (class_index == 5)))
        {
          int32_t gz_sum = 0;
          uint16_t index;
          const int32_t threshold =
              (int32_t)APP_TILT_GZ_WINDOW * APP_TILT_GZ_MIN_DPS * 131;

          for (index = 0U; index < APP_TILT_GZ_WINDOW; ++index)
          {
            gz_sum += raw[index].gyro_z;
          }
          if (gz_sum < -threshold)
          {
            class_index = 4;
          }
          else if (gz_sum > threshold)
          {
            class_index = 5;
          }
        }
#endif
      }

      if (inference_ok != 0U)
      {
        Debug_SendLatency(elapsed_cycles);

        {
          char *cursor = message;
          cursor = Debug_AppendText(cursor, "gesture,");
          cursor = Debug_AppendText(cursor, gesture_class_names[class_index]);
          *cursor++ = ',';
          cursor = Debug_AppendU32(cursor, (uint32_t)confidence);
          *cursor++ = '\r';
          *cursor++ = '\n';
          *cursor = '\0';
          Debug_SendBuffer(message, (uint16_t)(cursor - message));
        }
#if APP_ENABLE_OLED
        if (oled_ready != 0U)
        {
          OLED_ShowGesture(gesture_class_names[class_index], confidence);
        }
#endif
        GestureEvent_SetState(&gesture_event, GESTURE_EVENT_LATCHED);
      }
      else
      {
        Debug_SendText("gesture,inference_failed\r\n");
        GestureEvent_Reset(&gesture_event);
      }

      ++inference_count;
      if ((inference_count % 20U) == 0U)
      {
        Debug_SendStack(osThreadGetStackSpace(inferenceTaskHandle));
      }
    }
  }
}

/* ==========================================================================
 * Small non-blocking logging helpers
 * ========================================================================== */
static void Debug_SendText(const char *text)
{
#if APP_UART_TX_DMA
  (void)UartTx_WriteString(text);
#else
  uint16_t length = 0U;
  while (text[length] != '\0')
  {
    ++length;
  }
  if (uart_mutex != NULL)
  {
    (void)osMutexAcquire(uart_mutex, osWaitForever);
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, length, 100U);
  if (uart_mutex != NULL)
  {
    (void)osMutexRelease(uart_mutex);
  }
#endif
}

static void Debug_SendBuffer(const char *buffer, uint16_t length)
{
#if APP_UART_TX_DMA
  (void)UartTx_Write(buffer, length);
#else
  if (uart_mutex != NULL)
  {
    (void)osMutexAcquire(uart_mutex, osWaitForever);
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, 100U);
  if (uart_mutex != NULL)
  {
    (void)osMutexRelease(uart_mutex);
  }
#endif
}

static char *Debug_AppendText(char *cursor, const char *text)
{
  while (*text != '\0')
  {
    *cursor++ = *text++;
  }
  return cursor;
}

static char *Debug_AppendU32(char *cursor, uint32_t value)
{
  char digits[10];
  uint32_t count = 0U;

  if (value == 0U)
  {
    *cursor++ = '0';
    return cursor;
  }
  while (value > 0U && count < (uint32_t)sizeof(digits))
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while (count > 0U)
  {
    *cursor++ = digits[--count];
  }
  return cursor;
}

static char *Debug_AppendI32(char *cursor, int32_t value)
{
  if (value < 0)
  {
    uint32_t magnitude = (uint32_t)(-(value + 1)) + 1U;
    *cursor++ = '-';
    return Debug_AppendU32(cursor, magnitude);
  }
  return Debug_AppendU32(cursor, (uint32_t)value);
}

static char *Debug_AppendHex8(char *cursor, uint8_t value)
{
  static const char hex[] = "0123456789ABCDEF";
  *cursor++ = hex[(value >> 4) & 0x0FU];
  *cursor++ = hex[value & 0x0FU];
  return cursor;
}

static void Debug_SendWhoAmI(uint8_t who_am_i)
{
  char message[] = "MPU6050 WHO_AM_I=0x00\r\n";
  message[19] = "0123456789ABCDEF"[(who_am_i >> 4) & 0x0FU];
  message[20] = "0123456789ABCDEF"[who_am_i & 0x0FU];
  Debug_SendText(message);
}

static void Debug_SendClock(void)
{
  char message[48];
  uint32_t pll_source = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE;
  char *cursor = message;

  cursor = Debug_AppendText(cursor, "clock,");
  cursor = Debug_AppendText(cursor, (pll_source != 0U) ? "hse" : "hsi");
  *cursor++ = ',';
  cursor = Debug_AppendU32(cursor, HAL_RCC_GetSysClockFreq());
  *cursor++ = '\r';
  *cursor++ = '\n';
  *cursor = '\0';
  Debug_SendBuffer(message, (uint16_t)(cursor - message));
}

static void Debug_SendStack(uint32_t free_bytes)
{
  char message[32];
  char *cursor = message;

  cursor = Debug_AppendText(cursor, "stack_free,");
  cursor = Debug_AppendU32(cursor, free_bytes);
  *cursor++ = '\r';
  *cursor++ = '\n';
  *cursor = '\0';
  Debug_SendBuffer(message, (uint16_t)(cursor - message));
}

static void Debug_SendLatency(uint32_t cycles)
{
  char message[32];
  uint32_t cycles_per_us = (uint32_t)(SystemCoreClock / 1000000U);
  char *cursor = message;

  if (cycles_per_us == 0U)
  {
    cycles_per_us = 1U;
  }
  cursor = Debug_AppendText(cursor, "infer_us,");
  cursor = Debug_AppendU32(cursor, cycles / cycles_per_us);
  *cursor++ = '\r';
  *cursor++ = '\n';
  *cursor = '\0';
  Debug_SendBuffer(message, (uint16_t)(cursor - message));
}

static void Debug_ScanI2C1(void)
{
  char message[32];
  uint8_t address;
  uint8_t found = 0U;

  for (address = 0x08U; address <= 0x77U; ++address)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 1U, 5U) == HAL_OK)
    {
      char *cursor = message;
      cursor = Debug_AppendText(cursor, "i2c1,0x");
      cursor = Debug_AppendHex8(cursor, address);
      *cursor++ = '\r';
      *cursor++ = '\n';
      *cursor = '\0';
      Debug_SendBuffer(message, (uint16_t)(cursor - message));
      ++found;
    }
  }

  if (found == 0U)
  {
    Debug_SendText("i2c1,no_device\r\n");
  }
}

static void Debug_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Debug_ScanI2C2(void)
{
  char message[32];
  uint8_t address;
  uint8_t found = 0U;

  for (address = 0x08U; address <= 0x77U; ++address)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(address << 1), 1U, 5U) == HAL_OK)
    {
      char *cursor = message;
      cursor = Debug_AppendText(cursor, "i2c,0x");
      cursor = Debug_AppendHex8(cursor, address);
      *cursor++ = '\r';
      *cursor++ = '\n';
      *cursor = '\0';
      Debug_SendBuffer(message, (uint16_t)(cursor - message));
      ++found;
    }
  }

  if (found == 0U)
  {
    Debug_SendText("i2c,no_device\r\n");
  }
}

/* USER CODE END Application */
