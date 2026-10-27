#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"

#define MPU6050_I2C_ADDRESS        (0x68U << 1)
#define MPU6050_WHO_AM_I_VALUE     0x68U

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} MPU6050_RawData;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i);
HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData *data);
/**
 * @brief  Average the gyro output to estimate the zero-rate bias.
 * @param  discard_samples  Samples read and discarded first (sensor settling).
 * @param  sample_count     Samples averaged into the bias.
 */
HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c,
                                        uint16_t sample_count,
                                        uint32_t sample_interval_ms,
                                        uint16_t discard_samples,
                                        int16_t *bias_x,
                                        int16_t *bias_y,
                                        int16_t *bias_z);

#endif /* __MPU6050_H */
