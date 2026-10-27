#include "mpu6050.h"

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_WHO_AM_I     0x75U

#define MPU6050_I2C_TIMEOUT_MS   100U
#define MPU6050_READ_RETRIES     3U

static HAL_StatusTypeDef MPU6050_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                uint8_t register_address,
                                                uint8_t value)
{
  return HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDRESS, register_address,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           MPU6050_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *who_am_i)
{
  if ((hi2c == NULL) || (who_am_i == NULL))
  {
    return HAL_ERROR;
  }
  return HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDRESS, MPU6050_REG_WHO_AM_I,
                          I2C_MEMADD_SIZE_8BIT, who_am_i, 1U,
                          MPU6050_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* Full device reset, then wake up and use the gyro PLL as clock source.
   * The PLL is noticeably more stable than the internal 8 MHz oscillator. */
  status = MPU6050_WriteRegister(hi2c, MPU6050_REG_PWR_MGMT_1, 0x80U);
  if (status != HAL_OK) return status;
  HAL_Delay(100U);

  status = MPU6050_WriteRegister(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01U);
  if (status != HAL_OK) return status;
  HAL_Delay(10U);

  /* 100 Hz output rate with the DLPF enabled: 1 kHz / (9 + 1). */
  status = MPU6050_WriteRegister(hi2c, MPU6050_REG_SMPLRT_DIV, 0x09U);
  if (status != HAL_OK) return status;

  /* DLPF 44 Hz, gyro +/-250 dps, accelerometer +/-2 g. */
  status = MPU6050_WriteRegister(hi2c, MPU6050_REG_CONFIG, 0x03U);
  if (status != HAL_OK) return status;
  status = MPU6050_WriteRegister(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00U);
  if (status != HAL_OK) return status;
  return MPU6050_WriteRegister(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00U);
}

HAL_StatusTypeDef MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData *data)
{
  uint8_t buffer[14];
  HAL_StatusTypeDef status;
  uint8_t attempt;

  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  for (attempt = 0U; attempt < MPU6050_READ_RETRIES; ++attempt)
  {
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDRESS,
                              MPU6050_REG_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT, buffer, sizeof(buffer),
                              MPU6050_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      break;
    }
  }
  if (status != HAL_OK)
  {
    return status;
  }

  data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
  data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
  data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
  data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
  data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
  data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c,
                                        uint16_t sample_count,
                                        uint32_t sample_interval_ms,
                                        uint16_t discard_samples,
                                        int16_t *bias_x,
                                        int16_t *bias_y,
                                        int16_t *bias_z)
{
  MPU6050_RawData data;
  int32_t sum_x = 0;
  int32_t sum_y = 0;
  int32_t sum_z = 0;
  uint32_t total_samples;
  uint32_t index;

  if ((hi2c == NULL) || (bias_x == NULL) || (bias_y == NULL) ||
      (bias_z == NULL) || (sample_count == 0U))
  {
    return HAL_ERROR;
  }

  total_samples = (uint32_t)discard_samples + (uint32_t)sample_count;
  for (index = 0U; index < total_samples; ++index)
  {
    if (MPU6050_ReadRaw(hi2c, &data) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (index >= discard_samples)
    {
      sum_x += data.gyro_x;
      sum_y += data.gyro_y;
      sum_z += data.gyro_z;
    }
    HAL_Delay(sample_interval_ms);
  }

  *bias_x = (int16_t)(sum_x / (int32_t)sample_count);
  *bias_y = (int16_t)(sum_y / (int32_t)sample_count);
  *bias_z = (int16_t)(sum_z / (int32_t)sample_count);
  return HAL_OK;
}
