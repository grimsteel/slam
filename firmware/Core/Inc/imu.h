#pragma once

#include "stm32f4xx_hal.h"

// 7 bit addr
#define MPU_ADDR (0x68 << 1)

typedef struct {
  I2C_HandleTypeDef* i2c;
  float accel[3];
  float gyro[3];
  float temp;
  uint8_t buf1[14];
  uint8_t buf2[14];
  uint8_t* dma_buf;
  uint8_t* read_buf;
  _Bool ready;
} imu_t;

HAL_StatusTypeDef imu_init(imu_t* imu, I2C_HandleTypeDef* i2c);
/// Call when interrupt hit
void imu_dma_start(imu_t* imu);
/// Call when DMA interrupt hit
void imu_dma_complete(imu_t* imu, I2C_HandleTypeDef* event_i2c);
/// Parse the most recent readings from DMA
void imu_parse_update(imu_t* imu);
HAL_StatusTypeDef imu_debug_regs(imu_t* imu);
