#include "imu.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "util.h"
#include "SEGGER_RTT.h"

// ms
#define MPU_I2C_TIMEOUT 100

// Hz
#define MPU_OUTPUT_RATE 1000
#define TARGET_SAMPLE_RATE 4
#define SAMPLE_RATE_DIV (MPU_OUTPUT_RATE / TARGET_SAMPLE_RATE - 1)

#define REG_POWER 0x6B
#define REG_SAMPLE_RATE 0x19
#define REG_CFG 0x1A
#define REG_GYRO_CFG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INTERRUPT_CONFIG 0x37
#define REG_INTERRUPT_ENABLE 0x38

#define MPU_2G_LSB_SENS 16384.0f
#define MPU_250_LSB_SENS 131.0f

// First sensor register (accel xout high)
#define REG_SENSORS_START 0x3B
// Accel (6) + Gyro (6) + Temp (2)
#define REG_SENSORS_LEN 14

#define MPU_SET_REG(reg, size, val) HAL_I2C_Mem_Write(imu->i2c, MPU_ADDR, (reg), I2C_MEMADD_SIZE_8BIT, (val), (size), MPU_I2C_TIMEOUT)

HAL_StatusTypeDef imu_init(imu_t* imu, I2C_HandleTypeDef* i2c) {
  ERROR_DEF;
  
  for (int i =0 ; i < 3; i++) {
    imu->accel[i] = 0;
    imu->gyro[i] =0 ;
  }
  imu->dma_buf = imu->buf1;
  imu->read_buf = imu->buf2;

  imu->i2c = i2c;

  CHECK(HAL_I2C_IsDeviceReady(i2c, MPU_ADDR, 3 /* trials */, MPU_I2C_TIMEOUT /* 100 ms timeout */));

  // Wakeup
  uint8_t sleep_reg_wakeup = 0x00;
  CHECK(MPU_SET_REG(REG_POWER, 1, &sleep_reg_wakeup));

  // Batch write registers
  uint8_t mpu_config_values[] = {
    SAMPLE_RATE_DIV,
    0x03, // CONFIG: DLPF = 3 (low pass w/ 4.8 ms delay)
    0x00, // GYRO_CONFIG: FS_SEL = 0 (250 deg/s range)
    0x00, // ACCEL_CONFIG: AFS_SEL = 0 (2g range)
  };

  // Write 4 registers starting at sample rate
  CHECK(MPU_SET_REG(REG_SAMPLE_RATE, 4, mpu_config_values));

  uint8_t interrupt_config_values[] = {
    // Interrupt config:
    // active low (1), open drain (1), latch until clear (1), clear on any read (1),
    // don't care about the 4 lower bits
    0xF0,
    // Interrupt source: only data ready
    0x01
  };
  CHECK(MPU_SET_REG(REG_INTERRUPT_CONFIG, 2, interrupt_config_values));

  imu->ready = 1;
  
  return HAL_OK;
}
/// Call when interrupt hit
void imu_dma_start(imu_t* imu) {
  if (imu->ready) {
    HAL_I2C_Mem_Read_DMA(
      imu->i2c, MPU_ADDR, 
      REG_SENSORS_START, I2C_MEMADD_SIZE_8BIT,
      imu->dma_buf, REG_SENSORS_LEN
    );
  }
}
/// Call when DMA interrupt hit
void imu_dma_complete(imu_t* imu, I2C_HandleTypeDef* event_i2c) {
  if (event_i2c != imu->i2c) return;
  
  // Swap buffers to prevent race on next read
  if (imu->dma_buf == imu->buf1) {
    imu->dma_buf = imu->buf2;
    imu->read_buf = imu->buf1;
  } else {
    imu->dma_buf = imu->buf1;
    imu->read_buf = imu->buf2;
  }
}
/// Parse the most recent readings from DMA
void imu_parse_update(imu_t* imu) {
  int16_t reading;
  for (int i = 0; i < 3; i++) {
    // Accelerometer
    reading = imu->read_buf[i * 2 + 1];
    // upper 8 bits
    reading |= imu->read_buf[i * 2] << 8;

    imu->accel[i] = reading / MPU_2G_LSB_SENS * 9.81;

    // Gyroscope
    reading = imu->read_buf[8 + i * 2 + 1];
    // upper 8 bits
    reading |= imu->read_buf[8 + i * 2] << 8;

    // degrees/s
    imu->gyro[i] = reading / MPU_250_LSB_SENS;
  }
  reading = imu->read_buf[7] | (imu->read_buf[6] << 8);
  // degrees celcius
  imu->temp = reading / 340.0f + 36.53f;
}

HAL_StatusTypeDef imu_debug_regs(imu_t* imu) {
  ERROR_DEF;
  uint8_t read_buf[7];
  CHECK(HAL_I2C_Mem_Read(imu->i2c, MPU_ADDR, I2C_MEMADD_SIZE_8BIT, REG_SAMPLE_RATE, read_buf, 4, MPU_I2C_TIMEOUT));
  CHECK(HAL_I2C_Mem_Read(imu->i2c, MPU_ADDR, I2C_MEMADD_SIZE_8BIT, REG_INTERRUPT_CONFIG, read_buf + 4, 2, MPU_I2C_TIMEOUT));
  CHECK(HAL_I2C_Mem_Read(imu->i2c, MPU_ADDR, I2C_MEMADD_SIZE_8BIT, 0x75, read_buf + 6, 1, MPU_I2C_TIMEOUT));
  for (int i = 0; i < 4; i++) {
    SEGGER_RTT_printf(0, "%02x = %02x\n", REG_SAMPLE_RATE + i, read_buf[i]);
  }

  for (int i = 0; i < 2; i++) {
    SEGGER_RTT_printf(0, "%02x = %02x\n", REG_INTERRUPT_CONFIG + i, read_buf[i + 4]);
  }

  SEGGER_RTT_printf(0, "Hello = %02x\n",  read_buf[6]);

  return HAL_OK;
}
