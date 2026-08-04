#include "lds.h"

#define LDS_BYTE(idx) lds->buf[(lds->buf_idx + (idx)) % LDS_BUFFER_SIZE]

#define LDS_PWM_MIN 0
#define LDS_PWM_MAX 999
// Feedforward initial value from testing
#define LDS_PWM_INIT 635


HAL_StatusTypeDef lds_init(lds_t* lds, UART_HandleTypeDef* uart, TIM_HandleTypeDef* tim, uint32_t tim_channel) {
  lds->uart = uart;
  lds->motor_tim = tim;
  lds->motor_tim_channel = tim_channel;

  // Setup PID
  lds->motor_pid.integral = 0;
  // 300 RPM
  lds->motor_pid.setpoint = LDS_RPM;
  // experimental testing
  // https://www.desmos.com/calculator/toskeknabq
  lds->motor_pid.kp = 1.583;
  lds->motor_pid.ki = 1.5;

  HAL_StatusTypeDef res;
  
  res = HAL_TIM_PWM_Start(tim, tim_channel);
  if (res != HAL_OK) return res;
  
  res = HAL_UART_Receive_DMA(lds->uart, lds->buf, LDS_BUFFER_SIZE);
  if (res != HAL_OK) return res;

  return HAL_OK;
}
HAL_StatusTypeDef lds_update(lds_t* lds) {
  // Find where the DMA is currently writing at (counter is bytes left)
  uint16_t dma_idx= LDS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(lds->uart->hdmarx);
  // Bytes between our pointer and the DMA pointer
  uint16_t bytes_avail = (dma_idx >= lds->buf_idx) ? (dma_idx - lds->buf_idx) : (LDS_BUFFER_SIZE + dma_idx - lds->buf_idx);
  while (bytes_avail >= LDS_PAYLOAD_SIZE) {
    // look for header
    if (LDS_BYTE(0) == LDS_HEADER) {
      // parse checksum
      uint32_t check = 0;
      for (int i = 0; i < 10; i++) {
        // add in 16 bits at a time
        check = (check << 1) + LDS_BYTE(2 * i) + (LDS_BYTE(2 * i + 1) << 8);
      }
      // 15 bit result
      check = ((check & 0x7FFF) + (check >> 15)) & 0x7FFF;
      uint16_t provided_check = LDS_BYTE(20) | (LDS_BYTE(21) << 8);
      
      // Verify
      if (check != provided_check) goto parse_fail;
      
      // Packet index
      uint8_t measure_idx = LDS_BYTE(1) - 0xA0;
      if (measure_idx < 0 || measure_idx >= 90) goto parse_fail;

      lds->speed = (LDS_BYTE(2) | (LDS_BYTE(3) << 8)) / 64.0f;

      for (int i = 0; i < 4; i++) {
        // 4 bytes per reading, 4 bytes before
        int offset = 4 + i * 4;
        int angle = measure_idx * 4 + i;
        uint8_t distance_2 = LDS_BYTE(offset + 1);
        if (distance_2 & 0x80) {
          // Top byte is set = invalid data
          lds->dists[angle] = 0;
          lds->strengths[angle] = 0;
        } else {
          // remove upper 2 bits from the second byte
          lds->dists[angle] = LDS_BYTE(offset) | ((distance_2 & 0x3f) << 8);
          lds->strengths[angle] = LDS_BYTE(offset + 2) | (LDS_BYTE(offset + 3) << 8);
        };
      }

      // Advance a full packet
      lds->buf_idx = (lds->buf_idx + LDS_PAYLOAD_SIZE) % LDS_BUFFER_SIZE;
      bytes_avail -= LDS_PAYLOAD_SIZE;
    } else {
      goto parse_fail;
    }
    continue;
parse_fail:
    // This was not the start of a packet - continue
    lds->buf_idx = (lds->buf_idx + 1) % LDS_BUFFER_SIZE;
    bytes_avail--;
  }

  return HAL_OK;
}

HAL_StatusTypeDef lds_pid_update(lds_t* lds, float dt) {
  float duty = LDS_PWM_INIT + pid_update(
    &lds->motor_pid,
    lds->speed,
    dt
  );
  // clamp
  if (duty < LDS_PWM_MIN) duty = LDS_PWM_MIN;
  else if (duty > LDS_PWM_MAX) duty = LDS_PWM_MAX;

  __HAL_TIM_SET_COMPARE(lds->motor_tim, lds->motor_tim_channel, ((uint16_t) duty));

  return HAL_OK;
}
