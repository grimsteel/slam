#pragma once

#include "stm32f4xx_hal.h"
#include "pid.h"

#define LDS_PAYLOAD_SIZE 22
#define LDS_BUFFER_SIZE 128
#define LDS_HEADER 0xFA
#define LDS_RPM 300.0f

typedef struct {
  UART_HandleTypeDef* uart;
  TIM_HandleTypeDef* motor_tim;
  uint32_t motor_tim_channel;
  uint8_t buf[LDS_BUFFER_SIZE];
  uint16_t buf_idx;
  uint16_t dists[360];
  uint16_t strengths[360];
  float speed;
  pid_controller_t motor_pid;
} lds_t;

HAL_StatusTypeDef lds_init(lds_t* lds, UART_HandleTypeDef* uart, TIM_HandleTypeDef* tim, uint32_t tim_channel);
HAL_StatusTypeDef lds_update(lds_t* lds);
HAL_StatusTypeDef lds_pid_update(lds_t* lds, float dt);
