#pragma once

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef motor_init();
void motor_set_duty(uint8_t motor_idx, int16_t duty_cycle);
void motor_set_rpm(uint8_t motor_idx, int16_t rpm);
