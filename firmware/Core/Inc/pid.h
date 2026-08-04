#pragma once

#include "stm32f4xx_hal.h"

typedef struct {
  float kp;
  float ki;
  float integral;
  float setpoint;
} pid_controller_t;

float pid_update(pid_controller_t* pid, float current, float dt);
