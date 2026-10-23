#include "motor.h"
#include "SEGGER_RTT.h"
#include "stm32f4xx_hal_tim.h"
#include "util.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// Hardcode to save memory
TIM_HandleTypeDef* motor_tim[4] = {
  &htim2,
  &htim2,
  &htim3,
  &htim3
};
// Swap for reverse
uint32_t motor_chan_a[4] = {
  TIM_CHANNEL_1,
  TIM_CHANNEL_4,
  TIM_CHANNEL_2,
  TIM_CHANNEL_4
};
uint32_t motor_chan_b[4] = {
  TIM_CHANNEL_2,
  TIM_CHANNEL_3,
  TIM_CHANNEL_1,
  TIM_CHANNEL_3
};

// Derived experimentally
uint16_t motor_rpm_bias[4] = {
  598,
  611,
  640,
  570
};
uint16_t motor_rpm_offset[4] = {
  251,
  246,
  235,
  266
};

HAL_StatusTypeDef motor_init() {
  ERROR_DEF;
  for (uint32_t channel = TIM_CHANNEL_1; channel <= TIM_CHANNEL_4; channel += 0x04) {
    CHECK(HAL_TIM_PWM_Start(&htim2, channel));
    CHECK(HAL_TIM_PWM_Start(&htim3, channel));
  }
  return res;
}
void motor_set_duty(uint8_t motor_idx, int16_t duty_cycle) {
  if (motor_idx >= 4) return;
  if (duty_cycle >= 1000 || duty_cycle <= -1000) return;
  if (duty_cycle < 0) {
    duty_cycle = -duty_cycle;
    __HAL_TIM_SET_COMPARE(motor_tim[motor_idx], motor_chan_b[motor_idx], duty_cycle);
    // Set to 999 for fast off
    __HAL_TIM_SET_COMPARE(motor_tim[motor_idx], motor_chan_a[motor_idx], 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor_tim[motor_idx], motor_chan_a[motor_idx], duty_cycle);
    // Set to 999 for fast off
    __HAL_TIM_SET_COMPARE(motor_tim[motor_idx], motor_chan_b[motor_idx], 0);
  }
}
void motor_set_rpm(uint8_t motor_idx, int16_t rpm) {
  if (motor_idx > 4) return;

  // fully cut power
  if (rpm == 0) motor_set_duty(motor_idx, 0);
  
  int negate = 0;
  if (rpm < 0) {
    negate = 1;
    rpm = -rpm;
  }

  // bounds on normal operation
  if (rpm < 70 || rpm > 135) return;

  int16_t result = (int16_t) (
    (motor_rpm_bias[motor_idx] * (rpm + motor_rpm_offset[motor_idx])) >> 8
  );
  if (negate) result = -result;
  SEGGER_RTT_printf(0, "%d set %d -> %d\n", motor_idx, rpm, result);
  motor_set_duty(motor_idx, result);
}
