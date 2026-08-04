#include "pid.h"

// 1 ms
#define MIN_DT 0.001f
#define MAX_INTEGRAL 1000.0f
#define MIN_INTEGRAL -1000.0f

float pid_update(pid_controller_t* pid, float current, float dt) {
  float error = pid->setpoint - current;
  if (dt < MIN_DT) dt = MIN_DT;

  // Clamp integral
  pid->integral += error * dt;
  if (pid->integral > MAX_INTEGRAL) pid->integral = MAX_INTEGRAL;
  if (pid->integral < MIN_INTEGRAL) pid->integral = MIN_INTEGRAL;

  float output = pid->kp * error + pid->ki * pid->integral;

  return output;
}
