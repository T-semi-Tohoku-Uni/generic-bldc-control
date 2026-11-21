#include "drive_pid.h"
#include "drive_control.h"

/* 位置型PID。今回の偏差から出力そのものを計算する。 */
float locate_pid( volatile float output, float target, float p, float i, float d, volatile float *different_sum, volatile float *low_pass_different_sum, volatile float *last_difference, int cutoff)
{

  float input;
  float difference;
  float derivative;

  difference = target - output;

  *different_sum += difference * DRIVE_CONTROL_PERIOD_S;
  if (*different_sum > 1000) {
    *different_sum = 1000;
  }
  if (*different_sum < -1000) {
    *different_sum = -1000;
  }

  derivative = difference - *last_difference;
  *low_pass_different_sum += (derivative - *low_pass_different_sum) / (float)cutoff;  //ローパス

  if (*low_pass_different_sum > 1000.0) {
    *low_pass_different_sum = 1000;
  }
  if (*low_pass_different_sum < -1000.0) {
    *low_pass_different_sum = -1000.0;
  }

  input = p * difference + i * (*different_sum) + d * (*low_pass_different_sum);

  *last_difference = difference;

  if (input >= 16384) {
    input = 16384;
  }
  if (input <= -16384) {
    input = -16384;
  }

  return input;
}

/* 速度型PID。今回の出力の増減量を、前回の出力に足す。 */
float speed_pid( volatile float output, float target, float p, float i, float d, volatile float *low_pass_different_sum, volatile float *last_difference, volatile float *last_last_difference, volatile float last_input, volatile float *low_pass_derivative, int cutoff)
{
  float input;
  float difference;
  float derivative;

  difference = target - output;
  derivative = difference - 2 * *last_difference + *last_last_difference;
  *low_pass_derivative += (derivative - *low_pass_derivative) / (float)cutoff;  //ローパス

  if (*low_pass_derivative > 1000.0) {
    *low_pass_derivative = 1000;
  }
  if (*low_pass_derivative < -1000.0) {
    *low_pass_derivative = -1000.0;
  }

  input = p * (difference - *last_difference) + i * (difference * DRIVE_CONTROL_PERIOD_S) + d * *low_pass_derivative / DRIVE_CONTROL_PERIOD_S;
  input += last_input;

  if (input >= 16384) {
    input = 16384;
  }
  if (input <= -16384) {
    input = -16384;
  }

  *last_last_difference = *last_difference;
  *last_difference = difference;

  return input;

}
