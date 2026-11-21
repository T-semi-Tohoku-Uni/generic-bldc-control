#ifndef DRIVE_PID_H
#define DRIVE_PID_H

float locate_pid(
    volatile float output, float target, float p, float i, float d,
    volatile float *different_sum, volatile float *low_pass_different_sum,
    volatile float *last_difference, int cutoff);

float speed_pid(
    volatile float output, float target, float p, float i, float d,
    volatile float *low_pass_different_sum, volatile float *last_difference,
    volatile float *last_last_difference, volatile float last_input,
    volatile float *low_pass_derivative, int cutoff);

#endif
