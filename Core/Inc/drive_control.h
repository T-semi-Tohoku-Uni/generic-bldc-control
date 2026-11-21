#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>

#define DRIVE_CONTROL_PERIOD_S 0.0002

void drive_init(void);
void drive_report(void);

/* Called from the FDCAN RX interrupt after validating a command. */
void drive_accept_can_command(float speed_target, uint8_t requested_pid_mode, uint8_t requested_control_mode);

#endif
