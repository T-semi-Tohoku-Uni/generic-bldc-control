#ifndef DRIVE_HARDWARE_H
#define DRIVE_HARDWARE_H

#include "main.h"

#include <stdbool.h>

HAL_StatusTypeDef stspin_read_reg(uint8_t reg, uint8_t *value);
uint8_t stspin_clear_faults(void);
bool drive_v3_charge_bootstrap(void);

bool as5047p_startup_check(void);
uint16_t as5047p_read_angle_checked(uint8_t *fault_out);
uint16_t as5047p_read_angle(void);
void as5047p_report_stopped_fault(void);

int calib_load(float *zero_offset_rad_out);
HAL_StatusTypeDef calib_save(float zero_offset_rad_val);

#endif
