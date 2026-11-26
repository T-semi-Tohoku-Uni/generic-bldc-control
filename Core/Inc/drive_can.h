#ifndef DRIVE_CAN_H
#define DRIVE_CAN_H

#include "main.h"

void FDCAN1_ConfigFilterAndStart(void);
void drive_can_button_task(void);

#endif
