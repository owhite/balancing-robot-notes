#pragma once
#include "supervisor.h"
#include <FlexCAN_T4.h>

void run_mode_torque_threshold(Supervisor_typedef *sup,
			       FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);
