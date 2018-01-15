#ifndef COMMAND_STRUCTS
#define COMMAND_STRUCTS

#include "cassie_out_t_types.h"
#include "cassie_user_in_t_types.h"

typedef enum {
	OpState_Unknown = 0x00,
	OpState_Calibrated = 0x01,
	OpState_MotorPower = 0x02,
} CassieOpState;

#endif
