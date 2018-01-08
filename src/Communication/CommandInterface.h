/*
 * CommandInterface.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef COMMANDINTERFACE_H_
#define COMMANDINTERFACE_H_

#include "SharedRobotDefinitions.h"

#define MAX_TRAJ_PTS 500
#define MAX_CON_SWITCH 40

namespace CommandInterface {

typedef enum {
	StateInfo = 0,
	NewTrajectory = 1,
	StandingObjective = 2,
	UserParams = 3
} MESSAGE_TYPE;

typedef enum {
	Idle = 0,
	Standing = 1,
	Walking_DS = 2,
	Walking_SS_Left = 3,
	Walking_SS_Right = 4
} OP_STATE;

#pragma pack(push, 1)
typedef struct {
	uint8_t op_mode;
	uint32_t run_count;
	int32_t x_mm[nX];
	int32_t com[4];
	int32_t com_vel[4];
	int32_t left[4];
	int32_t right[4];
}StateInfo_Struct;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint8_t append;
	uint32_t run_count;
	uint8_t numContactSwitch;
	uint16_t numPoints;
	uint32_t step_height;
	int32_t dt_c;
}ROM_TargTraj_Struct;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	uint8_t con_state;
	int32_t T_start; //on !append T_start = 0
	int32_t T_end;
	int32_t left[4];
	int32_t right[4];
}ContactInfo_Struct;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	int32_t com[4];
	int32_t com_xdd[4];
}ROM_TrajPt_Struct;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
	int32_t dxT[4];
	uint32_t step_time_us;
	uint32_t ds_perc;
	uint32_t step_height;
} USER_Params;
#pragma pack(pop)

};

#pragma pack(push, 1)
typedef struct {
	CommandInterface::StateInfo_Struct state;
	CommandInterface::USER_Params usr_cmd;
	CommandInterface::ROM_TargTraj_Struct trajInfo;
	CommandInterface::ROM_TrajPt_Struct posTraj[MAX_TRAJ_PTS];
	CommandInterface::ContactInfo_Struct conSched[MAX_CON_SWITCH];
}CommAction_Struct;
#pragma pack(pop)

typedef struct {
	uint8_t con_state;
	double T_start;
	double T_end;
	double left[4];
	double right[4];
}ContactInfo_Struct;

typedef struct {
	double com[4];
	double com_xdd[4];
}ROM_TrajPt_Struct;

typedef struct {
	double dt_c; //dt per point
	double step_height;
	uint8_t numContactSwitch;
	uint16_t numPoints;
	uint32_t run_count;
	std::vector<ROM_TrajPt_Struct> com_traj;
	std::vector<ContactInfo_Struct> con_sched;
}ROM_Policy_Struct;


//#pragma pack(push, 1)
//typedef struct {
//
//} TrajPoint_Struct;
//#pragma pack(pop)
//
//#pragma pack(push, 1)
//typedef struct {
//	int NumPoints;
//	int NumPhases;
//
//} NewTraj_Struct;
//#pragma pack(pop)





#endif /* COMMANDINTERFACE_H_ */
