/*
 * SharedRobotDefinitions.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef SHAREDROBOTDEFINITIONS_H_
#define SHAREDROBOTDEFINITIONS_H_

#include "Common_Structs.h"
#include "RobotDefinitions.h"

#define MAX_NUM_PHASES 37


typedef struct {
	double m_dTargetZPos_m;
	double m_dTargetYaw_rad;
	double m_dTargetPitch_rad;
} StandingParams;

typedef struct {

} WalkingParams;

typedef struct {
	double Kp;
	double Kd;
} PD_CONTROLLER;

static constexpr double mu = 0.9;

#endif /* SHAREDROBOTDEFINITIONS_H_ */
