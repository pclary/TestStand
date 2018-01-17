/*
 * RobotDefinitions.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef ROBOTDEFINITIONS_H_
#define ROBOTDEFINITIONS_H_

#include "Command_Structs.h"

#define nQstiff 5
#define nQ 5 //# joints
#define nX nQ //pos=vel+1 (quaternion)
#define nU 3 //# actuated joints
#define nCON 2 //# contact points
#define nEQ 3 //#of equality constraints 1*3

#define XDD_TARGETS 2 //# of cartesian target accelerations
#define QDD_TARGETS 0 //# of joint target accels

#define DOF 3 // 2D vs 3D

static const double m_dControlTime_s = 0.0005; //how often the controller runs

static const double m_dNominalZTarget_m = 0.95;

static const std::string xml_model_filename = "../../ThirdParty/mjpro150/model/singleleg_pinned_stiff.xml";

#pragma pack(push, 1)
typedef struct {
	double qpos[nQ];
	double torques[nU];
	double accels[XDD_TARGETS*DOF];
	double targ_pos[XDD_TARGETS*DOF];
	uint8_t op_state;
	double Kp;
	double Kd;
	double freq;
	double amp;
} telemetry_t;
#pragma pack(pop)

#endif /* ROBOTDEFINITIONS_H_ */
