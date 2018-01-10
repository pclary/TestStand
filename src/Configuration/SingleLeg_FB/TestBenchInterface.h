/*
 * TestBenchInterface.h
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#ifndef TESTBENCHINTERFACE_H_
#define TESTBENCHINTERFACE_H_

#include <Eigen/Dense>
#include "DynamicModel.h"
#include "OSC_RBDL.h"
#include "udp_comms.h"
#include "Command_Structs.h"
#include "CassieToRBDL.h"

//x and z foot targets
typedef struct {
	double bodyZPos;
	double bodyZVel;
	double bodyZAcc;
	double footPos[2];
	double footVel[2];
	double footAcc[2];
	bool bContact;
}ControlObjective;

class TestBenchInterface {
public:
	TestBenchInterface();
	virtual ~TestBenchInterface();

	bool Init();

	bool Run(ControlObjective cntrl);

	void SetFootGainX(double kp, double kd) { PD_footX.Kp = kp; PD_footX.Kd = kd; }
	void SetFootGainZ(double kp, double kd) { PD_footZ.Kp = kp; PD_footZ.Kd = kd; }
	void SetBodyGainZ(double kp, double kd) { PD_bodyZ.Kp = kp; PD_bodyZ.Kd = kd; }

private:

	void UpdateStateEstimate();

	DynamicModel cassie;
	OSC_RBDL* osc;

	udp_comms* comms;
	udp_comms* comms_vis;

	cassie_out_t sensors;
	cassie_user_in_t command;

	PD_CONTROLLER PD_footX;
	PD_CONTROLLER PD_footZ;
	PD_CONTROLLER PD_bodyZ;

	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;

	double qpos[nQ];
	double qvel[nQ];

	bool m_bVisConn;

	static constexpr double cx[] = {-0.079, 0.079};
};

#endif /* TESTBENCHINTERFACE_H_ */
