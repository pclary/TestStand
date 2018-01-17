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
#include <iostream>
#include <fstream>

//x and z foot targets
typedef struct {
	double footPos[2];
	double footVel[2];
	double footAcc[2];
	double amp;
	double freq;
	bool bSwitchModes;
}ControlObjective;

class TestBenchInterface {
public:
	TestBenchInterface();
	virtual ~TestBenchInterface();

	bool Init();

	bool Run(ControlObjective cntrl, double* bRadio);

	void SetFootGainX(double kp, double kd) { PD_footX.Kp = kp; PD_footX.Kd = kd; }
	void SetFootGainZ(double kp, double kd) { PD_footZ.Kp = kp; PD_footZ.Kd = kd; }

private:

	void UpdateStateEstimate();
	void UpdateLogging(cassie_out_t sensors);

	DynamicModel cassie;
	DynamicState dyn_state;
	OSC_RBDL* osc;

	udp_comms* comms_tx;
	udp_comms* comms_rx;
	udp_comms* comms_vis;

	std::ofstream logFile;

	cassie_out_t sensors;
	cassie_user_in_t command;

	PD_CONTROLLER PD_footX;
	PD_CONTROLLER PD_footZ;

	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;

	double qpos[nQ];
	double qvel[nQ];

	bool m_bVisConn;

	static constexpr double cx[] = {-0.079, 0.079};
};

#endif /* TESTBENCHINTERFACE_H_ */
