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
	double bodyZPos;
	double bodyZVel;
	double bodyZAcc;
	unsigned int idx;
}ControlObjective;

class TestBenchInterface {
public:
	TestBenchInterface();
	virtual ~TestBenchInterface();

	bool Init();

	bool Run(ControlObjective cntrl, double* bRadio);

	void SetCOMXGains(double kp, double kd) { PD_COM_X.Kp = kp; PD_COM_X.Kd = kd; }
	void SetCOMYGains(double kp, double kd) { PD_COM_Y.Kp = kp; PD_COM_Y.Kd = kd; }
	void SetCOMZGains(double kp, double kd) { PD_COM_Z.Kp = kp; PD_COM_Z.Kd = kd; }
	void SetStanceXYGains(double kp, double kd) { PD_StanceXY.Kp = kp; PD_StanceXY.Kd = kd; }
	void SetStanceZGains(double kp, double kd) { PD_StanceZ.Kp = kp; PD_StanceZ.Kd = kd; }
	void SetJointGains(double kp, double kd) { PD_Pitch.Kp = kp; PD_Pitch.Kd = kd; }

private:

	void UpdateStateEstimate();
	void StandingController(DynamicModel* dyn, DynamicState* dyn_state, ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u, telemetry_t* telem);

	void logStats();

	DynamicModel cassie;
	DynamicState dyn_state;
	OSC_RBDL* osc;

	udp_comms* comms_tx;
	udp_comms* comms_rx;
	udp_comms* comms_vis;

	cassie_out_t sensors;
	cassie_user_in_t command;

	logVars_t stats;
	std::ofstream logFile;

	PD_CONTROLLER PD_COM_X;
	PD_CONTROLLER PD_COM_Y;
	PD_CONTROLLER PD_COM_Z;
	PD_CONTROLLER PD_Pitch;
	PD_CONTROLLER PD_StanceXY;
	PD_CONTROLLER PD_StanceZ;

	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;

	double qpos[nX];
	double qvel[nQ];

	bool m_bVisConn;

	int targetIds[XDD_TARGETS];

	static constexpr double cx[] = {-0.079, 0.079};
};

#endif /* TESTBENCHINTERFACE_H_ */
