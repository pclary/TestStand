/*
 * Cassie.h
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#ifndef TESTBENCHINTERFACE_H_
#define TESTBENCHINTERFACE_H_

#include <Eigen/Dense>
#include "DynamicModel.h"
#include "DynamicState.h"
#include "OSC_RBDL.h"
#include "StateEstimator.h"
#include "udp_comms.h"
#include "Command_Structs.h"
#include "CassieToRBDL.h"
#include "StateEstimatorStructs.h"

#include <thread>
#include <mutex>
#include <condition_variable>


class Cassie {
public:
	Cassie();
	virtual ~Cassie();

	bool Init();

	bool Run(ControlObjective cntrl);

	void SetFootGainX(double kp, double kd) { PD_footX.Kp = kp; PD_footX.Kd = kd; }
	void SetFootGainZ(double kp, double kd) { PD_footZ.Kp = kp; PD_footZ.Kd = kd; }
	void SetBodyGainZ(double kp, double kd) { PD_bodyZ.Kp = kp; PD_bodyZ.Kd = kd; }

private:

	void UpdateStateEstimate();

	DynamicModel* m_pDynamicModel;
	DynamicState* m_pDynamicState;
	OSC_RBDL* m_pController;
	StateEstimator* m_pEstimator;

	state_t m_State;

	PD_CONTROLLER PD_COM;
    PD_CONTROLLER PD_Stance;
    PD_CONTROLLER PD_Pitch;

	//thread synchronization
	std::mutex mut;
	std::condition_variable cv;

	std::thread controlThread;
	bool controlReady;

	std::thread estimationThread;
	bool estimationReady;

	bool mainReady;

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
