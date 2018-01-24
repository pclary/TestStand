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
#include "OSC_RBDL.h"
#include "udp_comms.h"
#include "Command_Structs.h"
#include "CassieToRBDL.h"
#include <iostream>
#include <fstream>
#include "MPCOptions.h"

typedef struct {
	double stepTime_s; //time offset into current step
	double swingTime_s; //total swing duration (ie max stepTime_s)
	double start_pos[4];
	double end_pos[4];
} swing_foot_plan_t;


//x and z foot targets
typedef struct {
	double bodyZPos;
	double bodyZVel;
	double bodyZAcc;
	unsigned int idx;
}ControlObjective;

class Cassie {
public:
	Cassie();
	virtual ~Cassie();

	bool Init();

	bool Run(ControlObjective cntrl, double* bRadio);

	void SetCOMXGains(double kp, double kd) { PD_COM_X.Kp = kp; PD_COM_X.Kd = kd; }
	void SetCOMYGains(double kp, double kd) { PD_COM_Y.Kp = kp; PD_COM_Y.Kd = kd; }
	void SetCOMZGains(double kp, double kd) { PD_COM_Z.Kp = kp; PD_COM_Z.Kd = kd; }
	void SetStanceXYGains(double kp, double kd) { PD_StanceXY.Kp = kp; PD_StanceXY.Kd = kd; }
	void SetStanceZGains(double kp, double kd) { PD_StanceZ.Kp = kp; PD_StanceZ.Kd = kd; }
	void SetJointGains(double kp, double kd) { PD_Pitch.Kp = kp; PD_Pitch.Kd = kd; }

	bool isTrajCopied() { return m_bTrajCopied; };
	bool isReadyToCopy() { return m_bReadyToCopyTraj; }

private:

	void UpdateStateEstimate();
	void StandingController(ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u);
	void WalkingController(ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u);
	void GetSwingFootFF(double* cur_pos, double* cur_vel, swing_foot_plan_t swing_foot, double* ff_accel);
	void GetCOMTarget(double* com_targ, double* com_ff);
	void PlannerThread();
	void UpdateController(ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u);


	void logStats();

	DynamicModel dyn_model;
	DynamicState dyn_state;
	OSC_RBDL* osc;

	udp_comms* comms;
	udp_comms* comms_vis;
	udp_comms* comms_planner;

	CommandInterface::StateInfo_Struct state_info;

	MPC_OPTIONS* policy_opt;

	ROM_Policy_Struct targ_traj;
	swing_foot_plan_t leftFoot;
	swing_foot_plan_t rightFoot;

	telemetry_t telem;

	uint32_t rcv_run_count;
	double planTime_s;
	int nActiveIndex;


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

	double m_dDeltaTime_s;

	bool m_bTrajCopied;
	bool m_bReadyToCopyTraj;

	bool m_bVisConn;
	bool m_bPlanConn;
	bool m_bNewPlan;
	bool m_bProcMessage;

	int targetIds[XDD_TARGETS];

	static constexpr double cx[] = {-0.079, 0.079};
};

#endif /* TESTBENCHINTERFACE_H_ */
