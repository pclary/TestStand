/*
 * TestBenchInterface.cpp
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

TestBenchInterface::TestBenchInterface() {
	// TODO Auto-generated constructor stub
#ifdef EMBEDDED
	comms_tx = new udp_comms(true, 25000, "10.10.10.3");
	comms_rx = new udp_comms(false, 25000, "10.10.10.100");
	comms_vis = new udp_comms(true, 8880, "192.168.1.101");
#else
	comms_tx = new udp_comms(false, 25001, "127.0.0.1");
	comms_rx = new udp_comms(false, 25000, "127.0.0.1");
	comms_vis = new udp_comms(true, 8880, "127.0.0.1");
#endif

	int contactIds[] = {2, 3, 4, 5};
	for (int i = 0; i < XDD_TARGETS; i++)
		targetIds[i] = i+1;

	dyn_state.Init(contactIds);

	osc = new OSC_RBDL(targetIds);
	osc->AddQDDIdx(3);
	osc->AddQDDIdx(4);
	osc->AddQDDIdx(5);

	for (int i = 0; i < DOF*XDD_TARGETS; i++)
		bActive(i,0) = true;

	for (int i = 0; i < 10; i++)
		command.torque[i] = 0.0;

	for (int i = 0; i < nQ; i++)
		qpos[i] = qvel[i] = 0.0;

	m_bVisConn = true;
}

TestBenchInterface::~TestBenchInterface() {
	// TODO Auto-generated destructor stub
}

bool TestBenchInterface::Init() {

	if (!comms_tx->conn())
	{
		printf("Failed to connect... returning\n");
		return false;
	}
	if (!comms_rx->conn())
	{
		printf("Failed to connect... returning\n");
		return false;
	}
#ifndef EMBEDDED
	comms_tx = comms_rx;
#endif
	if (!comms_vis->conn())
	{
		printf("Failed to connect to visualizer... continuing\n");
		m_bVisConn = false;
	}

	cassie.LoadModel(xml_model_filename);

	int num_retries = 0;
	while (!comms_rx->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&cassie, sensors, qpos, qvel);
	cassie.setState(qpos, qvel);

	osc->InitMatrices(&cassie);

	return true;
}

bool TestBenchInterface::Run(ControlObjective cntrl)
{
	static int vis_tx_rate = 0;
	int num_retries = 0;
	while (!comms_rx->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&cassie, sensors, qpos, qvel);
	cassie.setState(qpos, qvel);

//	printf("Q:\n");
//	for (int i = 0; i < nX; i++)
//		printf("%f\n", qpos[i]);

	dyn_state.UpdateDynamicState(&cassie);

	Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();

	telemetry_t telem;

	StandingController(&cassie, &dyn_state, cntrl, &u, &telem);

	TorqueToCassieInputs(u.data(), &command);

	bool bTxSuccess = true;

//#ifndef EMBEDDED
	bTxSuccess = comms_tx->send_cassie_inputs(command);
//#endif

	if (m_bVisConn)
	{
		if (vis_tx_rate++ > 120)
		{

			for (int i = 0; i < 3; i++)
				telem.qpos[i] = qpos[i];
			telem.qpos[3] = qpos[nQ];
			for (int i = 4; i < nX; i++)
				telem.qpos[i] = qpos[i-1];

			vis_tx_rate = 0;
			comms_vis->send_telemetry(telem);
		}
	}

//	usleep(1e6);
	//add logging

	return bTxSuccess;
}

void TestBenchInterface::StandingController(DynamicModel* dyn, DynamicState* dyn_state, ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u, telemetry_t* telem)
{
	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		bActive(i,0) = true;

	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
	Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

	dyn->GetTargetPoints(&x, &xd, targetIds);

//	for (int i = 0; i < nQ; i++)
//		printf("%f,",qpos[i]);
//	for (int i = 0; i < nQ; i++)
//		printf("%f,",qvel[i]);
//	for (int i = 0; i < DOF*XDD_TARGETS; i++)
//		printf("%f,",x(i));
//	for (int i = 0; i < DOF*XDD_TARGETS-1; i++)
//		printf("%f,",xd(i));
//	printf("%f\n",xd(DOF*XDD_TARGETS-1));

	double targBy = 0.0;
	double targBx = 0.0;

	double count = 0.0;
	for (int i = 1; i < 5; i++)
	{
		targBx += x(i*3);
		targBy += x(i*3+1);
		count += 1.0;
	}
	targBx /= count;
	targBy /= count;

	xdd(0,0) = PD_COM.Kp*(targBx - qpos[0]) + PD_COM.Kd*(0.0 - qvel[0]);
	xdd(1,0) = PD_COM.Kp*(targBy - qpos[1]) + PD_COM.Kd*(0.0 - qvel[1]);
	xdd(2,0) = PD_COM.Kp*(cntrl.bodyZPos - qpos[2]) + PD_COM.Kd*(cntrl.bodyZVel - qvel[2]) + cntrl.bodyZAcc;

//	printf("COM\n");
//	printf("x: %f\t%f\t%f\t%f\n", targBx, qpos[0], qvel[0], xdd(0,0));
//	printf("y: %f\t%f\t%f\t%f\n", targBy, qpos[1], qvel[1], xdd(1,0));
//	printf("z: %f\t%f\t%f\t%f\n", cntrl.bodyZPos, qpos[2], qvel[2], xdd(2,0));

	for (int i = 0; i < nCON; i++)
	{
		xdd(3+DOF*i,0) = PD_Stance.Kd*(0.0 - xd(DOF*(i+1)));
		xdd(4+DOF*i,0) = PD_Stance.Kd*(0.0 - xd(DOF*(i+1)+1));
		xdd(5+DOF*i,0) = PD_Stance.Kp*(-0.005 - x(DOF*(i+1)+2)) + PD_Stance.Kd*(0.0 - xd(DOF*(i+1)+2));
//		printf("Contact: %d\n",i);
//		printf("x: %f\t%f\n", xd(DOF*(i+1)), xdd(3+DOF*i,0));
//		printf("y: %f\t%f\n", xd(DOF*(i+1)+1), xdd(4+DOF*i,0));
//		printf("z: %f\t%f\t%f\n", x(DOF*(i+1)+2), xd(DOF*(i+1)+2), xdd(5+DOF*i,0));
	}


	double quat[4];
	double euler[3];
	cassie.GetMainBodyQuaternion(quat);
	quaternionToEuler(quat, euler);
	xdd(15,0) = PD_Pitch.Kp*(0.0 - euler[0]) + PD_Pitch.Kd*(0.0 - qvel[3]);
	xdd(16,0) = PD_Pitch.Kp*(0.0 - euler[1]) + PD_Pitch.Kd*(0.0 - qvel[4]);
	xdd(17,0) = 0.0*PD_Pitch.Kp*(0.0 - euler[2]) + PD_Pitch.Kd*(0.0 - qvel[5]);

//	printf("Attitude\n");
//	printf("roll: %f\t%f\t%f\n", qpos[3], qvel[3], xdd(15,0));
//	printf("pitch: %f\t%f\t%f\n", qpos[4], qvel[4], xdd(16,0));
//	printf("yaw: %f\t%f\t%f\n", qpos[5], qvel[5], xdd(17,0));

	bool bInContact[] = {true, true, true, true};

	osc->RunPTSC(dyn, dyn_state, xdd, bActive, bInContact, u);

//	printf("Torques:\n");
//	printf("Left:\t");
//	for (int i = 0; i < nU/2; i++)
//		printf("%f\t", (*u)(i,0));
//	printf("\nRight:\t");
//	for (int i = nU/2; i < nU; i++)
//		printf("%f\t", (*u)(i,0));
//	printf("\n");

	for (int i = 0; i < nU; i++)
		telem->torques[i] = (*u)(i,0);
	for (int i = 0; i < XDD_TARGETS*DOF; i++)
	{
		telem->accels[i] = xdd(i);
		telem->targ_pos[i] = x_t(i);
	}
}


