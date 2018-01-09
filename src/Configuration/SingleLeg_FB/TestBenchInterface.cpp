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
	comms = new udp_comms(false, 8888, "192.168.1.147");
	comms_vis = new udp_comms(true, 8880, "192.168.1.148");
#else
	comms = new udp_comms(false, 8888, "127.0.0.1");
	comms_vis = new udp_comms(true, 8880, "127.0.0.1");
#endif
	int contactIds[] = {0, 1};
	int targetIds[] = {0, 1, 2};

	osc = new OSC_RBDL(contactIds, targetIds);

	for (int i = 0; i < DOF*XDD_TARGETS; i++)
		bActive(i,0) = true;

	for (int i = 0; i < 10; i++)
		command.torque[i] = 0.0;

	m_bVisConn = true;
}

TestBenchInterface::~TestBenchInterface() {
	// TODO Auto-generated destructor stub
}

bool TestBenchInterface::Init() {

	if (!comms->conn())
	{
		printf("Failed to connect... returning\n");
		return false;
	}

	if (!comms_vis->conn())
	{
		printf("Failed to connect to visualizer... continuing\n");
		m_bVisConn = false;
	}

	cassie.LoadModel(xml_model_filename);

	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&cassie, sensors, qpos, qvel);
	cassie.setState(qpos, qvel);

	osc->InitMatrices(&cassie);

	return true;
}

bool TestBenchInterface::Run(ControlObjective cntrl)
{
	timespec ts, tf;

	static int vis_tx_rate = 0;
	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	clock_gettime(CLOCK_REALTIME, &ts);

	CassieOutputsToState(&cassie, sensors, qpos, qvel);
	cassie.setState(qpos, qvel);

	Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	x_t(2) = cntrl.bodyZPos;
	x_t(3) = cntrl.footPos[0] + cx[1];
	x_t(5) = cntrl.footPos[1];
	x_t(6) = cntrl.footPos[0] + cx[0];
	x_t(8) = cntrl.footPos[1];
	xd_t(2) = cntrl.bodyZVel;
	xd_t(3) = cntrl.footVel[0];
	xd_t(5) = cntrl.footVel[1];
	xd_t(6) = cntrl.footVel[0];
	xd_t(8) = cntrl.footVel[1];

	Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
	Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

	cassie.GetTargetPoints(&x, &xd);

	xdd(2) = PD_bodyZ.Kp*(x_t(2) - x(2)) + PD_bodyZ.Kd*(xd_t(2) - xd(2)) + cntrl.bodyZAcc;
	xdd(3) = PD_footX.Kp*(x_t(3) - x(3)) + PD_footX.Kd*(xd_t(3) - xd(3)) + cntrl.footAcc[0];
	xdd(6) = PD_footX.Kp*(x_t(6) - x(6)) + PD_footX.Kd*(xd_t(6) - xd(6)) + cntrl.footAcc[0];
	xdd(5) = PD_footZ.Kp*(x_t(5) - x(5)) + PD_footZ.Kd*(xd_t(5) - xd(5)) + cntrl.footAcc[1];
	xdd(8) = PD_footZ.Kp*(x_t(8) - x(8)) + PD_footZ.Kd*(xd_t(8) - xd(8)) + cntrl.footAcc[1];

//	printf("%f\t%f\t%f\t%f\t%f\t%f\n", xdd(5), xdd(8), x(5), xd(5), x(8), xd(8));

	bool bContact[] = {cntrl.bContact, cntrl.bContact};
	osc->RunPTSC(&cassie, xdd, bActive, bContact, &u);

	clock_gettime(CLOCK_REALTIME, &tf);
	unsigned int diff_us = (diff(ts,tf).tv_nsec)/1e3;
	printf("%u\n", diff_us);

	TorqueToCassieInputs(u.data(), &command);

	bool bTxSuccess = comms->send_cassie_inputs(command);

	if (m_bVisConn)
	{
		if (vis_tx_rate++ > 20)
		{
			vis_tx_rate = 0;
			comms_vis->send_cassie_outputs(sensors);
		}
	}

	//add logging

	return bTxSuccess;
}

