/*
 * TestBenchInterface.cpp
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

TestBenchInterface::TestBenchInterface() {
	// TODO Auto-generated constructor stub
	comms = new udp_comms(false, 8888);
	comms_vis = new udp_comms(true, 8880);

	int contactIds[] = {0, 1};
	int targetIds[] = {0, 1};

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
	static int vis_tx_rate = 0;
	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&cassie, sensors, qpos, qvel);
	cassie.setState(qpos, qvel);

	Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();
	x_t(0) = cntrl.footPos[0] + cx[1];
	x_t(2) = cntrl.footPos[1];
	x_t(3) = cntrl.footPos[0] + cx[0];
	x_t(5) = cntrl.footPos[1];
	xd_t(0) = cntrl.footVel[0];
	xd_t(2) = cntrl.footVel[1];
	xd_t(3) = cntrl.footVel[0];
	xd_t(5) = cntrl.footVel[1];

	Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
	Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

	cassie.GetTargetPoints(&x, &xd);

	xdd(0) = PD_footX.Kp*(x_t(0) - x(0)) + PD_footX.Kd*(xd_t(0) - xd(0)) + cntrl.footAcc[0];
	xdd(3) = PD_footX.Kp*(x_t(3) - x(3)) + PD_footX.Kd*(xd_t(3) - xd(3)) + cntrl.footAcc[0];
	xdd(2) = PD_footZ.Kp*(x_t(2) - x(2)) + PD_footZ.Kd*(xd_t(2) - xd(2)) + cntrl.footAcc[1];
	xdd(5) = PD_footZ.Kp*(x_t(5) - x(5)) + PD_footZ.Kd*(xd_t(5) - xd(5)) + cntrl.footAcc[1];

	bool bContact[] = {false, false};
	osc->RunPTSC(&cassie, xdd, bActive, bContact, &u);

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

