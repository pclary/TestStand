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

	int contactIds[] = {0, 1};
	for (int i = 0; i < XDD_TARGETS; i++)
		targetIds[i] = i;

	dyn_state.Init(contactIds);

	osc = new OSC_RBDL(targetIds);

	for (int i = 0; i < DOF*XDD_TARGETS; i++)
		bActive(i,0) = true;

	for (int i = 0; i < 10; i++)
		command.torque[i] = 0.0;

	m_bVisConn = true;

	logFile.open("ethercat_log.csv");
}

TestBenchInterface::~TestBenchInterface() {
	// TODO Auto-generated destructor stub
}

bool TestBenchInterface::Init() {

	if (!comms_tx->conn())
	{
		printf("Failed to connect... rt tx... returning\n");
		return false;
	}
	if (!comms_rx->conn())
	{
		printf("Failed to connect... rt rx... returning\n");
		return false;
	}

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

bool TestBenchInterface::Run(ControlObjective cntrl, double* bRadio)
{
	static int vis_tx_rate = 0;
	int num_retries = 0;
	while (!comms_rx->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&cassie, sensors, qpos, qvel);
	cassie.setState(qpos, qvel);

	for (int i = 0; i < 16; i++)
		bRadio[i] = sensors.pelvis.radio.channel[i];

	dyn_state.UpdateDynamicState(&cassie);

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

	cassie.GetTargetPoints(&x, &xd, targetIds);

	xdd(0) = PD_footX.Kp*(x_t(0) - x(0)) + PD_footX.Kd*(xd_t(0) - xd(0)) + cntrl.footAcc[0];
	xdd(3) = PD_footX.Kp*(x_t(3) - x(3)) + PD_footX.Kd*(xd_t(3) - xd(3)) + cntrl.footAcc[0];
	xdd(2) = PD_footZ.Kp*(x_t(2) - x(2)) + PD_footZ.Kd*(xd_t(2) - xd(2)) + cntrl.footAcc[1];
	xdd(5) = PD_footZ.Kp*(x_t(5) - x(5)) + PD_footZ.Kd*(xd_t(5) - xd(5)) + cntrl.footAcc[1];

	bool bContact[] = {false, false};
	osc->RunPTSC(&cassie, &dyn_state, xdd, bActive, bContact, &u);

	for (int i = 0; i < 10; i++)
		command.torque[i] = 0.0;

	TorqueToCassieInputs(u.data(), &command);



	bool bTxSuccess = true;

#ifdef EMBEDDED

	UpdateLogging(sensors);

	if (sensors.pelvis.radio.channel[11] < 0.5)
		for (int i = 0; i < 10; i++)
			command.torque[i] = 0.0;
	command.torque[2] *= sensors.leftLeg.hipPitchDrive.gearRatio;
	command.torque[3] *= sensors.leftLeg.kneeDrive.gearRatio;
	command.torque[4] *= sensors.leftLeg.footDrive.gearRatio;
#endif
	comms_tx->send_cassie_inputs(command);

	if (m_bVisConn)
	{
		if (vis_tx_rate++ > 120)
		{

			telemetry_t telem;
			telem.op_state = 0x00;
			if (sensors.isCalibrated)
				telem.op_state |= OpState_Calibrated;
			else
				telem.op_state &= ~OpState_Calibrated;
			if (sensors.pelvis.radio.channel[11] > 0.0)
				telem.op_state |= OpState_MotorPower;
			else
				telem.op_state &= ~OpState_MotorPower;
			for (int i = 0; i < nQ; i++)
				telem.qpos[i] = qpos[i];
			for (int i = 0; i < nU; i++)
				telem.torques[i] = u(i,0);
			for (int i = 0; i < XDD_TARGETS*DOF; i++)
			{
				telem.accels[i] = xdd(i);
				telem.targ_pos[i] = x_t(i);
//				printf("%f\t", x_t(i));
			}
			telem.Kp = PD_footX.Kp;
			telem.Kd = PD_footX.Kd;
			telem.freq = cntrl.freq;
			telem.amp = cntrl.amp;
//			printf("\n");

			vis_tx_rate = 0;
			comms_vis->send_telemetry(telem);
		}
	}

	//add logging

	return bTxSuccess;
}

void TestBenchInterface::UpdateLogging(cassie_out_t sensors)
{

	for (int i = 0; i < 5; i++)
		logFile << sensors.pelvis.targetPc.etherCatStatus[i] << ",";
	logFile << sensors.pelvis.targetPc.etherCatStatus[5] << "\n";
}
