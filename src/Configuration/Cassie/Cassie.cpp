/*
 * Cassie.cpp
 *
 *  Created on: Dec 24, 2017
 *      Author: tapgar
 */

#include "Cassie.h"

Cassie::Cassie() {
	// TODO Auto-generated constructor stub
#ifdef EMBEDDED
	comms = new udp_comms(false, 8888, "192.168.1.147");
	comms_vis = new udp_comms(true, 8880, "192.168.1.148");
#else
	comms = new udp_comms(false, 8888, "127.0.0.1");
	comms_vis = new udp_comms(true, 8880, "127.0.0.1");
#endif

	int contactIds[] = {2, 3, 4, 5};
	int targetIds[] = {1, 2, 3, 4, 5};

	m_pDynamicModel = new DynamicModel();

	m_pDynamicState = new DynamicState();
	m_pDynamicState->Init(contactIds, targetIds);

	m_pController = new OSC_RBDL();

    PD_COM.Kp = 100.0;
    PD_COM.Kd = 20.0;
    PD_Stance.Kp = 100.0;
    PD_Stance.Kd = 30.0;
    PD_Pitch.Kp = 100.0;
    PD_Pitch.Kd = 20.0;

    m_pController->AddQDDIdx(3); //roll regulation
    m_pController->AddQDDIdx(4); //pitch regulation
    m_pController->AddQDDIdx(5); //yaw regulation
    m_pController->AddQDDIdx(12); //ankle regulation
    m_pController->AddQDDIdx(21); //ankle regulation

	m_pEstimator = new StateEstimator();

	for (int i = 0; i < DOF*XDD_TARGETS; i++)
		bActive(i,0) = true;

	for (int i = 0; i < 10; i++)
		command.torque[i] = 0.0;

	m_bVisConn = true;

	controlReady = false;
	estimationReady = false;
	mainReady = false;

	controlThread = thread(&Cassie::ControlThread, this, m_pDynamicModel, m_pDynamicState, m_pController);
	estimationThread = thread(&Cassie::EstimationThread, this, m_pDynamicModel, m_pDynamicState, m_pEstimator);
}

Cassie::~Cassie() {
	// TODO Auto-generated destructor stub
}

bool Cassie::Init() {

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

	m_pDynamicModel->LoadModel(xml_model_filename);

	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	m_pEstimator->Init(m_pDynamicModel, sensors);
	m_pDynamicModel->setState(m_pEstimator->GetState());
	m_pDynamicState->UpdateDynamicState(m_pDynamicModel);
	m_pController->InitMatrices(m_pDynamicModel);

	return true;
}

void Cassie::ControlThread()
{
	while (true)
	{
		// Wait until the main process sends data
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk, []{return mainReady;});
		controlReady = false;

		//run controller (just stand for now)
		Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();
		StandingController(&u);
		TorqueToCassieInputs(u.data(), &command);

		controlReady = true;
		lk.unlock();
		cv.notify_one();
	}
}

void Cassie::EstimationThread()
{
	while (true)
	{
		// Wait until the main process sends data
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk, []{return mainReady;});
		estimationReady = false;

		//run estimator
		m_pEstimator->Update(m_pDynamicModel, m_pDynamicState, sensors);

		estimationReady = true;
		lk.unlock();
		cv.notify_one();
	}
}

bool Cassie::Run(ControlObjective cntrl)
{
	timespec ts, tf;
	clock_gettime(CLOCK_REALTIME, &ts);

	static int vis_tx_rate = 0;
	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	m_pEstimator->GetState(&m_State);

	//send data to waiting threads
	{
			std::lock_guard<std::mutex> lk(mut);
			mainReady = true;
	}
	cv.notify_all();
	usleep(20);

	//wait for control and estimation threads
	{
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk, []{return (estimationReady && controlReady);});
	}

	clock_gettime(CLOCK_REALTIME, &tf);
	unsigned int diff_us = (diff(ts,tf).tv_nsec)/1e3;

	bool bTxSuccess = comms->send_cassie_inputs(command);

	//update kinematics
	m_pDynamicModel->setState(m_pEstimator->GetState());
	//update matrices for controller and state estimator
	m_pDynamicState->UpdateDynamicState(m_pDynamicModel);

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

void Cassie::StandingController(Eigen::Matrix<double, nU, 1> u)
{
	Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		bActive(i,0) = true;

	bActive(DOF*XDD_TARGETS+3,0) = false;
	bActive(DOF*XDD_TARGETS+4,0) = false;

	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
	Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

	m_pDynamicModel->GetTargetPoints(&x, &xd);

	double targBz = 0.9;
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


	xdd(0,0) = PD_COM.Kp*(targBx - m_State.qpos[0]) - PD_COM.Kd*m_State.qvel[0];
	xdd(1,0) = PD_COM.Kp*(targBy - m_State.qpos[1]) - PD_COM.Kd*m_State.qvel[1];
	xdd(2,0) = PD_COM.Kp*(targBz - m_State.qpos[2]) - PD_COM.Kd*m_State.qvel[2];

	for (int i = 0; i < nCON; i++)
	{
		xdd(3+DOF*i,0) = PD_Stance.Kd*(0.0 - xd(DOF*(i+1)));
		xdd(4+DOF*i,0) = PD_Stance.Kd*(0.0 - xd(DOF*(i+1)+1));
		xdd(5+DOF*i,0) = PD_Stance.Kp*(-0.005 - x(DOF*(i+1)+2)) + PD_Stance.Kd*(0.0 - xd(DOF*(i+1)+2));
	}

	xdd(15,0) = PD_Pitch.Kp*(0.0 - m_State.qpos[3]) - PD_Pitch.Kd*(0.0 - m_State.qvel[3]);
	xdd(16,0) = PD_Pitch.Kp*(0.0 - m_State.qpos[4]) - PD_Pitch.Kd*(0.0 - m_State.qvel[4]);
	xdd(17,0) = PD_Pitch.Kp*(0.0 - m_State.qpos[5]) - PD_Pitch.Kd*(0.0 - m_State.qvel[5]);

	bool bInContact[] = {true, true, true, true};

	m_pController->RunPTSC(m_pDynamicState, xdd, bActive, bInContact, &u);
}
