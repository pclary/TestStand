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
	comms = new udp_comms("10.10.10.100", "10.10.10.3", 25000);
	comms_vis = new udp_comms("192.168.1.148", "192.168.1.101", 8880);
	comms_planner = new udp_comms("192.168.1.148", "192.168.1.200", 8888);
#else
	comms = new udp_comms("127.0.0.2", "127.0.0.1", 8884);
	comms_vis = new udp_comms("127.0.0.2", "127.0.0.3", 8880);
	comms_planner = new udp_comms("127.0.0.2", "127.0.0.4", 8888);
#endif

	int contactIds[] = {2, 3, 4, 5};
	for (int i = 0; i < XDD_TARGETS; i++)
		targetIds[i] = i+1;

	dyn_state.Init(contactIds);

	osc = new OSC_RBDL(targetIds);
	osc->AddQDDIdx(3);
	osc->AddQDDIdx(4);
	osc->AddQDDIdx(5);

	m_dDeltaTime_s = 0.0;

	for (int i = 0; i < DOF*XDD_TARGETS; i++)
		bActive(i,0) = true;

	for (int i = 0; i < 10; i++)
		command.torque[i] = 0.0;

	for (int i = 0; i < nQ; i++)
		qpos[i] = qvel[i] = 0.0;

	m_bVisConn = true;
	m_bPlanConn = true;
	m_bNewPlan = false;

	logFile.open("logfile.csv");

	state_info.run_count = 0;
	state_info.eOpState = CommandInterface::Idle;

	policy_opt = new MPC_OPTIONS();
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
	if (!comms_planner->conn())
	{
		printf("Failed to connect to planner... continuing\n");
		m_bPlanConn = false;
	}

	dyn_model.LoadModel(xml_model_filename);

	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&dyn_model, sensors, qpos, qvel);
	dyn_model.setState(qpos, qvel);

	osc->InitMatrices(&dyn_model);

	if (m_bPlanConn)
		planCommThread = thread(&Cassie::PlannerThread, this);

	return true;
}

void Cassie::UpdateController(ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u) {

	planTime_s += m_dDeltaTime_s;
	leftFoot.stepTime_s += m_dDeltaTime_s;
	rightFoot.stepTime_s += m_dDeltaTime_s;
	state_info.run_count++;

	if (m_bNewPlan)
	{
		m_bReadyToCopyTraj = true;
		// Wait until the main process is ready for the new trajectory
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk, []{return m_bTrajCopied;});

		m_bReadyToCopyTraj = false;
		m_bNewPlan = false;

		planTime_s = double(state_info.run_count - targ_traj.run_count)*0.0005;
		nActiveIndex = 0;
	}

	static CommandInterface::OP_STATE ePrevOpState = state_info.eOpState;

	if ((state_info.eOpState != CommandInterface::Idle && state_info.eOpState != CommandInterface::Standing) || bNewTraj)
	{
		for (int i = 0; i < targ_traj.numContactSwitch; i++)
		{
			if (planTime_s >= targ_traj.con_sched[i].T_start && planTime_s < targ_traj.con_sched[i].T_end)
			{
				if (targ_traj.con_sched[i].con_state == Double)
				{
					state_info.eOpState = CommandInterface::Walking_DS;
					for (int j = 0; j < 4; j++)
					{
						leftFoot.start_pos[j] = left[j];
						rightFoot.start_pos[j] = right[j];
					}
				}
				else if (targ_traj.con_sched[i].con_state == SS_Left)
				{
					state_info.eOpState = CommandInterface::Walking_SS_Left;
					if (state_info.eOpState != ePrevOpState)
					{
						rightFoot.stepTime_s = planTime_s - targ_traj.con_sched[i].T_start;
						rightFoot.swingTime_s = targ_traj.con_sched[i].T_end - targ_traj.con_sched[i].T_start;
						for (int j = 0; j < 4; j++)
							rightFoot.start_pos[j] = targ_traj.con_sched[i].right[j];
					}
					for (int j = 0; j < 4; j++)
						leftFoot.start_pos[j] = rom_state.left[j];
				}
				else if (targ_traj.con_sched[i].con_state == SS_Right)
				{
					state_info.eOpState = CommandInterface::Walking_SS_Right;
					if (state_info.eOpState != ePrevOpState)
					{
						leftFoot.stepTime_s = planTime_s - targ_traj.con_sched[i].T_start;
						leftFoot.swingTime_s = targ_traj.con_sched[i].T_end - targ_traj.con_sched[i].T_start;
						for (int j = 0; j < 4; j++)
							leftFoot.start_pos[j] = targ_traj.con_sched[i].left[j];
					}
					for (int j = 0; j < 4; j++)
						rightFoot.start_pos[j] = rom_state.right[j];
				}

				nActiveIndex = i;
				break;
			}
		}
	}

	switch(state_info.eOpState)
	{

	case CommandInterface::Idle:
	case CommandInterface::Standing:
		StandingController(cntrl, u);
		break;

	case CommandInterface::Walking_DS:
	case CommandInterface::Walking_SS_Left:
	case CommandInterface::Walking_SS_Right:
		WalkingController(cntrl, u);
		break;

	}

}

bool Cassie::Run(ControlObjective cntrl, double* bRadio)
{
	timespec ts, tf;
	clock_gettime(CLOCK_REALTIME, &ts);

	static int vis_tx_rate = 0;
	int num_retries = 0;
	while (!comms->receive_cassie_outputs(&sensors))
		if (num_retries++ > 5)
			return false;

	CassieOutputsToState(&dyn_model, sensors, qpos, qvel);
	dyn_model.setState(qpos, qvel);

	for (int i = 0; i < nX; i++)
		stats.qpos[i] = qpos[i];
	for (int i = 0; i < nQ; i++)
		stats.qvel[i] = qvel[i];

	for (int i = 0; i < 16; i++)
		bRadio[i] = sensors.pelvis.radio.channel[i];

	dyn_state.UpdateDynamicState(&dyn_model);

	Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();

	UpdateController(cntrl, &u); //increments forward in contact schedule

	TorqueToCassieInputs(u.data(), &command);

	bool bTxSuccess = true;

#ifdef EMBEDDED

	if (sensors.pelvis.radio.channel[11] < 0.5)
		for (int i = 0; i < 10; i++)
			command.torque[i] = 0.0;

	command.torque[0] *= sensors.leftLeg.hipRollDrive.gearRatio;
	command.torque[1] *= sensors.leftLeg.hipYawDrive.gearRatio;
	command.torque[2] *= sensors.leftLeg.hipPitchDrive.gearRatio;
	command.torque[3] *= sensors.leftLeg.kneeDrive.gearRatio;
	command.torque[4] *= sensors.leftLeg.footDrive.gearRatio;
	command.torque[5] *= sensors.rightLeg.hipRollDrive.gearRatio;
	command.torque[6] *= sensors.rightLeg.hipYawDrive.gearRatio;
	command.torque[7] *= sensors.rightLeg.hipPitchDrive.gearRatio;
	command.torque[8] *= sensors.rightLeg.kneeDrive.gearRatio;
	command.torque[9] *= sensors.rightLeg.footDrive.gearRatio;
#endif

	bTxSuccess = comms->send_cassie_inputs(command);

	comms_planner->send_state_info(state_info);

	//	clock_gettime(CLOCK_REALTIME, &tf);
	//	stats.qp_dt_nsec = diff(ts,tf).tv_nsec;
	//	ts = tf;
	//
	//	//	logStats(); //need to spin this off on another thread... currently causing time spikes
	//	clock_gettime(CLOCK_REALTIME, &tf);
	//	stats.log_dt_nsec = diff(ts,tf).tv_nsec;
	//	ts = tf;

	if (m_bVisConn)
	{
		if (vis_tx_rate++ > 60)
		{
			telem.op_state = 0x00;
			if (sensors.isCalibrated)
				telem.op_state |= OpState_Calibrated;
			else
				telem.op_state &= ~OpState_Calibrated;
			if (sensors.pelvis.radio.channel[11] > 0.0)
				telem.op_state |= OpState_MotorPower;
			else
				telem.op_state &= ~OpState_MotorPower;
			for (int i = 0; i < 3; i++)
				telem.qpos[i] = qpos[i];
			telem.qpos[3] = qpos[nQ];
			for (int i = 4; i < nX; i++)
				telem.qpos[i] = qpos[i-1];

			telem.select_index = cntrl.idx;
			telem.Kp[0] = PD_COM_X.Kp;
			telem.Kd[0] = PD_COM_X.Kd;
			telem.Kp[1] = PD_COM_Y.Kp;
			telem.Kd[1] = PD_COM_Y.Kd;
			telem.Kp[2] = PD_COM_Z.Kp;
			telem.Kd[2] = PD_COM_Z.Kd;
			telem.Kp[3] = PD_StanceXY.Kp;
			telem.Kd[3] = PD_StanceXY.Kd;
			telem.Kp[4] = PD_StanceZ.Kp;
			telem.Kd[4] = PD_StanceZ.Kd;
			telem.Kp[5] = PD_Pitch.Kp;
			telem.Kd[5] = PD_Pitch.Kd;

			vis_tx_rate = 0;
			comms_vis->send_telemetry(telem);
		}
	}

	clock_gettime(CLOCK_REALTIME, &tf);

	m_dDeltaTime_s = double(diff(ts,tf).tv_nsec)/1e9;

	return bTxSuccess;
}

void Cassie::GetCOMTarget(double* com_targ, double* com_ff)
{
	int N = int(planTime_s / targ_traj->dt_c);
	if (N > MAX_TRAJ_PTS-1)
	{
		//		printf("youre fucked!\n");
		N = MAX_TRAJ_PTS-2;
	}

	for (int i = 0; i < 4; i++)
		com_targ[i] = targ_traj->com_traj[N].com[i];
	for (int i = 0; i < 4; i++)
		com_targ[i+4] = (targ_traj->com_traj[N+1].com[i] - targ_traj->com_traj[N-1].com[i])/(2*targ_traj->dt_c);
	for (int i = 0; i < 4; i++)
		com_ff[i] = targ_traj->com_traj[N].com_xdd[i];

}


void Cassie::GetSwingFootFF(double* cur_pos, double* cur_vel, swing_foot_plan_t swing_foot, double* ff_accel)
{
	double t, T;
	Matrix2d A = Matrix2d::Zero();
	Vector2d b = Vector2d::Zero();
	Vector2d ab = Vector2d::Zero();

	double targ[] = {0.0, 0.0, 0.0};
	double vel_targ[] = {0.0, 0.0, 0.0};
	if (swing_foot.stepTime_s < swing_foot.swingTime_s/2.0) //still in up phase
	{
		for (int i = 0; i < 2; i++)
			targ[i] = (swing_foot.end_pos[i] + swing_foot.start_pos[i])/2.0;
		targ[2] = state_info.step_height;
		T = (swing_foot.swingTime_s/2.0) - swing_foot.stepTime_s;
		for (int i = 0; i < 2; i++)
		{
			double dT = swing_foot.swingTime_s;
			double dX = swing_foot.end_pos[i] - swing_foot.start_pos[i];
			double const_vel = dX/dT;
			vel_targ[i] = 2.0*const_vel; //const accel/decel profile
		}
	}
	else
	{
		T = swing_foot.swingTime_s - swing_foot.stepTime_s;
		for (int i = 0; i < 2; i++)
			targ[i] = swing_foot.end_pos[i];
	}


	//you may need to make a seperate region near the end of the swing phase where T gets small
	t = 0.0;

	A << pow(T,2.0)/3.0, pow(T,2.0)/6.0, T/2, T/2;

	for (int i = 0; i < 3; i++)
	{
		double y0, yd0, yT, ydT;
		y0 = cur_pos[i];
		yd0 = cur_vel[i];

		yT = targ[i];
		ydT = vel_targ[i];
		b << yT - y0 - yd0*T, ydT - yd0;
		ab = pseudoinverse(A)*b;
		ff_accel[i] = ab(0)*(1-t/T) + ab(1)*t/T;
	}
}

void Cassie::WalkingController(ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u)
{
	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		bActive(i,0) = true;

	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
	Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

	dyn_model.GetTargetPoints(&x, &xd, targetIds);

	bool bDesiredContact[] = {true, true, true, true};

	double com_targ[8], com_ff[4];
	GetCOMTarget(com_targ, com_ff);

	for (int i = 0; i < 3; i++)
	{
		xdd(i,0) = PD_COM.Kp*(com_targ[i] - x(i)) + PD_COM.Kd*(com_targ[i+4] - xd(i));
		xdd(i,0) += com_ff[i];
	}

	const double* targ_left = targ_traj->con_sched[nActiveIndex].left;
	const double* targ_right = targ_traj->con_sched[nActiveIndex].right;

	if (targ_traj->con_sched[nActiveIndex].con_state == SS_Left)
	{
		bDesiredContact[2] = bDesiredContact[3] = false;
		//left stance
		for (int i = 0; i < 2; i++)
		{
			xdd(3+DOF*i,0) = -PD_Stance.Kd*xd(3+DOF*i);
			xdd(4+DOF*i,0) = -PD_Stance.Kd*xd(4+DOF*i);
			xdd(5+DOF*i,0) = PD_Stance.Kp*(-5e-3 - x(5+DOF*i)) + PD_Stance.Kd*(0.0 - xd(5+DOF*i));
		}
		double cur_pos[3];
		double cur_vel[3];
		double ff_accel[3];
		for (int i = 0; i < 3; i++)
		{
			cur_pos[i] = (x(3 + i) + x(6 + i))/2.0;
			cur_vel[i] = (xd(3 + i) + xd(6 + i))/2.0;
		}
		GetSwingFootFF(cur_pos, cur_vel, rightFoot, ff_accel);
		for (int i = 2; i < 4; i++)
		{
			xdd(3+DOF*i,0) = ff_accel[0];
			xdd(4+DOF*i,0) = ff_accel[1];
			xdd(5+DOF*i,0) = ff_accel[2];
		}
	}
	else if (targ_traj->con_sched[nActiveIndex].con_state == SS_Right)
	{
		bDesiredContact[0] = bDesiredContact[1] = false;

		double cur_pos[3];
		double cur_vel[3];
		double ff_accel[3];
		for (int i = 0; i < 3; i++)
		{
			cur_pos[i] = (x(9 + i) + x(12 + i))/2.0;
			cur_vel[i] = (xd(9 + i) + xd(12 + i))/2.0;
		}
		GetSwingFootFF(cur_pos, cur_vel, rightFoot, ff_accel);
		for (int i = 0; i < 2; i++)
		{
			xdd(3+DOF*i,0) = ff_accel[0];
			xdd(4+DOF*i,0) = ff_accel[1];
			xdd(5+DOF*i,0) = ff_accel[2];
		}

		//right stance
		for (int i = 2; i < 4; i++)
		{
			xdd(3+DOF*i,0) = -PD_Stance.Kd*xd(3+DOF*i);
			xdd(4+DOF*i,0) = -PD_Stance.Kd*xd(4+DOF*i);
			xdd(5+DOF*i,0) = PD_Stance.Kp*(-5e-3 - x(5+DOF*i)) + PD_Stance.Kd*(0.0 - xd(5+DOF*i));
		}
	}
	else
	{
		//right stance
		for (int i = 0; i < 4; i++)
		{
			xdd(3+DOF*i,0) = -PD_Stance.Kd*xd(3+DOF*i);
			xdd(4+DOF*i,0) = -PD_Stance.Kd*xd(4+DOF*i);
			xdd(5+DOF*i,0) = PD_Stance.Kp*(-5e-3 - x(5+DOF*i)) + PD_Stance.Kd*(0.0 - xd(5+DOF*i));
		}
	}

	double quat[4];
	double euler[3];
	dyn_model.GetMainBodyQuaternion(quat);
	quaternionToEuler(quat, euler);
	xdd(15,0) = PD_Pitch.Kp*(0.0 - euler[0]) + PD_Pitch.Kd*(0.0 - qvel[3]);
	xdd(16,0) = PD_Pitch.Kp*(0.0 - euler[1]) + PD_Pitch.Kd*(0.0 - qvel[4]);
	xdd(17,0) = PD_Pitch.Kp*(com_targ[3] - euler[2]) + PD_Pitch.Kd*(com_targ[7] - qvel[5]) + com_ff[3];


	for (int i = 0; i < XDD_TARGETS*DOF; i++)
	{
		stats.x[i] = x(i);
		stats.xd[i] = xd(i);
	}
	for (int i = XDD_TARGETS*DOF; i < XDD_TARGETS*DOF+QDD_TARGETS; i++)
	{
		stats.x[i] = euler[i - XDD_TARGETS*DOF];
		stats.xd[i] = qvel[3 + i - XDD_TARGETS*DOF];
	}


	osc->RunPTSC(&dyn_model, &dyn_state, xdd, bActive, bDesiredContact, u);


	for (int i = 0; i < nU; i++)
		stats.torques[i] = telem->torques[i] = (*u)(i,0);
	for (int i = 0; i < XDD_TARGETS*DOF; i++)
	{
		telem->accels[i] = xdd(i);
		telem->targ_pos[i] = x_t(i);
	}
	for (int i = 0; i < XDD_TARGETS*DOF+QDD_TARGETS; i++)
	{
		stats.accels[i] = xdd(i);
		stats.targ_pos[i] = x_t(i);
		stats.targ_vel[i] = xd_t(i);
	}

}

void Cassie::StandingController(ControlObjective cntrl, Eigen::Matrix<double, nU, 1>* u)
{
	Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		bActive(i,0) = true;

	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> x_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS, 1> xd_t = Eigen::Matrix<double, DOF*XDD_TARGETS, 1>::Zero();
	Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd = Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1>::Zero();

	Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
	Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

	dyn_model.GetTargetPoints(&x, &xd, targetIds);

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
	x_t(0) = targBx;
	x_t(1) = targBy;
	x_t(2) = cntrl.bodyZPos;

	for (int i = 3; i < XDD_TARGETS*DOF; i++)
	{
		x_t(i) = x(i);
	}

	xdd(0,0) = PD_COM_X.Kp*(targBx - x(0)) + PD_COM_X.Kd*(0.0 - xd(0));
	xdd(1,0) = PD_COM_Y.Kp*(targBy - x(1)) + PD_COM_Y.Kd*(0.0 - xd(1));
	xdd(2,0) = PD_COM_Z.Kp*(cntrl.bodyZPos - x(2)) + PD_COM_Z.Kd*(cntrl.bodyZVel - xd(2)) + cntrl.bodyZAcc;

	//	printf("COM\n");
	//	printf("x: %f\t%f\t%f\t%f\n", targBx, qpos[0], qvel[0], xdd(0,0));
	//	printf("y: %f\t%f\t%f\t%f\n", targBy, qpos[1], qvel[1], xdd(1,0));
	//	printf("z: %f\t%f\t%f\t%f\n", cntrl.bodyZPos, qpos[2], qvel[2], xdd(2,0));

	for (int i = 0; i < nCON; i++)
	{
		xdd(3+DOF*i,0) = PD_StanceXY.Kd*(0.0 - xd(DOF*(i+1)));
		xdd(4+DOF*i,0) = PD_StanceXY.Kd*(0.0 - xd(DOF*(i+1)+1));
		xdd(5+DOF*i,0) = PD_StanceZ.Kp*(-0.005 - x(DOF*(i+1)+2)) + PD_StanceZ.Kd*(0.0 - xd(DOF*(i+1)+2));
		//		printf("Contact: %d\n",i);
		//		printf("x: %f\t%f\n", xd(DOF*(i+1)), xdd(3+DOF*i,0));
		//		printf("y: %f\t%f\n", xd(DOF*(i+1)+1), xdd(4+DOF*i,0));
		//		printf("z: %f\t%f\t%f\n", x(DOF*(i+1)+2), xd(DOF*(i+1)+2), xdd(5+DOF*i,0));
	}


	double quat[4];
	double euler[3];
	dyn_model.GetMainBodyQuaternion(quat);
	quaternionToEuler(quat, euler);
	xdd(15,0) = PD_Pitch.Kp*(0.0 - euler[0]) + PD_Pitch.Kd*(0.0 - qvel[3]);
	xdd(16,0) = PD_Pitch.Kp*(0.0 - euler[1]) + PD_Pitch.Kd*(0.0 - qvel[4]);
	xdd(17,0) = 0.0*PD_Pitch.Kp*(0.0 - euler[2]) + PD_Pitch.Kd*(0.0 - qvel[5]);


	for (int i = 0; i < XDD_TARGETS*DOF; i++)
	{
		stats.x[i] = x(i);
		stats.xd[i] = xd(i);
	}
	for (int i = XDD_TARGETS*DOF; i < XDD_TARGETS*DOF+QDD_TARGETS; i++)
	{
		stats.x[i] = euler[i - XDD_TARGETS*DOF];
		stats.xd[i] = qvel[3 + i - XDD_TARGETS*DOF];
	}

	//	printf("Attitude\n");
	//	printf("roll: %f\t%f\t%f\n", qpos[3], qvel[3], xdd(15,0));
	//	printf("pitch: %f\t%f\t%f\n", qpos[4], qvel[4], xdd(16,0));
	//	printf("yaw: %f\t%f\t%f\n", qpos[5], qvel[5], xdd(17,0));

	bool bInContact[] = {true, true, true, true};

	osc->RunPTSC(&dyn_model, &dyn_state, xdd, bActive, bInContact, u);

	//	printf("Torques:\n");
	//	printf("Left:\t");
	//	for (int i = 0; i < nU/2; i++)
	//		printf("%f\t", (*u)(i,0));
	//	printf("\nRight:\t");
	//	for (int i = nU/2; i < nU; i++)
	//		printf("%f\t", (*u)(i,0));
	//	printf("\n");

	for (int i = 0; i < nU; i++)
		stats.torques[i] = telem->torques[i] = (*u)(i,0);
	for (int i = 0; i < XDD_TARGETS*DOF; i++)
	{
		telem->accels[i] = xdd(i);
		telem->targ_pos[i] = x_t(i);
	}
	for (int i = 0; i < XDD_TARGETS*DOF+QDD_TARGETS; i++)
	{
		stats.accels[i] = xdd(i);
		stats.targ_pos[i] = x_t(i);
		stats.targ_vel[i] = xd_t(i);
	}
}

void Cassie::PlannerThread()
{
	//just wait until a new trajectory is received
	while (true)
	{
		CommandInterface::policy_params_t params;
		comms_planner->receive_policy_params(&params);
		m_bNewPlan = true;

		m_bTrajCopied = false;
		// Wait until the main process is ready for the new trajectory
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk, []{return m_bReadyToCopyTraj;});

		policy_opt->GetSolution(params.x, params.phases, params.num_phases, &targ_traj, 5e-3);

		targ_traj.run_count = params.run_count;

		m_bTrajCopied = true;

	}
}

void Cassie::logStats()
{
	for (int i = 0; i < nX; i++)
		logFile << stats.qpos[i] << ",";
	for (int i = 0; i < nQ; i++)
		logFile << stats.qvel[i] << ",";
	for (int i = 0; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		logFile << stats.targ_pos[i] << ",";
	for (int i = 0; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		logFile << stats.targ_vel[i] << ",";
	for (int i = 0; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		logFile << stats.accels[i] << ",";
	for (int i = 0; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		logFile << stats.x[i] << ",";
	for (int i = 0; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		logFile << stats.xd[i] << ",";
	for (int i = 0; i < nU; i++)
		logFile << stats.torques[i] << ",";
	logFile << stats.udp_dt_nsec << ",";
	logFile << stats.rbdl_dt_nsec << ",";
	logFile << stats.qp_dt_nsec << ",";
	logFile << stats.tx_dt_nsec << ",";
	logFile << stats.log_dt_nsec << std::endl;
}
