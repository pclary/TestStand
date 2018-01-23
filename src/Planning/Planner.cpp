/*
 * Planner.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: tapgar
 */

#include "Planner.h"
#include <unistd.h>

using namespace std;

Planner::Planner() {
	// TODO Auto-generated constructor stub
	opt = new MPC_OPTIONS();
	mpc.Init();

	bReadyToCopyState = true;
	cv.notify_one();

#if LOGGING
	fileTraj.open("traj.csv", std::ofstream::out);
#endif

	comms = new udp_comms("192.168.1.200", "192.168.1.148", 8888);
	comms_vis = new udp_comms("192.168.1.200", "192.168.1.101", 8886);

//	comms = new udp_comms("127.0.0.1", "127.0.0.1", 8888);
//	comms_vis = new udp_comms("127.0.0.1", "127.0.0.1", 8886);
}

Planner::~Planner() {
	// TODO Auto-generated destructor stub
}

bool Planner::Init() {

	if (!comms->conn())
	{
		printf("couldn't connect to robot\n");
		return false;
	}

	if (!comms_vis->conn())
	{
		printf("couldn't connect to visualizer\n");
		return false;
	}


	commThread = thread(&Planner::ReceiveState, this);

	return true;

}

void Planner::Run()
{
	bool bSuccess = Update();
	bSuccess = true;
	if (bSuccess)
	{
		mpc.GetParams(&policy_params);
		comms->send_policy_params(policy_params);
		comms_vis->send_policy_params(policy_params);
		printf("SENT PARAMS!\n");
	}
}

bool Planner::Update() {

	//step timing.... target pos... etc

	bReadyToCopyState = false;

	policy_params.run_count = state_info.run_count;

	for (int i = 0; i < 4; i++)
	{
		opt->x0[i] = state_info.com[i];
		opt->x0[i+4] = state_info.com_vel[i];
	}
	int idx[] = {0, 1, 3};
	for (int i = 0; i < 3; i++)
	{
		opt->x0[i+8] = state_info.left[idx[i]];
		opt->x0[i+11] = state_info.right[idx[i]];
	}

	opt->xT[0] = opt->x0[0] + state_info.xT[0];
	opt->xT[1] = opt->x0[1] + state_info.xT[1];
	opt->xT[2] = opt->x0[2] + state_info.xT[2];
	opt->xT[3] = opt->x0[3];

	//if the contact schedule doesn't change from previous then a warmstart is simple
	bool bWarmStart = UpdateContactSchedule();

	bReadyToCopyState = true;
	cv.notify_one();

	mpc.SetProblem(opt, bWarmStart);

	bool bSuccess = mpc.Run();

	return bSuccess;

}

bool Planner::UpdateContactSchedule()
{
	static uint32_t last_rcv_run_count = 0;

	bool bWarmStart = false;

	double step_time = state_info.step_time;
	double ds_perc = state_info.ds_perc;
	opt->num_phases = 17;

	bool bLastLeft = true;
	if (state_info.eOpState == CommandInterface::Idle || state_info.eOpState == CommandInterface::Standing)
	{
		//create new plan
		for (int i = 0; i < opt->num_phases/2; i++)
		{
			opt->phase[i*2].eType = Double;
			if (i == 0)
				opt->phase[i*2].T = 0.2;
			else
				opt->phase[i*2].T = ds_perc*step_time;

			if (bLastLeft)
				opt->phase[i*2+1].eType = SS_Right;
			else
				opt->phase[i*2+1].eType = SS_Left;
			opt->phase[i*2+1].T = (1.0 - ds_perc)*step_time;
			bLastLeft = !bLastLeft;
		}
		if (opt->num_phases % 2)
		{
			opt->phase[opt->num_phases-1].eType = Double;
			opt->phase[opt->num_phases-1].T = 0.2;
		}
	}
	else {
		//otherwise phases have been set before
		double phaseRunTime_s = double(state_info.run_count - last_rcv_run_count)*0.0005;
		double dT = 0.0;
		int nActiveIdx = 0;
		double deltaT = 0.0;
		for (int i = 0; i < opt->num_phases; i++)
		{
			dT += opt->phase[i].T;
			if (phaseRunTime_s < dT)
			{
				nActiveIdx = i;
				deltaT = dT - phaseRunTime_s;
				break;
			}
		}

		if (nActiveIdx > 0)
		{
			int nStartIdx = 0;
			//nActiveIdx shift
			for (int i = 0; i < opt->num_phases-nActiveIdx; i++)
				opt->phase[i] = opt->phase[i+nActiveIdx];

			if (opt->phase[0].eType == SS_Left || opt->phase[0].eType == SS_Right)
			{
				bLastLeft = (opt->phase[0].eType == SS_Left);
				opt->num_phases = 16;
				//create new plan
				for (int i = 1; i < opt->num_phases/2; i++)
				{
					opt->phase[i*2+1].eType = Double;
					opt->phase[i*2+1].T = ds_perc*step_time;

					if (bLastLeft)
						opt->phase[i*2].eType = SS_Right;
					else
						opt->phase[i*2].eType = SS_Left;
					opt->phase[i*2].T = (1.0 - ds_perc)*step_time;
					bLastLeft = !bLastLeft;
				}
			}
			else
			{
				bLastLeft = !(opt->phase[1].eType != SS_Left);
				opt->num_phases = 17;
				for (int i = 1; i < opt->num_phases/2; i++)
				{
					opt->phase[i*2].eType = Double;
					opt->phase[i*2].T = ds_perc*step_time;

					if (bLastLeft)
						opt->phase[i*2+1].eType = SS_Right;
					else
						opt->phase[i*2+1].eType = SS_Left;
					opt->phase[i*2+1].T = (1.0 - ds_perc)*step_time;
					bLastLeft = !bLastLeft;
				}
			}
			opt->phase[opt->num_phases-1].eType = Double;
			opt->phase[opt->num_phases-1].T = 0.2;
		}
		else
			bWarmStart = true;

		opt->phase[0].T = deltaT;
	}
	last_rcv_run_count = state_info.run_count;

	return bWarmStart;
}

void Planner::ReceiveState()
{
	while (true)
	{
		CommandInterface::StateInfo_Struct temp_info;
		comms->receive_state_info(&temp_info);

		// Wait until the main process is ready for the new trajectory
		std::unique_lock<std::mutex> lk(mut);
//		cv.wait(lk, []{return bReadyToCopyState;});
		cv.wait(lk, std::bind(&Planner::isReady, this));

		state_info = temp_info;
	}
}

#if LOGGING
void Planner::LogTrajectory() {
	fileTraj << targ_traj.numPoints << std::endl;
	for (int i = 0; i < targ_traj.numPoints; i++)
	{
		for (int j = 0; j < 4; j++)
			fileTraj << targ_traj.com_traj[i].com[j] << ",";
		for (int j = 0; j < 3; j++)
			fileTraj << targ_traj.com_traj[i].com_xdd[j] << ",";
		fileTraj << targ_traj.com_traj[i].com_xdd[3] << std::endl;
	}
}
#endif
