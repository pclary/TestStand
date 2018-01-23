#include <stdio.h>
#include <iostream>
#include <thread>

#include "udp_comms.h"
#include "Command_Structs.h"

#include "mujoco.h"
#include "CassieVis.h"
#include "SharedRobotDefinitions.h"
#include "CassieToMuJoCo.h"
#include "MPCOptions.h"

#include <thread>
#include <mutex>
#include <condition_variable>


using namespace std;

std::mutex mut;
std::condition_variable cv;

bool bReadyToCopyTraj = true;
CommandInterface::policy_params_t params;
ROM_Policy_Struct targ_traj;

void receive_policy(udp_comms* comms, MPC_OPTIONS* opt)
{
	while (true)
	{
		printf("waiting to recieve!\n");
		comms->receive_policy_params(&params);
		printf("received params!\n");
		// Wait until the main process is ready for the new trajectory
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk, []{return bReadyToCopyTraj;});

		opt->GetSolution(params.x, params.phases, params.num_phases, &targ_traj, 0.01);

	}
}

int main(int argc,char* argv[]) {

	bool bSaveVid = false;
	if (argc > 1)
	{
		if (strcmp(argv[1],"save") == 0)
			bSaveVid = true;
		else
		{
			printf("unknown option: %s\nexiting\n", argv[1]);
			return -1;
		}
	}
//	udp_comms* comms = new udp_comms("127.0.0.1", "127.0.0.1", 8880);
	udp_comms* comms = new udp_comms("192.168.1.101", "192.168.1.148", 8880);
	if (!comms->conn())
	{
		printf("Failed to connect... returning\n");
		return -1;
	}

//	udp_comms* comms_planner = new udp_comms("127.0.0.1", "127.0.0.1", 8886);
	udp_comms* comms_planner = new udp_comms("192.168.1.101", "192.168.1.200", 8886);
	if (!comms_planner->conn())
	{
		printf("Failed to connect to planner... returning\n");
		return -1;
	}

	while (true)
	{
		comms_planner->receive_policy_params(&params);
		printf("received params\n");
	}

	MPC_OPTIONS* opt = new MPC_OPTIONS();

	thread commThread = thread(&receive_policy, comms_planner, opt);

	mj_activate("../../ThirdParty/mjpro150/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mjModel* mj_Model = mj_loadXML(xml_model_filename.c_str(), 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return -1;
	}
	// Initialize mjData
	mjData* mj_Data = mj_makeData(mj_Model);

	CassieVis* vis = new CassieVis(mj_Model, bSaveVid, "remote_cassie");

	telemetry_t telem;

	while (true) {
		comms->receive_telemetry(&telem);

		for (int i = 0; i < nX; i++)
			mj_Data->qpos[i] = telem.qpos[i];

		mj_forward(mj_Model, mj_Data);

		bReadyToCopyTraj = false;
		vis->SetMPCPlan(&targ_traj);
		bReadyToCopyTraj = true;
		cv.notify_one();
		vis->Draw(mj_Data, telem);
	}

}
