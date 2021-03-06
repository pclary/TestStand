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

bool bProcMessage = false;
bool bReadyToCopyTraj = true;
bool bCopiedTraj = false;
CommandInterface::policy_params_t params;
ROM_Policy_Struct targ_traj;

//void receive_policy(udp_comms* comms, MPC_OPTIONS* opt)
//{
//	while (true)
//	{
//		bProcMessage = true;
//		comms->receive_policy_params(&params);
//		bProcMessage = false;
//
//		// Wait until the main process is ready for the new trajectory
//		std::unique_lock<std::mutex> lk(mut);
//		cv.wait(lk, []{return bReadyToCopyTraj;});
//
//		opt->GetSolution(params.x, params.phases, params.num_phases, &targ_traj, 0.01);
//
//		bCopiedTraj = true;
//		lk.unlock();
//		cv.notify_one();
//	}
//}

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

	MPC_OPTIONS* opt = new MPC_OPTIONS();
	opt->num_phases = 17;

	targ_traj.dt_c = 0.0;
	ROM_TrajPt_Struct null_rom;
	for (int i = 0; i < MAX_TRAJ_PTS; i++)
		targ_traj.com_traj.push_back(null_rom);
	ContactInfo_Struct null_con;
	for (int i = 0; i < MAX_CON_SWITCH; i++)
		targ_traj.con_sched.push_back(null_con);

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

		if (comms_planner->rcv_data_available())
		{
			comms_planner->receive_policy_params(&params);
			opt->GetSolution(params.x, params.phases, params.num_phases, &targ_traj, 0.01);
			vis->SetMPCPlan(&targ_traj);
		}

		for (int i = 0; i < nX; i++)
			mj_Data->qpos[i] = telem.qpos[i];

		mj_forward(mj_Model, mj_Data);

//		{
//			std::lock_guard<std::mutex> lk(mut);
//
//			if (!bProcMessage)
//			{
//				bReadyToCopyTraj = true;
//				cv.notify_one();
//			}
//		}
//		if (!bProcMessage)
//		{
//			std::unique_lock<std::mutex> lk(mut);
//			cv.wait(lk, []{return bCopiedTraj;});
//			vis->SetMPCPlan(&targ_traj);
//			bReadyToCopyTraj = false;
//		}
		vis->Draw(mj_Data, telem);
	}

}
