#include <stdio.h>
#include <iostream>
#include <thread>

#include "udp_comms.h"
#include "Command_Structs.h"
#include <mutex>          // std::mutex

#include "mujoco.h"
#include "Visualizer.h"
#include "SharedRobotDefinitions.h"
#include "CassieToMuJoCo.h"

using namespace std;

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

	udp_comms* comms = new udp_comms(false, 8880, "192.168.1.200");
//	udp_comms* comms = new udp_comms(false, 8880, "127.0.0.1");
	if (!comms->conn())
	{
		printf("Failed to connect... returning\n");
		return -1;
	}

	mj_activate("/home/tapgar/.mujoco/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mjModel* mj_Model = mj_loadXML(xml_model_filename.c_str(), 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return -1;
	}
	// Initialize mjData
	mjData* mj_Data = mj_makeData(mj_Model);

	Visualizer* vis = new Visualizer(mj_Model, bSaveVid, "remote_cassie");

	telemetry_t telem;

	while (true) {
		comms->receive_telemetry(&telem);
		for (int i = 0; i < nQ; i++)
			mj_Data->qpos[i] = telem.qpos[i];

		mj_forward(mj_Model, mj_Data);
//		for (int i = 0; i < XDD_TARGETS*DOF; i++)
//			printf("%f\t", telem.targ_pos[i]);
//		printf("\n");

//		for (int i = 0; i < nQ; i++)
//			printf("%f\t%f\n", mj_Data->qpos[i], mj_Data->qvel[i]);
		vis->DrawPinned(mj_Data, telem);
//		vis->Draw(mj_Data);
	}

}
