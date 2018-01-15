#include <stdio.h>
#include <iostream>
#include <thread>

#include "udp_comms.h"
#include "Command_Structs.h"
#include <sys/time.h>
#include "HelperFunctions.h"
#include "mujoco.h"
#include "SharedRobotDefinitions.h"
#include "CassieToMuJoCo.h"

using namespace std;

int main(int argc,char* argv[]) {

//	udp_comms* comms = new udp_comms(true, 8888, "192.168.1.147");
	udp_comms* comms = new udp_comms(true, 8888, "127.0.0.1");

	if (!comms->conn())
	{
		printf("Failed to connect... returning\n");
		return -1;
	}

	mj_activate("../../ThirdParty/mjpro150/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mjModel* mj_Model = mj_loadXML(xml_model_filename.c_str(), 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return -1;
	}
	// Initialize mjData
	mjData* mj_Data = mj_makeData(mj_Model);

	if (nQ == 6)
	{
		mjtNum qpos_init[] = {0.81285567,0.57385684,-1.52914241,1.75122487,-1.66851819,-0.74119364};
		mju_copy(mj_Data->qpos, qpos_init, nQ);
	}

	mj_forward(mj_Model, mj_Data);


	cassie_user_in_t command;
	cassie_out_t sensors;

	//initiate comms
	StateToCassieOutputs(mj_Data->qpos, mj_Data->qvel, &sensors);
	comms->send_cassie_outputs(sensors);
	comms->send_cassie_outputs(sensors);

	timespec ts, tf;
	while (true) {

		clock_gettime(CLOCK_REALTIME, &ts);
		comms->receive_cassie_inputs(&command);

		CassieInputsToTorque(command, mj_Data->ctrl);
		mj_step(mj_Model, mj_Data);

		StateToCassieOutputs(mj_Data->qpos, mj_Data->qvel, &sensors);
		comms->send_cassie_outputs(sensors);

		unsigned int diff_us = 0;
		while (diff_us < 5e2)
		{
			clock_gettime(CLOCK_REALTIME, &tf);
			diff_us = (diff(ts,tf).tv_nsec)/1e3;
		}
//		printf("%u\n", diff_us);
	}

}
