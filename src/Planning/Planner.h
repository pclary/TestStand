/*
 * Planner.h
 *
 *  Created on: Dec 8, 2017
 *      Author: tapgar
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include "MPC.h"
#include "CommandInterface.h"
#include <fstream>
#include "udp_comms.h"
#include <thread>
#include <mutex>
#include <condition_variable>


#define LOGGING false

class Planner {
public:
	Planner();
	virtual ~Planner();

	bool Init();
	void Run();

	bool isReady() { return bReadyToCopyState; };

private:
	CommandInterface::StateInfo_Struct state_info;

	CommandInterface::policy_params_t policy_params;

	udp_comms* comms;
	udp_comms* comms_vis;

	Ipopt::MPC mpc;

	std::mutex mut;
	std::condition_variable cv;

	MPC_OPTIONS* opt;

	bool bReadyToCopyState;

	std::thread commThread;

	bool UpdateContactSchedule();

	bool Update();
	void SendUIUpdate();
	void SendRobotUpdate();

	void ReceiveState();

#if LOGGING
	void LogTrajectory();
	ofstream fileTraj;
#endif

};

#endif /* PLANNER_H_ */
