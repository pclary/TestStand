/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

void profileLineZ(ControlObjective* cntrl, double t)
{
	double midZ = 0.7;

	double freq = 0.5;
	double amp = 0.2;

	double w = 2*freq*M_PI;

	cntrl->bodyZPos = midZ + amp*sin(w*t);
	cntrl->bodyZVel = w*amp*cos(w*t);
	cntrl->bodyZAcc = -w*w*amp*sin(w*t) + 9.806;
}

void profileSetPoint(ControlObjective* cntrl)
{
	double targZ = 0.9;

	cntrl->bodyZPos = targZ;
	cntrl->bodyZVel = 0.0;
	cntrl->bodyZAcc = 9.806;
}

int main() {

	TestBenchInterface robot;

	if (!robot.Init())
	{
		printf("robot init failed... quitting\n");
		return -1;
	}

	ControlObjective cntrl;

	robot.SetCOMGains(100.0, 20.0);
	robot.SetStanceGains(100.0, 30.0);
	robot.SetJointGains(100.0, 20.0);

	double t = 0.0; //for time varying profiles (uses discrete 2kHz)

	while (true)
	{
		//uncomment the profile you want
		//if recompiling becomes a pain in the ass then just send in a single argument for what profile you want
//		profileLineZ(&cntrl, t);
		profileSetPoint(&cntrl);

		if (!robot.Run(cntrl))
		{
			printf("comms error quitting for safety reasons\n");
			return -1;
		}
		t += 0.0005;
	}

	return 0;

}



