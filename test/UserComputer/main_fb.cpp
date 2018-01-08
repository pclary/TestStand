/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

void profileBodyLineZ(ControlObjective* cntrl, double t)
{
	double midZ = 0.6;

	double freq = 0.5;
	double amp = 0.15;

	double w = 2*freq*M_PI;

	for (int i = 0; i < 2; i++)
		cntrl->footPos[i] = cntrl->footVel[i] = cntrl->footAcc[i];
	cntrl->footPos[1] = -0.005;

	cntrl->bContact = true;

	cntrl->bodyZPos = midZ + amp*sin(w*t);
	cntrl->bodyZVel = w*amp*cos(w*t);
	cntrl->bodyZAcc = -w*w*amp*sin(w*t) + 9.806;
}

void profileBodySetPoint(ControlObjective* cntrl)
{
	double targZ = 0.7;

	for (int i = 0; i < 2; i++)
		cntrl->footPos[i] = cntrl->footVel[i] = cntrl->footAcc[i];
	cntrl->footPos[1] = -0.005;

	cntrl->bContact = true;
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

	robot.SetFootGainX(100.0, 50.0);
	robot.SetFootGainZ(100.0, 50.0);
	robot.SetBodyGainZ(100.0, 50.0);
//	robot.SetBodyGainZ(0.0, 0.0);

	double t = 0.0; //for time varying profiles (uses discrete 2kHz)

	while (true)
	{
		//uncomment the profile you want
		//if recompiling becomes a pain in the ass then just send in a single argument for what profile you want
		profileBodyLineZ(&cntrl, t);
//		profileBodySetPoint(&cntrl);

		if (!robot.Run(cntrl))
		{
			printf("comms error quitting for safety reasons\n");
			return -1;
		}
		t += 0.0005;
	}

	return 0;

}



