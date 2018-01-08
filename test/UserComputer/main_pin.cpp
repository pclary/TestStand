/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

void profileCircle(ControlObjective* cntrl, double t)
{
	double midX = 0.0;
	double midZ = 0.4;

	double freq = 0.5;
	double amp = 0.2;

	double w = 2*freq*M_PI;

	cntrl->footPos[0] = midX + amp*cos(w*t);
	cntrl->footPos[1] = midZ + amp*sin(w*t);
	cntrl->footVel[0] = -w*amp*sin(w*t);
	cntrl->footVel[1] = w*amp*cos(w*t);
	cntrl->footAcc[0] = -w*w*amp*cos(w*t);
	cntrl->footAcc[1] = -w*w*amp*sin(w*t);
}

void profileLineX(ControlObjective* cntrl, double t)
{
	double midX = 0.0;
	double midZ = 0.4;

	double freq = 0.5;
	double amp = 0.2;

	double w = 2*freq*M_PI;

	cntrl->footPos[0] = midX + amp*cos(w*t);
	cntrl->footPos[1] = midZ;
	cntrl->footVel[0] = -w*amp*sin(w*t);
	cntrl->footVel[1] = 0.0;
	cntrl->footAcc[0] = -w*w*amp*cos(w*t);
	cntrl->footAcc[1] = 0.0;
}

void profileLineZ(ControlObjective* cntrl, double t)
{
	double midX = 0.0;
	double midZ = 0.4;

	double freq = 0.5;
	double amp = 0.2;

	double w = 2*freq*M_PI;

	cntrl->footPos[0] = midX;
	cntrl->footPos[1] = midZ + amp*sin(w*t);
	cntrl->footVel[0] = 0.0;
	cntrl->footVel[1] = w*amp*cos(w*t);
	cntrl->footAcc[0] = 0.0;
	cntrl->footAcc[1] = -w*w*amp*sin(w*t);
}

void profileSetPoint(ControlObjective* cntrl)
{
	double targX = 0.0;
	double targZ = 0.4;

	cntrl->footPos[0] = targX;
	cntrl->footPos[1] = targZ;

	for (int i = 0; i < 2; i++)
		cntrl->footVel[i] = cntrl->footAcc[i] = 0.0;
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

	double t = 0.0; //for time varying profiles (uses discrete 2kHz)

	while (true)
	{
		//uncomment the profile you want
		//if recompiling becomes a pain in the ass then just send in a single argument for what profile you want
		profileCircle(&cntrl, t);
//		profileLineX(&cntrl, t);
//		profileLineZ(&cntrl, t);
//		profileSetPoint(&cntrl);

		if (!robot.Run(cntrl))
		{
			printf("comms error quitting for safety reasons\n");
			return -1;
		}
		t += 0.0005;
	}

	return 0;

}



