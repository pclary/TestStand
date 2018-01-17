/*
 * main.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: tapgar
 */

#include "TestBenchInterface.h"

void profileCircle(ControlObjective* cntrl, double t, double* bRadio)
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
	cntrl->bSwitchModes = false;
}

void profileLineX(ControlObjective* cntrl, double t, double* bRadio)
{
	static double midX = 0.0;
	static double midZ = 0.4;

	if (cntrl->bSwitchModes)
	{
		midX = cntrl->footPos[0];
		midZ = cntrl->footPos[1];
	}

	double freq = cntrl->freq;
	double amp = cntrl->amp;

	double w = 2*freq*M_PI;

	cntrl->footPos[0] = midX + amp*sin(w*t);
	cntrl->footPos[1] = midZ;
	cntrl->footVel[0] = w*amp*cos(w*t);
	cntrl->footVel[1] = 0.0;
	cntrl->footAcc[0] = -w*w*amp*sin(w*t);
	cntrl->footAcc[1] = 0.0;
	cntrl->bSwitchModes = false;
}

void profileLineZ(ControlObjective* cntrl, double t, double* bRadio)
{
	static double midX = 0.0;
	static double midZ = 0.4;

	if (cntrl->bSwitchModes)
	{
		midX = cntrl->footPos[0];
		midZ = cntrl->footPos[1];
	}

	double freq = cntrl->freq;
	double amp = cntrl->amp;

	double w = 2*freq*M_PI;

	cntrl->footPos[0] = midX;
	cntrl->footPos[1] = midZ + amp*sin(w*t);
	cntrl->footVel[0] = 0.0;
	cntrl->footVel[1] = w*amp*cos(w*t);
	cntrl->footAcc[0] = 0.0;
	cntrl->footAcc[1] = -w*w*amp*sin(w*t);
	cntrl->bSwitchModes = false;
}

void profileSetPoint(ControlObjective* cntrl, double* bRadio)
{
        static double deltaX = 0.0;
        static double deltaZ = 0.0;

	static double dPosX = 0.0;
	static double dPosZ = 0.0;
	if (!cntrl->bSwitchModes)
	{
		dPosX = cntrl->footPos[0];
		dPosZ = cntrl->footPos[1];
		deltaX = 0.0;
		deltaZ = 0.0;
	}

        deltaX += 0.05*bRadio[3];
        deltaZ += 0.05*bRadio[2];

        cntrl->footPos[0] = dPosX + 0.01*deltaX;
        cntrl->footPos[1] = dPosZ + 0.01*deltaZ;

	for (int i = 0; i < 2; i++)
		cntrl->footVel[i] = cntrl->footAcc[i] = 0.0;
	cntrl->bSwitchModes = false;
}

int main() {

	TestBenchInterface robot;

	if (!robot.Init())
	{
		printf("robot init failed... quitting\n");
		return -1;
	}

	ControlObjective cntrl;
	cntrl.footPos[0] = 0.0;
	cntrl.footPos[1] = 0.2;
	cntrl.freq = 0.1;
	cntrl.amp = 0.05;
	cntrl.bSwitchModes = true;

	robot.SetFootGainX(100.0, 10.0);
	robot.SetFootGainZ(100.0, 10.0);

	double t = 0.0; //for time varying profiles (uses discrete 2kHz)

	double bRadio[16];
	for (int i = 0; i < 16; i++)
		bRadio[i] = 0.0;

	while (true)
	{

		cntrl.freq = 0.9;//0.5 + 0.4*bRadio[6];
		cntrl.amp = 0.2;//0.3 + 0.25*bRadio[7];

		if (bRadio[10] < -0.1)
			profileLineX(&cntrl, t, bRadio);
		else if (bRadio[10] > 0.1)
			profileLineZ(&cntrl, t, bRadio);
		else
		{
//			cntrl.freq = 0.1;
//			cntrl.amp = 0.05;
			t = 0.0;
			profileSetPoint(&cntrl, bRadio);
			cntrl.bSwitchModes = true;
		}
//		profileCircle(&cntrl, t, bRadio);

		if (!robot.Run(cntrl, bRadio))
		{
			printf("comms error quitting for safety reasons\n");
			return -1;
		}

		double Kp = 200.0 + 100.0*bRadio[4];
	        double Kd = 20.0 + 10.0*bRadio[5];

		robot.SetFootGainX(Kp, Kd);
		robot.SetFootGainZ(Kp, Kd);

		t += 0.0005;
	}

	return 0;

}



