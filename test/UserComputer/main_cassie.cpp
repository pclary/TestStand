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

void profileSetPoint(ControlObjective* cntrl, double* bRadio)
{
	double targZ = 0.7 + 0.3*bRadio[7];

	cntrl->bodyZPos = targZ;
	cntrl->bodyZVel = 0.0;
	cntrl->bodyZAcc = 0.0;//9.806;
}

int main() {

	TestBenchInterface robot;

	if (!robot.Init())
	{
		printf("robot init failed... quitting\n");
		return -1;
	}

	ControlObjective cntrl;

	PD_CONTROLLER PD_COM_X;
	PD_COM_X.Kp = 10.0; PD_COM_X.Kd = 1.0;
	PD_CONTROLLER PD_COM_Y;
	PD_COM_Y.Kp = 10.0; PD_COM_Y.Kd = 1.0;
	PD_CONTROLLER PD_COM_Z;
	PD_COM_Z.Kp = 10.0; PD_COM_Z.Kd = 1.0;
	PD_CONTROLLER PD_Pitch;
	PD_Pitch.Kp = 10.0; PD_Pitch.Kd = 1.0;
	PD_CONTROLLER PD_StanceXY;
	PD_StanceXY.Kp = 10.0; PD_StanceXY.Kd = 0.1;
	PD_CONTROLLER PD_StanceZ;
	PD_StanceZ.Kp = 10.0; PD_StanceZ.Kd = 1.0;

	robot.SetCOMXGains(PD_COM_X.Kp, PD_COM_X.Kd);
	robot.SetCOMYGains(PD_COM_Y.Kp, PD_COM_Y.Kd);
	robot.SetCOMZGains(PD_COM_Z.Kp, PD_COM_Z.Kd);
	robot.SetStanceXYGains(PD_StanceXY.Kp, PD_StanceXY.Kd);
	robot.SetStanceZGains(PD_StanceZ.Kp, PD_StanceZ.Kd);
	robot.SetJointGains(PD_Pitch.Kp, PD_Pitch.Kd);

	double t = 0.0; //for time varying profiles (uses discrete 2kHz)

	double bRadio[16];

	while (true)
	{
		//uncomment the profile you want
		//if recompiling becomes a pain in the ass then just send in a single argument for what profile you want
//		profileLineZ(&cntrl, t);
		profileSetPoint(&cntrl, bRadio);

		if (!robot.Run(cntrl, bRadio))
		{
			printf("comms error quitting for safety reasons\n");
			return -1;
		}
		t += 0.0005;

		cntrl.idx = (unsigned int)((-1.0*bRadio[6] + 1.0)*3.0 - 1e-3);
		double sf = 1e-2;

		switch (cntrl.idx)
		{
		case 0:
			if (bRadio[15] > 0.0)
				PD_COM_X.Kp += sf * bRadio[0];
			else
				PD_COM_X.Kd += sf * bRadio[0];
			robot.SetCOMXGains(PD_COM_X.Kp, PD_COM_X.Kd);
			break;

		case 1:
			if (bRadio[15] > 0.0)
				PD_COM_Y.Kp += sf * bRadio[0];
			else
				PD_COM_Y.Kd += sf * bRadio[0];
			robot.SetCOMYGains(PD_COM_Y.Kp, PD_COM_Y.Kd);
			break;

		case 2:
			if (bRadio[15] > 0.0)
				PD_COM_Z.Kp += sf * bRadio[0];
			else
				PD_COM_Z.Kd += sf * bRadio[0];
			robot.SetCOMZGains(PD_COM_Z.Kp, PD_COM_Z.Kd);
			break;

		case 3:
			if (bRadio[15] > 0.0)
				PD_StanceXY.Kp += sf * bRadio[0];
			else
				PD_StanceXY.Kd += sf * bRadio[0];
			robot.SetStanceXYGains(PD_StanceXY.Kp, PD_StanceXY.Kd);
			break;

		case 4:
			if (bRadio[15] > 0.0)
				PD_StanceZ.Kp += sf * bRadio[0];
			else
				PD_StanceZ.Kd += sf * bRadio[0];
			robot.SetStanceZGains(PD_StanceZ.Kp, PD_StanceZ.Kd);
			break;

		case 5:
			if (bRadio[15] > 0.0)
				PD_Pitch.Kp += sf * bRadio[0];
			else
				PD_Pitch.Kd += sf * bRadio[0];
			robot.SetJointGains(PD_Pitch.Kp, PD_Pitch.Kd);
			break;
		}

	}

	return 0;

}



