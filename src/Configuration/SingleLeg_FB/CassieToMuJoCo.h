/*
 * CassieToMuJoCo.h
 *
 *  Created on: Jan 7, 2018
 *      Author: tapgar
 */

#ifndef CASSIETOMUJOCO_H_
#define CASSIETOMUJOCO_H_

#include "mujoco.h"
#include "RobotDefinitions.h"

static void CassieOutputsToState(mjModel* m, mjData* d, cassie_outputs_t sensors, double* qpos, double* qvel)
{

	for (int i = 0; i < nU; i++)
		qpos[i+1] = sensors.motorPosition[i+2];

	qpos[0] = 0.0;
	qpos[4] = sensors.jointPosition[1];
	qpos[5] = 0.653 + 0.2269 - qpos[1] - qpos[2] - qpos[3];

	for (int i = 0; i < nQ; i++)
		d->qpos[i] = qpos[i];

	mj_forward(m,d);

	qpos[0] = 0.0 - std::min(d->site_xpos[1*3+2], d->site_xpos[2*3+2]);

	double dY = d->efc_pos[0];//d->efc_J[nQ-1];
	double dZ = -0.5012 - d->efc_pos[2];

	double w_angle = atan2(-dY,-dZ);

	qpos[5] = w_angle + 0.653 + 0.2269 - qpos[1] - qpos[2] - qpos[3]; //yay for magic numbers (offsets in xml)

}

static void StateToCassieOutputs(double* qpos, double* qvel, cassie_outputs_t* sensors)
{
	for (int i = 0; i < nU; i++)
	{
		sensors->motorPosition[i+2] = qpos[i+1];
		sensors->motorVelocity[i+2] = qvel[i+1];
	}
	sensors->jointPosition[1] = qpos[4];
	sensors->jointVelocity[1] = qvel[4];


//	sensors->motorPosition[1] = qpos[0];
//	sensors->motorVelocity[1] = qvel[0];
//	printf("%f,%f\n", qpos[5], qvel[5]);


//	for(int i = 0; i < nQ; i++)
//		printf("%f\t", qpos[i]);
//	printf("\n");
}

static void TorqueToCassieInputs(double* u, cassie_inputs_t* command)
{
	for (int i = 2; i < 2+nU; i++)
		command->torque[i] = u[i-2];
}

static void CassieInputsToTorque(cassie_inputs_t command, double* u)
{
	for (int i = 2; i < 2+nU; i++)
		u[i-2] = command.torque[i];
}


#endif /* CASSIETOMUJOCO_H_ */
