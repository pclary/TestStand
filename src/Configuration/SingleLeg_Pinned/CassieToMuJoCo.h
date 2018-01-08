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
		qpos[i] = sensors.motorPosition[i+2];
	qpos[3] = sensors.jointPosition[1];
	qpos[4] = 0.653 + 0.2269 - qpos[0] - qpos[1] - qpos[2];

	for (int i = 0; i < nQ; i++)
		d->qpos[i] = qpos[i];

	mj_forward(m,d);

	double dY = d->efc_pos[0];//d->efc_J[nQ-1];
	double dZ = -0.5012 - d->efc_pos[2];

	double w_angle = atan2(-dY,-dZ);

	qpos[4] = w_angle + 0.653 + 0.2269 - qpos[0] - qpos[1] - qpos[2]; //yay for magic numbers (offsets in xml)

	//you don't use vel in mujoco anyways
}

static void StateToCassieOutputs(double* qpos, double* qvel, cassie_outputs_t* sensors)
{
	for (int i = 0; i < nU; i++)
	{
		sensors->motorPosition[i+2] = qpos[i];
		sensors->motorVelocity[i+2] = qvel[i];
	}
	sensors->jointPosition[1] = qpos[3];
	sensors->jointVelocity[1] = qvel[3];

//	printf("%f\n", qvel[4]);

//	printf("%f,%f,%f,%f,%f\n",qpos[0],qpos[1],qpos[2],qpos[3],qpos[4]);
//	for (int i = 0; i < nQ; i++)
//		printf("%f\t%f\n", qpos[i], qvel[i]);
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
