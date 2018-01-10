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

static void CassieOutputsToState(mjModel* m, mjData* d, cassie_out_t sensors, double* qpos, double* qvel)
{
	qpos[1] = sensors.leftLeg.hipPitchDrive.position;
	qvel[1] = sensors.leftLeg.hipPitchDrive.velocity;

	qpos[2] = sensors.leftLeg.kneeDrive.position;
	qvel[2] = sensors.leftLeg.kneeDrive.velocity;

	qpos[3] = sensors.leftLeg.tarsusJoint.position;
	qvel[3] = sensors.leftLeg.tarsusJoint.velocity;

	qpos[4] = sensors.leftLeg.footJoint.position;
	qvel[4] = sensors.leftLeg.footJoint.velocity;

	qpos[0] = 0.0;
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

static void StateToCassieOutputs(double* qpos, double* qvel, cassie_out_t* sensors)
{
	sensors->leftLeg.hipPitchDrive.position = qpos[1];
	sensors->leftLeg.hipPitchDrive.velocity = qvel[1];

	sensors->leftLeg.kneeDrive.position = qpos[2];
	sensors->leftLeg.kneeDrive.velocity = qvel[2];

	sensors->leftLeg.tarsusJoint.position = qpos[3];
	sensors->leftLeg.tarsusJoint.velocity = qvel[3];

	sensors->leftLeg.footJoint.position = qpos[4];
	sensors->leftLeg.footJoint.velocity = qvel[4];

	printf("%f,%f\n", qpos[5], qvel[5]);
}

static void TorqueToCassieInputs(double* u, cassie_user_in_t* command)
{
	for (int i = 2; i < 2+nU; i++)
		command->torque[i] = u[i-2];
}

static void CassieInputsToTorque(cassie_user_in_t command, double* u)
{
	for (int i = 2; i < 2+nU; i++)
		u[i-2] = command.torque[i];
}


#endif /* CASSIETOMUJOCO_H_ */
