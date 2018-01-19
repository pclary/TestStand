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

//static void CassieOutputsToState(mjModel* m, mjData* d, cassie_out_t sensors, double* qpos, double* qvel)
//{
//	qpos[1] = sensors.leftLeg.hipPitchDrive.position;
//	qvel[1] = sensors.leftLeg.hipPitchDrive.velocity;
//
//	qpos[2] = sensors.leftLeg.kneeDrive.position;
//	qvel[2] = sensors.leftLeg.kneeDrive.velocity;
//
//	qpos[3] = sensors.leftLeg.tarsusJoint.position;
//	qvel[3] = sensors.leftLeg.tarsusJoint.velocity;
//
//	qpos[4] = sensors.leftLeg.footJoint.position;
//	qvel[4] = sensors.leftLeg.footJoint.velocity;
//
//	qpos[0] = 0.0;
//	qpos[5] = 0.653 + 0.2269 - qpos[1] - qpos[2] - qpos[3];
//
//	for (int i = 0; i < nQ; i++)
//		d->qpos[i] = qpos[i];
//
//	mj_forward(m,d);
//
//	qpos[0] = 0.0 - std::min(d->site_xpos[1*3+2], d->site_xpos[2*3+2]);
//
//	double dY = d->efc_pos[0];//d->efc_J[nQ-1];
//	double dZ = -0.5012 - d->efc_pos[2];
//
//	double w_angle = atan2(-dY,-dZ);
//
//	qpos[5] = w_angle + 0.653 + 0.2269 - qpos[1] - qpos[2] - qpos[3]; //yay for magic numbers (offsets in xml)
//
//}

static void StateToCassieOutputs(double* qpos, double* qvel, cassie_out_t* sensors)
{
	for (int i = 0; i < 3; i++)
		sensors->pelvis.vectorNav.angularVelocity[i] = qvel[i+3];

	for (int i = 0; i < 4; i++)
		sensors->pelvis.vectorNav.orientation[i] = qpos[i+3];

	sensors->leftLeg.hipRollDrive.position = qpos[7];
	sensors->leftLeg.hipRollDrive.velocity = qvel[6];
	sensors->leftLeg.hipYawDrive.position = qpos[8];
	sensors->leftLeg.hipYawDrive.velocity = qvel[7];
	sensors->leftLeg.hipPitchDrive.position = qpos[9];
	sensors->leftLeg.hipPitchDrive.velocity = qvel[8];
	sensors->leftLeg.kneeDrive.position = qpos[10];
	sensors->leftLeg.kneeDrive.velocity = qvel[9];
	sensors->leftLeg.tarsusJoint.position = qpos[11];
	sensors->leftLeg.tarsusJoint.velocity = qvel[10];
	sensors->leftLeg.footJoint.position = qpos[12];
	sensors->leftLeg.footJoint.velocity = qvel[11];

	sensors->rightLeg.hipRollDrive.position = qpos[14];
	sensors->rightLeg.hipRollDrive.velocity = qvel[13];
	sensors->rightLeg.hipYawDrive.position = qpos[15];
	sensors->rightLeg.hipYawDrive.velocity = qvel[14];
	sensors->rightLeg.hipPitchDrive.position = qpos[16];
	sensors->rightLeg.hipPitchDrive.velocity = qvel[15];
	sensors->rightLeg.kneeDrive.position = qpos[17];
	sensors->rightLeg.kneeDrive.velocity = qvel[16];
	sensors->rightLeg.tarsusJoint.position = qpos[18];
	sensors->rightLeg.tarsusJoint.velocity = qvel[17];
	sensors->rightLeg.footJoint.position = qpos[19];
	sensors->rightLeg.footJoint.velocity = qvel[18];

//	printf("%f,%f\n", qpos[13], qpos[20]);
//
//	for (int i = 0; i < 3; i++)
//		printf("%f,",qpos[i]);
//	for (int i = 0; i < 3; i++)
//		printf("%f,",euler[i]);
//	for (int i = 0; i < nQ+1; i++)
//		printf("%f,",qpos[i]);
//	for (int i = 0; i < nQ-1; i++)
//		printf("%f,",qvel[i]);
//	printf("%f\n", qvel[nQ-1]);

}

//static void TorqueToCassieInputs(double* u, cassie_user_in_t* command)
//{
//	for (int i = 0; i < nU; i++)
//		command->torque[i] = u[i];
//}

static void CassieInputsToTorque(cassie_user_in_t command, double* u)
{
	for (int i = 0; i < nU; i++)
	{
		u[i] = command.torque[i];
//		printf("%f\t",u[i]);
	}
//	printf("\n");
}


#endif /* CASSIETOMUJOCO_H_ */
