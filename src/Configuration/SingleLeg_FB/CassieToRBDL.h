/*
 * CassieToRBDL.h
 *
 *  Created on: Jan 7, 2018
 *      Author: tapgar
 */

#ifndef CASSIETORBDL_H_
#define CASSIETORBDL_H_

#include "DynamicModel.h"
#include "RobotDefinitions.h"
#include <rbdl/rbdl.h>

static void CassieOutputsToState(DynamicModel* dyn, cassie_out_t sensors, double* qpos, double* qvel)
{
	//update directly measured states
	qpos[1] = sensors.leftLeg.hipPitchDrive.position;
	qvel[1] = sensors.leftLeg.hipPitchDrive.velocity;

	qpos[2] = sensors.leftLeg.kneeDrive.position;
	qvel[2] = sensors.leftLeg.kneeDrive.velocity;

	qpos[3] = sensors.leftLeg.tarsusJoint.position;
	qvel[3] = sensors.leftLeg.tarsusJoint.velocity;

	qpos[4] = sensors.leftLeg.footJoint.position;
	qvel[4] = sensors.leftLeg.footJoint.velocity;

	//qpos4
	qpos[5] = qpos[0] = 0.0;
	qvel[5] = qvel[0] = 0.0;

	dyn->setState(qpos, qvel);

	RigidBodyDynamics::Math::VectorNd constraint_pose = RigidBodyDynamics::Math::VectorNd::Zero(nEQ);
	RigidBodyDynamics::Math::VectorNd constraint_base_pose = RigidBodyDynamics::Math::VectorNd::Zero(nEQ);
	dyn->GetConstraintPointsDependent(&constraint_pose, &constraint_base_pose);

	double w_angle = atan2(constraint_pose(0) - constraint_base_pose(0), constraint_base_pose(2) - constraint_pose(2));

	qpos[5] = w_angle + 0.653 + 0.2269 - qpos[1] - qpos[2] - qpos[3]; //yay for magic numbers (offsets in xml)

	dyn->setState(qpos, qvel);

	RigidBodyDynamics::Math::MatrixNd Jeq = RigidBodyDynamics::Math::MatrixNd::Zero(nEQ, nQ); //constraint jacobian
	dyn->GetConstraintJacobian(&Jeq);

	RigidBodyDynamics::Math::MatrixNd Jeq_ind = Jeq.block<3,nQ-1>(0,0);
	RigidBodyDynamics::Math::MatrixNd Jeq_dep = Jeq.block<3,1>(0,nQ-1);
	RigidBodyDynamics::Math::MatrixNd gamma = RigidBodyDynamics::Math::MatrixNd::Zero(nQ, nQ-1);
	for (int i = 0; i < nQ-1; i++)
		gamma(i,i) = 1.0;

	gamma.block<1,nQ-1>(nQ-1,0) = -pseudoinverse(Jeq_dep)*Jeq_ind;

	RigidBodyDynamics::Math::VectorNd qdot_ind = RigidBodyDynamics::Math::VectorNd::Zero(nQ-1);//ind vel
	for (int i = 0; i < nQ-1; i++)
		qdot_ind(i) = qvel[i];

	RigidBodyDynamics::Math::VectorNd qdot = gamma*qdot_ind;

	for (int i = 0; i < nQ; i++)
		qvel[i] = qdot(i);

	RigidBodyDynamics::Math::VectorNd x = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);
	RigidBodyDynamics::Math::VectorNd xd = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);

	dyn->GetTargetPoints(&x, &xd);

	qpos[0] = -(x(5)+x(8))/2.0;
	qvel[0] = -(xd(5)+xd(8))/2.0;

//	qpos[0] = sensors.motorPosition[1];
//	qvel[0] = sensors.motorVelocity[1];

	printf("%f,%f\n", qpos[5], qvel[5]);

//	if (x(5) < x(8))
//	{
//		qpos[0] = -x(5);
//		qvel[0] = -xd(5);
//	}
//	else
//	{
//		qpos[0] = -x(8);
//		qvel[0] = -xd(8);
//	}

//	for(int i = 0; i < nQ; i++)
//		printf("%f\t", qpos[i]);
//	printf("\n");

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


#endif /* CASSIETORBDL_H_ */
