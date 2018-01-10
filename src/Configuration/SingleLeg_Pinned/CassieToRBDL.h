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
	qpos[0] = sensors.leftLeg.hipPitchDrive.position;
	qvel[0] = sensors.leftLeg.hipPitchDrive.velocity;

	qpos[1] = sensors.leftLeg.kneeDrive.position;
	qvel[1] = sensors.leftLeg.kneeDrive.velocity;

	qpos[2] = sensors.leftLeg.tarsusJoint.position;
	qvel[2] = sensors.leftLeg.tarsusJoint.velocity;

	qpos[3] = sensors.leftLeg.footJoint.position;
	qvel[3] = sensors.leftLeg.footJoint.velocity;

	//qpos4
	qpos[4] = 0.0;
	qvel[4] = 0.0;

	dyn->setState(qpos, qvel);

	RigidBodyDynamics::Math::VectorNd constraint_pose = RigidBodyDynamics::Math::VectorNd::Zero(nEQ);
	RigidBodyDynamics::Math::VectorNd constraint_base_pose = RigidBodyDynamics::Math::VectorNd::Zero(nEQ);
	dyn->GetConstraintPointsDependent(&constraint_pose, &constraint_base_pose);

	double w_angle = atan2(constraint_pose(0) - constraint_base_pose(0), constraint_base_pose(2) - constraint_pose(2));

	qpos[4] = w_angle + 0.653 + 0.2269 - qpos[0] - qpos[1] - qpos[2]; //yay for magic numbers (offsets in xml)

	dyn->setState(qpos, qvel);

	RigidBodyDynamics::Math::MatrixNd Jeq = RigidBodyDynamics::Math::MatrixNd::Zero(nEQ, nQ); //constraint jacobian
//	RigidBodyDynamics::Math::VectorNd Jeqdotq = RigidBodyDynamics::Math::VectorNd::Zero(nEQ);
	dyn->GetConstraintJacobian(&Jeq);
//	dyn->GetConstraintVel(&Jeqdotq);


	RigidBodyDynamics::Math::MatrixNd Jeq_ind = Jeq.block<3,nQ-1>(0,0);
	RigidBodyDynamics::Math::MatrixNd Jeq_dep = Jeq.block<3,1>(0,nQ-1);
	RigidBodyDynamics::Math::MatrixNd gamma = RigidBodyDynamics::Math::MatrixNd::Zero(nQ, nQ-1);
	for (int i = 0; i < nQ-1; i++)
		gamma(i,i) = 1.0;
//	std::cout << Jeq << std::endl;
//	std::cout << Jeq_ind << std::endl;
//	std::cout << Jeq_dep << std::endl;
//	std::cout << gamma << std::endl;
	gamma.block<1,nQ-1>(nQ-1,0) = -pseudoinverse(Jeq_dep)*Jeq_ind;

//	std::cout << gamma << std::endl;

	RigidBodyDynamics::Math::VectorNd qdot_ind = RigidBodyDynamics::Math::VectorNd::Zero(nQ-1);//ind vel
	for (int i = 0; i < nQ-1; i++)
		qdot_ind(i) = qvel[i];

	RigidBodyDynamics::Math::VectorNd qdot = gamma*qdot_ind;

	for (int i = 0; i < nQ; i++)
		qvel[i] = qdot(i);
//	printf("%f\n", qpos[4]);
//	printf("%f\t%f\t%f\t%f\n", constraint_base_pose(0), constraint_base_pose(2), constraint_pose(0), constraint_pose(2));
//
//	for (int i = 0; i < nQ; i++)
//		printf("%f\t%f\n", qpos[i], qvel[i]);

}

static void StateToCassieOutputs(double* qpos, double* qvel, cassie_out_t* sensors)
{
	sensors->leftLeg.hipPitchDrive.position = qpos[0];
	sensors->leftLeg.hipPitchDrive.velocity = qvel[0];

	sensors->leftLeg.kneeDrive.position = qpos[1];
	sensors->leftLeg.kneeDrive.velocity = qvel[1];

	sensors->leftLeg.tarsusJoint.position = qpos[2];
	sensors->leftLeg.tarsusJoint.velocity = qvel[2];

	sensors->leftLeg.footJoint.position = qpos[3];
	sensors->leftLeg.footJoint.velocity = qvel[3];
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
