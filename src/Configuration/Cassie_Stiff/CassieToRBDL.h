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
	double prev_base_pos[3];
	double prev_base_vel[3];
	for (int i = 0; i < 3; i++)
	{
		prev_base_pos[i] = qpos[i];
		prev_base_vel[i] = qvel[i];
	}
	qpos[0] = qpos[1] = qpos[2] = 0.0;
	qvel[0] = qvel[1] = qvel[2] = 0.0;
	RigidBodyDynamics::Math::Quaternion temp_quat;
	temp_quat(0) = temp_quat(1) = temp_quat(2) = 0.0;
	temp_quat(3) = 1.0;

	dyn->SetQuaternion(temp_quat, qpos);

	for (int i = 0; i < 3; i++)
		qvel[i+3] = sensors.pelvis.vectorNav.angularVelocity[i];

	qpos[6] = sensors.leftLeg.hipRollDrive.position;
	qvel[6] = sensors.leftLeg.hipRollDrive.velocity;
	qpos[7] = sensors.leftLeg.hipYawDrive.position;
	qvel[7] = sensors.leftLeg.hipYawDrive.velocity;
	qpos[8] = sensors.leftLeg.hipPitchDrive.position;
	qvel[8] = sensors.leftLeg.hipPitchDrive.velocity;
	qpos[9] = sensors.leftLeg.kneeDrive.position;
	qvel[9] = sensors.leftLeg.kneeDrive.velocity;
	qpos[10] = sensors.leftLeg.tarsusJoint.position;
	qvel[10] = sensors.leftLeg.tarsusJoint.velocity;
	qpos[11] = sensors.leftLeg.footJoint.position;
	qvel[11] = sensors.leftLeg.footJoint.velocity;

	qpos[13] = sensors.rightLeg.hipRollDrive.position;
	qvel[13] = sensors.rightLeg.hipRollDrive.velocity;
	qpos[14] = sensors.rightLeg.hipYawDrive.position;
	qvel[14] = sensors.rightLeg.hipYawDrive.velocity;
	qpos[15] = sensors.rightLeg.hipPitchDrive.position;
	qvel[15] = sensors.rightLeg.hipPitchDrive.velocity;
	qpos[16] = sensors.rightLeg.kneeDrive.position;
	qvel[16] = sensors.rightLeg.kneeDrive.velocity;
	qpos[17] = sensors.rightLeg.tarsusJoint.position;
	qvel[17] = sensors.rightLeg.tarsusJoint.velocity;
	qpos[18] = sensors.rightLeg.footJoint.position;
	qvel[18] = sensors.rightLeg.footJoint.velocity;

	qpos[12] = qpos[19] = 0.0;

	dyn->setPos(qpos);

	RigidBodyDynamics::Math::VectorNd conrod_angles = RigidBodyDynamics::Math::VectorNd::Zero(nEQ);
	dyn->GetConrodAngles(&conrod_angles);

	qpos[12] = conrod_angles(0);
	qpos[19] = conrod_angles(1);

	RigidBodyDynamics::Math::Quaternion quat = RigidBodyDynamics::Math::Quaternion(
			sensors.pelvis.vectorNav.orientation[1],
			sensors.pelvis.vectorNav.orientation[2],
			sensors.pelvis.vectorNav.orientation[3],
			sensors.pelvis.vectorNav.orientation[0]);

	dyn->SetQuaternion(quat, qpos);

	dyn->setPos(qpos);

	RigidBodyDynamics::Math::MatrixNd Jeq = RigidBodyDynamics::Math::MatrixNd::Zero(nEQ, nQ); //constraint jacobian
	dyn->GetConstraintJacobian(&Jeq);

	RigidBodyDynamics::Math::MatrixNd Jeq_ind = RigidBodyDynamics::Math::MatrixNd::Zero(nEQ, nQ-2);
	Jeq_ind.block<nEQ,12>(0,0) = Jeq.block<nEQ,12>(0,0); Jeq_ind.block<nEQ,6>(0,12) = Jeq.block<nEQ,6>(0,13);

	RigidBodyDynamics::Math::MatrixNd Jeq_dep = RigidBodyDynamics::Math::MatrixNd::Zero(nEQ, 2);
	Jeq_dep.block<nEQ,1>(0,0) = Jeq.block<nEQ,1>(0,12); Jeq_dep.block<nEQ,1>(0,1) = Jeq.block<nEQ,1>(0,19);

	RigidBodyDynamics::Math::MatrixNd gamma = RigidBodyDynamics::Math::MatrixNd::Zero(nQ, nQ-2);
	RigidBodyDynamics::Math::VectorNd qdot_ind = RigidBodyDynamics::Math::VectorNd::Zero(nQ-2);//ind vel

	int idx = 0;
	for (int i = 0; i < nQ; i++)
	{
		if (i == 12 || i == 19)
			continue;
		gamma(i,idx) = 1.0;
		qdot_ind(idx) = qvel[i];
		idx++;
	}
	RigidBodyDynamics::Math::MatrixNd temp = -pseudoinverse(Jeq_dep)*Jeq_ind;
	gamma.block<1,nQ-2>(12,0) = temp.block<1,nQ-2>(0,0);
	gamma.block<1,nQ-2>(19,0) = temp.block<1,nQ-2>(1,0);

	RigidBodyDynamics::Math::VectorNd qdot = gamma*qdot_ind;

	for (int i = 0; i < nQ; i++)
		qvel[i] = qdot(i);

	for (int i = 0; i < 3; i++)
	{
		qpos[i] = prev_base_pos[i];
		qvel[i] = prev_base_vel[i];
	}

	dyn->setState(qpos, qvel);

	static int foot_con_idx = -1;
	static Eigen::Vector3d world_point_pos;
	static Eigen::Vector3d world_point_vel;
	static bool bFirst = true;

	RigidBodyDynamics::Math::VectorNd x = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);
	RigidBodyDynamics::Math::VectorNd xd = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);

	int targids[] = {1,2,3,4,5};
	dyn->GetTargetPoints(&x, &xd, targids);

	Eigen::Vector3d point_pos, point_vel;
	int min_foot_idx = -1;
	if (x(5) < x(8) && x(5) < x(11) && x(5) < x(14))
	{
		min_foot_idx = 0;
		point_pos = x.block<3,1>(3,0);
		point_vel = xd.block<3,1>(3,0);
	}
	else if (x(8) < x(5) && x(8) < x(11) && x(8) < x(14))
	{
		min_foot_idx = 1;
		point_pos = x.block<3,1>(6,0);
		point_vel = xd.block<3,1>(6,0);
	}
	else if (x(11) < x(8) && x(11) < x(5) && x(11) < x(14))
	{
		min_foot_idx = 2;
		point_pos = x.block<3,1>(9,0);
		point_vel = xd.block<3,1>(9,0);
	}
	else
	{
		min_foot_idx = 3;
		point_pos = x.block<3,1>(12,0);
		point_vel = xd.block<3,1>(12,0);
	}

	if (min_foot_idx != foot_con_idx)
	{
		world_point_pos = point_pos;
		world_point_pos(2) = 0.0;
		world_point_vel = point_vel;
	}

	qpos[0] += world_point_pos(0) - point_pos(0);
	qvel[0] += world_point_vel(0) - point_vel(0);
	qpos[1] += world_point_pos(1) - point_pos(1);
	qvel[1] += world_point_vel(1) - point_vel(1);
	qpos[2] += world_point_pos(2) - point_pos(2);
	qvel[2] += world_point_vel(2) - point_vel(2);


	foot_con_idx = min_foot_idx;

//	qpos[0] = -(x(3)+x(6)+x(9)+x(12))/4.0;
//	qvel[0] = -(xd(3)+xd(6)+xd(9)+xd(12))/4.0;
//	qpos[1] = -(x(4)+x(7)+x(10)+x(13))/4.0;
//	qvel[1] = -(xd(4)+xd(7)+xd(10)+xd(13))/4.0;
//	qpos[2] = -(x(5)+x(8)+x(11)+x(14))/4.0;
//	qvel[2] = -(xd(5)+xd(8)+xd(11)+xd(14))/4.0;
//
//	bFirst = false;

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
	for (int i = 0; i < nU; i++)
		command->torque[i] = u[i];
}

static void CassieInputsToTorque(cassie_user_in_t command, double* u)
{
	for (int i = 0; i < nU; i++)
		u[i] = command.torque[i];
}


#endif /* CASSIETORBDL_H_ */
