/*
 * FloatingBaseEstimator.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#include "FloatingBaseEstimator.h"

FloatingBaseEstimator::FloatingBaseEstimator() {
	// TODO Auto-generated constructor stub

}

FloatingBaseEstimator::~FloatingBaseEstimator() {
	// TODO Auto-generated destructor stub
}

void FloatingBaseEstimator::Init(state_t* s) {

	Q = Eigen::Matrix<double, 7, 7>::Zero();
	Q(0,0) = Q(1,1) = Q(2,2) = 1e0; //we know the update eq for w_dot is wrong... set variance high
	Q(3,3) = Q(4,4) = Q(5,5) = Q(6,6) = 1e-2; //small process noise for quaternion

	R = Eigen::Matrix<double, 7, 7>::Zero();
	R(0,0) = R(1,1) = R(2,2) = 1e-4; //these are the rot vels from the QP estimator
	R(3,3) = R(4,4) = R(5,5) = 1e-4; //quaternions in the x,y plane
	R(6,6) = 1e-2; //quaternion about z... compass indoors can cause this to jump

	P = 10*Q; //initialize P with high variance

	for (int i = 0; i < 3; i++)
		x(i,0) = s->qvel[3+i];
	for (int i = 0; i < 4; i++)
		x(i+3,0) = s->quat[i];

	I = Eigen::Matrix<double, 7, 7>::Zero();
	for (int i = 0; i < 7; i++)
		I(i,i) = 1.0;
	H = I;
}

void FloatingBaseEstimator::Update(DynamicModel* dyn, state_t* s, cassie_out_t sensors, double dt)
{
	EKF_Update(s, sensors, dt);

	for (int i = 0; i < 3; i++)
		s->qpos[i] += s->qvel[i]*dt;

	quaternionToEuler(s->quat, &(s->qpos[3]));


}

void FloatingBaseEstimator::EKF_Update(state_t* s, cassie_out_t sensors)
{
	//Model Update
	Eigen::Matrix<double, 4, 4> quat_update;
	quat_update << 0.0, -x(0,0)/2, -x(1,0)/2, -x(2,0)/2,
				x(0,0)/2, 0.0, x(2,0)/2, -x(1,0)/2,
				x(1,0)/2, -x(2,0)/2, 0.0, x(0,0)/2,
				x(2,0)/2, x(1,0)/2, -x(0,0)/2, 0.0;

	x.block<4,1>(3,0) = x.block<4,1>(3,0) + quat_update*(x.block<4,1>(3,0))*dt;

	A = Eigen::Matrix<double, 7, 7>::Zero();
	A.block<4,4>(3,3) = quat_update;
	A.block<4,3>(3,0) << -x(4,0)/2, -x(5,0)/2, -x(6,0)/2,
						x(3,0)/2, -x(6,0)/2, x(5,0)/2,
						x(6,0)/2, x(3,0)/2, -x(4,0)/2,
						-x(5,0)/2, x(4,0)/2, x(3,0)/2;
	A *= dt;
	A += I;

	P = A*P*A.transpose() + Q;

	//Measurement Update
	S = (H*P*H.transpose() + R).llt().solve(I);
	K = S*(P*H.transpose());

	for (int i = 0; i < 3; i++)
		z(i,0) = s->qvel[3+i];
	for (int i = 0; i < 4; i++)
		z(i+3,0) = sensors.pelvis.vectorNav.orientation[i];

	x = x + K*(z - H*x);
	P = (I-K*H)*P;

	double mag = sqrt(pow(x(3,0),2.0) + pow(x(4,0),2.0) + pow(x(5,0),2.0) + pow(x(6,0),2.0));
	for (int i = 0; i < 4; i++)
		x(i+3,0) /= mag;

	for (int i = 0; i < 4; i++)
		s->quat[i] = x(i+3,0);

}
