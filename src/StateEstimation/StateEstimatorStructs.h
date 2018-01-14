/*
 * StateEstimatorStructs.h
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#ifndef STATEESTIMATORSTRUCTS_H_
#define STATEESTIMATORSTRUCTS_H_

#include <Eigen/Dense>

typedef struct {
	double qpos[nQ]; //joint pos
	double qvel[nQ]; //joint vel
	double force[nCON*DOF]; //estimated force at contact point
	double contact[nCON]; //contact 0->1
	bool contact_state[nCON]; //whether the point is in contact
	double quat[4]; //duplicate of euclidian in joint pos
} state_t;

typedef struct {
	double zqpos[nENC];
	double zqvel[nENC];
	double ztorque[nU];
	Eigen::Vector3d zmag;
	Eigen::Vector3d zimu;
	Eigen::Vector3d zacc;
	Eigen::Matrix<double, 4, 1> zquat;
} measurement_t;
#endif /* STATEESTIMATORSTRUCTS_H_ */
