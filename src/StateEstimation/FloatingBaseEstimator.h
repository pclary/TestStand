/*
 * FloatingBaseEstimator.h
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#ifndef FLOATINGBASEESTIMATOR_H_
#define FLOATINGBASEESTIMATOR_H_

#include <Eigen/Dense>

class FloatingBaseEstimator {
public:
	FloatingBaseEstimator();
	virtual ~FloatingBaseEstimator();

	void Update(DynamicModel* dyn, state_t* s, measurement_t z);

	void Init(state_t* s);

private:

	Eigen::Matrix<double, 7, 7> A;
	Eigen::Matrix<double, 7, 7> P;
	Eigen::Matrix<double, 7, 7> Q;
	Eigen::Matrix<double, 7, 7> H;
	Eigen::Matrix<double, 7, 7> R;
	Eigen::Matrix<double, 7, 7> K;
	Eigen::Matrix<double, 7, 7> I;
	Eigen::Matrix<double, 7, 1> x;
	Eigen::Matrix<double, 7, 1> z;

	void EKF_Update(state_t* s, cassie_out_t sensors);

};

#endif /* FLOATINGBASEESTIMATOR_H_ */
