/*
 * BodyStateEstimator.h
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#ifndef BODYSTATEESTIMATOR_H_
#define BODYSTATEESTIMATOR_H_

#include <Eigen/Dense>

class BodyStateEstimator {
public:
	BodyStateEstimator();
	virtual ~BodyStateEstimator();

	void Update(DynamicModel* dyn, DynamicState* dyn_state, state_t* s, measurement_t z);

	void Init();

private:
	qpOASES::SQProblem* qp; //for QPs where H or A may change
	qpOASES::Options qpOptions;

	void UpdateMeasurementError(DynamicModel* dyn, state_t* s, measurement_t z);
	bool SolveQP();

	Eigen::MatrixXd M; //Mass matrix
	Eigen::VectorXd bias; //coriolis, grav, spring
	Eigen::MatrixXd Bt; //B transpose
	Eigen::MatrixXd Jc; //contact jacobian
	Eigen::MatrixXd Jeq; //constrian jacobian
	Eigen::VectorXd JeqdotQdot;

	Eigen::Matrix<double, nQ, nQ> Nc; //constraint projector
	Eigen::Matrix<double, nQ, 1> gamma; //

	Eigen::Matrix<double, nQ*2 + nCON*DOF, 1> x;//open vars

	//measurement-model error Ax=b
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
	Eigen::Matrix<double, nENC + nQ + 6 + 3*nCON, nENC + nQ + 6 + 3*nCON> W; //QP weighting matrix

	//QP matrices
	// z = 0.5*x'Hx + g'x;
	Eigen::Matrix<double, nQ*2 + nCON*DOF, nQ*2 + nCON*DOF, Eigen::RowMajor> H;//hessian
	Eigen::Matrix<double, 1, nQ*2 + nCON*DOF, Eigen::RowMajor> gt; //jacobian
	//Equality and Inequality Constraints
	Eigen::Matrix<double, nQ + nEQ*DOF, nQ*2 + nCON*DOF, Eigen::RowMajor> C;
	Eigen::Matrix<double, nQ + nEQ*DOF, 1> clb;
	Eigen::Matrix<double, nQ + nEQ*DOF, 1> cub;
	//Upper/Lower bounds
	Eigen::Matrix<double, nQ*2 + nCON*DOF, 1> ub;
	Eigen::Matrix<double, nQ*2 + nCON*DOF, 1> lb;

	unsigned int enc_idx[nENC];

	static constexpr double m_dWeightEncoder = 5e0; //units in joint vel
	static constexpr double m_dWeightOmega = 5e0; //units in rot velocity
	static constexpr double m_dWeightLinVel = 1e-1; //units in lin velocity
	static constexpr double m_dWeightInContact = 1e1; //units in velocity
	static constexpr double m_dWeightModel = 3e-3; //units generalized force (much higher)

};

#endif /* BODYSTATEESTIMATOR_H_ */
