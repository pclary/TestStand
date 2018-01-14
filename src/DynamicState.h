/*
 * DynamicState.h
 *
 *  Created on: Jan 11, 2018
 *      Author: tapgar
 */

#ifndef DYNAMICSTATE_H_
#define DYNAMICSTATE_H_

#include "DynamicModel.h"
#include <Eigen/Dense>

typedef struct {
	Eigen::MatrixXd M; //Mass matrix
	Eigen::VectorXd bias; //coriolis, grav, spring
	Eigen::MatrixXd Bt; //B transpose
	Eigen::MatrixXd Jc; //contact jacobian
	Eigen::MatrixXd Jeq; //constrian jacobian
	Eigen::VectorXd JeqdotQdot; //accel difference at constraint
} DynamicMatrices;

class DynamicState {
public:
	DynamicState();
	virtual ~DynamicState();

	void Init(int* conIds);

	void UpdateDynamicState(DynamicModel* dyn);

	DynamicMatrices m_ConfigStiff;
	DynamicMatrices m_ConfigSpring;

	void GetStiffConfig(Eigen::MatrixXd* M, Eigen::VectorXd* bias, Eigen::MatrixXd* Bt,
			Eigen::MatrixXd* Jc, Eigen::MatrixXd* Jeq, Eigen::VectorXd* JeqdotQdot)
	{
		*M = m_ConfigStiff.M;
		*bias = m_ConfigStiff.bias;
		*Bt = m_ConfigStiff.Bt;
		*Jc = m_ConfigStiff.Jc;
		*Jeq = m_ConfigStiff.Jeq;
		*JeqdotQdot = m_ConfigStiff.JeqdotQdot;
	}

	void GetSpringConfig(Eigen::MatrixXd* M, Eigen::VectorXd* bias, Eigen::MatrixXd* Bt,
			Eigen::MatrixXd* Jc, Eigen::MatrixXd* Jeq, Eigen::VectorXd* JeqdotQdot)
	{
		*M = m_ConfigSpring.M;
		*bias = m_ConfigSpring.bias;
		*Bt = m_ConfigSpring.Bt;
		*Jc = m_ConfigSpring.Jc;
		*Jeq = m_ConfigSpring.Jeq;
		*JeqdotQdot = m_ConfigSpring.JeqdotQdot;
	}

private:
	int contactSiteIds[nCON];
};

#endif /* DYNAMICSTATE_H_ */
