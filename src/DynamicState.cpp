/*
 * DynamicState.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: tapgar
 */

#include "DynamicState.h"

using namespace Eigen;
using namespace std;

DynamicState::DynamicState() {

}


DynamicState::~DynamicState() {
	// TODO Auto-generated destructor stub
}

void DynamicState::Init(int* conIds) {
	// TODO Auto-generated constructor stub


	m_ConfigStiff.M = MatrixXd::Zero(nQstiff,nQstiff);
	m_ConfigStiff.bias = VectorXd::Zero(nQstiff);
	m_ConfigStiff.Bt = MatrixXd::Zero(nQstiff, nU);
	m_ConfigStiff.Jc = MatrixXd::Zero(nCON*DOF, nQstiff);
	m_ConfigStiff.Jeq = MatrixXd::Zero(nEQ, nQstiff);
	m_ConfigStiff.JeqdotQdot = VectorXd::Zero(nEQ);

	m_ConfigSpring.M = MatrixXd::Zero(nQ,nQ);
	m_ConfigSpring.bias = VectorXd::Zero(nQ);
	m_ConfigSpring.Bt = MatrixXd::Zero(nQ, nU);
	m_ConfigSpring.Jc = MatrixXd::Zero(nCON*DOF, nQ);
	m_ConfigSpring.Jeq = MatrixXd::Zero(nEQ, nQ);
	m_ConfigSpring.JeqdotQdot = VectorXd::Zero(nEQ);

	for (int i = 0; i < nCON; i++)
		contactSiteIds[i] = conIds[i];


}
void DynamicState::UpdateDynamicState(DynamicModel* dyn)
{
	dyn->GetMassMatrix(&m_ConfigSpring.M);

	dyn->GetBiasForce(&m_ConfigSpring.bias);
	VectorXd passive = VectorXd::Zero(nQ);
	dyn->GetPassiveForce(&passive);
	m_ConfigSpring.bias -= passive;

	dyn->GetSelectorMatrix(&m_ConfigSpring.Bt);

	//Contact jacobians
	for (int i = 0; i < nCON; i++)
	{
		MatrixXd jc = MatrixXd::Zero(DOF, nQ);
		dyn->GetSiteJacobian(&jc, contactSiteIds[i]);
		m_ConfigSpring.Jc.block<DOF,nQ>(i*DOF,0) = jc;
	}

	//Constraint jacobians
	dyn->GetConstraintJacobian(&m_ConfigSpring.Jeq);
	dyn->GetConstraintAccel(&m_ConfigSpring.JeqdotQdot);

	int idx_i = 0;
	for (int i = 0; i < nQ; i++)
	{
		if (dyn->IsSpringJoint(i))
			continue;

		int idx_j = 0;
		for (int j = 0; j < nQ; j++)
		{
			if (dyn->IsSpringJoint(j))
				continue;
			m_ConfigStiff.M(idx_i,idx_j) = m_ConfigSpring.M(i,j);
			idx_j++;
		}

		m_ConfigStiff.bias(idx_i) = m_ConfigSpring.bias(i);
		m_ConfigStiff.Bt.row(idx_i) = m_ConfigSpring.Bt.row(i);
		m_ConfigStiff.Jc.col(idx_i) = m_ConfigSpring.Jc.col(i);
		m_ConfigStiff.Jeq.col(idx_i) = m_ConfigSpring.Jeq.col(i);

		idx_i++;
	}
	m_ConfigStiff.JeqdotQdot = m_ConfigSpring.JeqdotQdot;
}
