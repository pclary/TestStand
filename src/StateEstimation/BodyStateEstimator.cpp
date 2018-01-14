/*
 * BodyStateEstimator.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#include "BodyStateEstimator.h"

BodyStateEstimator::BodyStateEstimator() {
	// TODO Auto-generated constructor stub
	qp = new qpOASES::SQProblem(nQ*2 + nCON*DOF, nQ + nCON + nEQ*DOF);

	qpOptions.setToMPC( );
	qpOptions.printLevel = qpOASES::PL_NONE;
	qp->setOptions( qpOptions );
}

BodyStateEstimator::~BodyStateEstimator() {
	// TODO Auto-generated destructor stub
}

void BodyStateEstimator::Init()
{
	M = MatrixXd::Zero(nQ,nQ);
	bias = VectorXd::Zero(nQ);
	Bt = MatrixXd::Zero(nQ, nU);
	Jc = MatrixXd::Zero(nCON*DOF, nQ);
	A = MatrixXd::Zero(nENC + nQ + 6 + 3*nCON, nQ*2 + nCON*DOF);
	b = VectorXd::Zero(nENC + nQ + 6 + 3*nCON);
	Jeq = MatrixXd::Zero(nEQ, nQ);
	JeqdotQdot = VectorXd::Zero(nEQ);
	x = Matrix<double, nQ*2 + nCON*DOF, 1>::Zero();

	H = Matrix<double, nQ*2 + nCON*DOF, nQ*2 + nCON*DOF>::Zero();
	gt = Matrix<double, 1, nQ*2 + nCON*DOF>::Zero();
	C = Matrix<double, nQ + nEQ*DOF, nQ*2 + nCON*DOF>::Zero();
	clb = Matrix<double, nQ + nEQ*DOF, 1>::Zero();
	cub = Matrix<double, nQ + nEQ*DOF, 1>::Zero();

	//set up lb and ub
	lb = -Matrix<double, nQ*2 + nCON*DOF, 1>::Ones()*numeric_limits<double>::max();
	ub = Matrix<double, nQ*2 + nCON*DOF, 1>::Ones()*numeric_limits<double>::max();
	for (int i = 0; i < nCON; i++)
		lb(nQ*2+i*DOF+2,0) = 0.0;

	//initialize weight matrix
	W = Matrix<double, nENC + nQ + 6 + 3*nCON, nENC + nQ + 6 + 3*nCON>::Zero();
	for (int i = 0; i < nENC; i++)
		W(i,i) = m_dWeightEncoder;
	for (int i = nENC; i < nENC+nQ; i++)
		W(i,i) = m_dWeightModel;
	for (int i = nENC+nQ; i < nENC+nQ+3; i++)
		W(i,i) = m_dWeightOmega;
	for (int i = nENC+nQ+3; i < nENC+nQ+6; i++)
		W(i,i) = m_dWeightLinVel;
	for (int i = nENC+nQ+6; i < nENC+nQ+6+3*nCON; i++)
		W(i,i) = m_dWeightInContact;

	for (int i = 0; i < nENC/2; i++)
	{
		enc_idx[i] = i+6;
		enc_idx[i+nENC/2] = i+15;
	}

	for (int i = 0; i < nENC; i++)
	{
		A(i,enc_idx[i]) = 1.0;
		b(i,0) = z.zqvel[i];
	}

	for (int i = 0; i < nQ; i++)
	{
		A(i+nENC,i+nQ) = 1.0;
		b(i+nENC,0) = 0.0;
	}

	for (int i = 0; i < 3; i++)
	{
		A(i+nENC+nQ,i+3) = 1.0;
	}


}

void BodyStateEstimator::Update(DynamicModel* dyn, DynamicState* dyn_state, state_t* s, measurement_t z)
{
	dyn_state->GetSpringConfig(&M, &bias, &Bt, &Jc, &Jeq, &JeqdotQdot);

	//populate A and b
	UpdateMeasurementError(dyn, s, z);

	Matrix<double, nENC + nQ + 6 + 3*nCON, nENC + nQ + 6 + 3*nCON> tempW = W;

	//adjust limits based on contact forces
	for (int i = 0; i < nCON; i++)
	{
		if (s->contact_state[i])
		{
			for (int j = 0; j < DOF; j++)
			{
				ub(nQ*2+i*DOF+j,0) = numeric_limits<double>::max();
				lb(nQ*2+i*DOF+j,0) = -1.0*numeric_limits<double>::max();
				if (j == DOF-1)
					lb(nQ*2+i*DOF+j,0) = 0.0;
				tempW(nENC+nQ+6+i*DOF+j,nENC+nQ+6+i*DOF+j) = m_dWeightInContact;
			}
		}
		else
		{
			for (int j = 0; j < DOF; j++)
			{
				ub(nQ*2+i*DOF+j,0) = 0.0;
				lb(nQ*2+i*DOF+j,0) = 0.0;
				tempW(nENC+nQ+6+i*DOF+j,nENC+nQ+6+i*DOF+j) = 0.0;
			}
		}
	}

	Matrix<double, nQ, nQ> I = Matrix<double, nQ, nQ>::Identity();
	Matrix<double, nQ, nQ> Hinv = M.inverse();
	Matrix<double, nEQ, nEQ> JHinvJ = pseudoinverse(Jeq*Hinv*Jeq.transpose());

	Nc = I - Jeq.transpose()*JHinvJ*Jeq*Hinv;
	gamma = Jeq.transpose()*JHinvJ*JeqdotQdot;

	C.block<nQ,nQ>(0,0) = (1.0/m_dDeltaTime_s)*M;
	C.block<nQ,nQ>(0,nQ) = -1.0*I;
	C.block<nQ,nCON*DOF>(0,2*nQ) = -Nc*Jc.transpose();
	C.block<nEQ*DOF,nQ>(nQ,0) = Jeq;

	//update measurement error

	for (int i = 0; i < nENC; i++)
		b(i,0) = (z.zqpos[i] - s->qpos[enc_idx[i]])/m_dDeltaTime_s;

	//update rotational velocity
	for (int i = 0; i < 3; i++)
		b(i+nENC+nQ,0) = z.zimu(i);

	//update accelerometer
	Eigen::MatrixXd J_imu = Eigen::MatrixXd::Zero(3, nQ);
	dyn->GetSiteJacobian(&J_imu, 0);

	A.block<3,nQ>(nENC+nQ+3,0) = J_imu;
	for (int i = 0; i < 3; i++)
		b(i+nENC+nQ+3,0) = s->qvel[i] + z.zacc[i]*m_dDeltaTime_s;

	//contact (vel error)... weightings set to zero if not in contact
	A.block<nQ,nCON*DOF>(nENC+nQ+6,nQ*2) = Nc*Jc.transpose();

	Matrix<double, nU, 1> u = Map<Matrix<double, nU, 1>>(z.ztorque);

	clb.block<nQ,1>(0,0) = cub.block<nQ,1>(0,0) = Bt*u -Nc*bias - gamma;

	H  = 2.0*A.transpose()*tempW*A;
	gt = -2.0*b.transpose()*tempW*A;

	SolveQP();

	for (int i = 0; i < nQ; i++)
		s->qvel[i] = x(i,0);
	for (int i = 0; i < nCON*DOF; i++)
		s->force[i] = x(i+2*nQ,0);

}

bool BodyStateEstimator::SolveQP()
{
	static bool bFirstCall = true;

	qpOASES::int_t nWSR = 100;
	qpOASES::returnValue eRet = qpOASES::SUCCESSFUL_RETURN;

	if (bFirstCall)
		eRet = qp->init(H.data(), g.data(), C.data(), lb.data(), ub.data(), clb.data(), cub.data(), nWSR, 0);
	else
		eRet = qp->hotstart(H.data(), g.data(), C.data(), lb.data(), ub.data(), clb.data(), cub.data(), nWSR, 0);

	qp->getPrimalSolution(x.data());

	if (eRet == qpOASES::SUCCESSFUL_RETURN)
	{
		bFirstCall = false;
		return true;
	}

	return false;
}
