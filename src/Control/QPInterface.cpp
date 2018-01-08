/*
 * QPInterface.cpp
 *
 *  Created on: Jan 4, 2018
 *      Author: tapgar
 */

#include "QPInterface.h"

QP_Interface::QP_Interface() {
	// TODO Auto-generated constructor stub
	qp = new qpOASES::SQProblem((nQ+nU+nCON*(2*(DOF-1)+1)), nQ+nCON*4*(DOF-1));

	qpOptions.setToMPC( );
	qpOptions.printLevel = qpOASES::PL_NONE;
	qp->setOptions( qpOptions );
}

QP_Interface::~QP_Interface() {
	// TODO Auto-generated destructor stub
}

bool QP_Interface::Solve(double* H_, double* g_, double* CE_, double* ce_,
			double* C_, double* cilb_, double* ciub_,
			double* lb_, double* ub_, double* x_res)
{
	static bool bFirstCall = true;

	qpOASES::int_t nWSR = 100;
	qpOASES::returnValue eRet = qpOASES::SUCCESSFUL_RETURN;

	for (int i = 0; i < (nQ+nU+nCON*(2*(DOF-1)+1))*(nQ+nU+nCON*(2*(DOF-1)+1)); i++)
		H[i] = H_[i];
	for (int i = 0; i < nQ+nU+nCON*(2*(DOF-1)+1); i++)
		g[i] = g_[i];
	for (int i = 0; i < nQ*(nQ+nU+nCON*(2*(DOF-1)+1)); i++)
		A[i] = CE_[i];
	for (int i = 0; i < nQ; i++)
		lbA[i] = ubA[i] = ce_[i];
	for (int i = nQ*(nQ+nU+nCON*(2*(DOF-1)+1)); i < (nQ+nCON*4*(DOF-1))*(nQ+nU+nCON*(2*(DOF-1)+1)); i++)
		A[i] = C_[i - nQ*(nQ+nU+nCON*(2*(DOF-1)+1))];
	for (int i = nQ; i < (nQ+nCON*4*(DOF-1)); i++)
	{
		lbA[i] = cilb_[i-nQ];
		ubA[i] = ciub_[i-nQ];
	}
	for (int i = 0; i < nQ+nU+nCON*(2*(DOF-1)+1); i++)
		lb[i] = lb_[i];
	for (int i = 0; i < nQ+nU+nCON*(2*(DOF-1)+1); i++)
		ub[i] = ub_[i];

	if (bFirstCall)
		eRet = qp->init(H, g, A, lb, ub, lbA, ubA, nWSR, 0);
	else
		eRet = qp->hotstart(H, g, A, lb, ub, lbA, ubA, nWSR, 0);

	qp->getPrimalSolution(x);
	for (int i = 0; i < (nQ+nU+nCON*(2*(DOF-1)+1)); i++)
		x_res[i] = (double)x[i];

	if (eRet == qpOASES::SUCCESSFUL_RETURN)
	{
		bFirstCall = false;
		return true;
	}

	return false;
}
