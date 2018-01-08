/*
 * QPInterface.h
 *
 *  Created on: Jan 4, 2018
 *      Author: tapgar
 */

#ifndef QPINTERFACE_H_
#define QPINTERFACE_H_

#include <qpOASES.hpp>
#include "SharedRobotDefinitions.h"


class QP_Interface {
public:
	QP_Interface();
	virtual ~QP_Interface();


	bool Solve(double* H, double* g, double* CE, double* ce,
			double* C, double* cilb, double* ciub,
			double* lb, double* ub, double* x_res);

private:
	qpOASES::SQProblem* qp; //for QPs where H or A may change

	qpOASES::real_t H[(nQ+nU+nCON*(2*(DOF-1)+1))*(nQ+nU+nCON*(2*(DOF-1)+1))];
	qpOASES::real_t g[nQ+nU+nCON*(2*(DOF-1)+1)];
	qpOASES::real_t A[(nQ+nCON*4*(DOF-1))*(nQ+nU+nCON*(2*(DOF-1)+1))];
	qpOASES::real_t lb[nQ+nU+nCON*(2*(DOF-1)+1)];
	qpOASES::real_t ub[nQ+nU+nCON*(2*(DOF-1)+1)];
	qpOASES::real_t lbA[nQ+nCON*4*(DOF-1)];
	qpOASES::real_t ubA[nQ+nCON*4*(DOF-1)];
	qpOASES::real_t x[(nQ+nU+nCON*(2*(DOF-1)+1))];

	qpOASES::Options qpOptions;


};

#endif /* QPINTERFACE_H_ */
