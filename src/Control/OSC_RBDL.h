/*
 * OSC_RBDL.h
 *
 *  Created on: Nov 8, 2017
 *      Author: tapgar
 */

#ifndef OSC_RBDL_H_
#define OSC_RBDL_H_

#include "QPInterface.h" //required for QP
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include "Common_Structs.h"
#include "HelperFunctions.h"
#include "SharedRobotDefinitions.h"
#include "DynamicModel.h"


class OSC_RBDL {

public:
	OSC_RBDL(int* conIds, int* targIds);

	void RunPTSC(DynamicModel* dyn, Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive, bool* bContact, Eigen::Matrix<double, nU, 1>* u);

	int AddQDDIdx(int idx) {

		if (m_nAssignedIndices == QDD_TARGETS || idx < 0 || idx >= nQ)
			return 1;
		qdd_targ_idx[m_nAssignedIndices] = idx;
		m_nAssignedIndices++;
		return 0;
	}

	void InitMatrices(DynamicModel* dyn);


private:

	QP_Interface m_qp;

	void GetMotorLimits(DynamicModel* dyn, Eigen::MatrixXd* mlb, Eigen::MatrixXd* mub);

	void UpdateMatricesAtState(DynamicModel* dyn);

	Eigen::MatrixXd M; //Mass matrix
	Eigen::VectorXd bias; //coriolis, grav, spring
	Eigen::MatrixXd Bt; //B transpose
	Eigen::MatrixXd Jc; //contact jacobian
	Eigen::MatrixXd A; //jacobian relating target
	Eigen::VectorXd AdotQdot; //time deriv above
	Eigen::MatrixXd Jeq; //constrian jacobian
	Eigen::VectorXd JeqdotQdot;

	Eigen::Matrix<double, nQ, nQ> Nc; //constraint projector
	Eigen::Matrix<double, nQ, 1> gamma; //

	Eigen::Matrix<double, nCON*DOF, nCON*(2*(DOF-1)+1)> V; //friction cone

	Eigen::Matrix<double, nQ+nU+nCON*(2*(DOF-1)+1), 1> x;//open vars

	Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS> W; //QP weighting matrix

	//QP matrices
	// z = 0.5*x'Hx + g'x;
	Eigen::Matrix<double, nQ+nU+nCON*(2*(DOF-1)+1), nQ+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> H;//hessian
	Eigen::Matrix<double, 1, nQ+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> gt; //jacobian
	//Equality constraints
	//Ax = b
	Eigen::Matrix<double, nQ, nQ+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> CE;
	Eigen::Matrix<double, nQ, 1> ce;
	//Inequality constraints
	//d <= Cx <= f
	Eigen::Matrix<double, nCON*4*(DOF-1), nQ+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> C;
	Eigen::Matrix<double, nCON*4*(DOF-1), 1> cilb;
	Eigen::Matrix<double, nCON*4*(DOF-1), 1> ciub;
	//Upper/Lower bounds
	Eigen::Matrix<double, nQ+nU+nCON*(2*(DOF-1)+1), 1> ub;
	Eigen::Matrix<double, nQ+nU+nCON*(2*(DOF-1)+1), 1> lb;

	int qdd_targ_idx[QDD_TARGETS];
	int m_nAssignedIndices;

	static constexpr double m_dWeight_Stance = 1e1;
	static constexpr double m_dWeight_Swing = 1e0;//0.2;
	static constexpr double m_dWeight_COM = 5.0;
	static constexpr double m_dWeight_Rest = 1e-1;//0.000002;
	static constexpr double m_dWeight_Tau = 1e-6;//0.000002;
	static constexpr double m_dWeight_Fx = 1e-4;
	static constexpr double m_dWeight_Fz = 1e-4;

	int contactSiteIds[nCON];
	int targetSiteIds[XDD_TARGETS];
	int* contactBodyIds;

};


#endif /* OSC_RBDL_H_ */
