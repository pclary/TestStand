/*
 * MPC.cpp
 *
 *  Created on: Nov 27, 2017
 *      Author: tapgar
 */

#include "MPC.h"

#include <iostream>
#include <sys/time.h>


using namespace Ipopt;

MPC::MPC() {
	// TODO Auto-generated constructor stub
	nlp = new MPC_NLP();
}

MPC::~MPC() {
	// TODO Auto-generated destructor stub
}

int MPC::Init() {
	app = IpoptApplicationFactory();
	app->RethrowNonIpoptException(true);

	// Change some options
	// Note: The following choices are only examples, they might not be
	//       suitable for your optimization problem.
	app->Options()->SetNumericValue("tol", 1e-4);
	app->Options()->SetNumericValue("max_cpu_time", 0.1);
	app->Options()->SetIntegerValue("print_level", 0);
	app->Options()->SetIntegerValue("file_print_level", 0);
	app->Options()->SetStringValue("mu_strategy", "adaptive");
	app->Options()->SetStringValue("output_file", "ipopt.out");
	app->Options()->SetStringValue("linear_solver", "ma27");
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");
//	app->Options()->SetStringValue("derivative_test", "first-order");
	ApplicationReturnStatus status;
	status = app->Initialize();
	if (status != Solve_Succeeded) {
		std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
	}
	return (int) status;
}

bool MPC::Run() {
	ApplicationReturnStatus status;
	// Ask Ipopt to solve the problem
	timespec ts, tf;
	clock_gettime(CLOCK_REALTIME, &ts);
	nlp->SetObjective(false);
	status = app->OptimizeTNLP(nlp);
//	if (status == Solve_Succeeded) {
//		//run again with objective
//		nlp->SetWarmStart(true);
//		nlp->SetObjective(true);
//		status = app->OptimizeTNLP(nlp);
//	}
	clock_gettime(CLOCK_REALTIME, &tf);

	m_nRunTime_ms = int(double(diff(ts,tf).tv_nsec)/1e6);
	std::cout<<"Runtime:"<<m_nRunTime_ms<<" ms"<<std::endl;


	if (status == Solve_Succeeded) {
		std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
		m_bSuccess = true;
	}
	else {
		m_bSuccess = false;
		std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
	}


	// As the SmartPtrs go out of scope, the reference count
	// will be decremented and the objects will automatically
	// be deleted.

	return m_bSuccess;
}
