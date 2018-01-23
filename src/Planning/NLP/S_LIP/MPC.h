/*
 * MPC.h
 *
 *  Created on: Nov 27, 2017
 *      Author: tapgar
 */

#ifndef MPC_H_
#define MPC_H_

#include "IpIpoptApplication.hpp"
#include "mpc_nlp.h"
#include <vector>
#include "Common_Structs.h"
#include "CommandInterface.h"
#include "HelperFunctions.h"

namespace Ipopt {
class MPC {
public:
	MPC();
	virtual ~MPC();

	int Init();
	bool Run();

	void SetProblem(MPC_OPTIONS* opts, bool bWarmStart)
	{
		opt = opts;
		nlp->Setup(opts, bWarmStart);
	}

	void GetSolution(ROM_Policy_Struct* targ_traj, double dt) { nlp->GetSolution(targ_traj, dt); };
	void GetParams(CommandInterface::policy_params_t* policy_params) { nlp->GetParams(policy_params); };


private:

	int m_nRunTime_ms;
	bool m_bSuccess;

	MPC_NLP* nlp;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> app;

	MPC_OPTIONS* opt;
};
};

#endif /* MPC_H_ */
