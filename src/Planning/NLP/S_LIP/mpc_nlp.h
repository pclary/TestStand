// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: hs071_nlp.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __MPC_NLP_HPP__
#define __MPC_NLP_HPP__

#include "IpTNLP.hpp"
#include <string.h>
#include "math.h"
#include "Common_Structs.h"
#include "CommandInterface.h"
#include "MPCOptions.h"
#include <vector>

namespace Ipopt {
class MPC_NLP : public TNLP
{
public:
	/** default constructor */
	MPC_NLP();

	/** default destructor */
	virtual ~MPC_NLP();

	/**@name Overloaded from TNLP */
	//@{
	/** Method to return some info about the nlp */
	virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
			Ipopt::Index& nnz_h_lag, IndexStyleEnum& Index_style);

	/** Method to return the bounds for my problem */
	virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
			Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

	/** Method to return the starting point for the algorithm */
	virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
			bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
			Ipopt::Index m, bool init_lambda,
			Ipopt::Number* lambda);

	/** Method to return the objective value */
	virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);

	/** Method to return the gradient of the objective */
	virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);

	/** Method to return the constraint residuals */
	virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g);

	/** Method to return:
	 *   1) The structure of the jacobian (if "values" is NULL)
	 *   2) The values of the jacobian (if "values" is not NULL)
	 */
	virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
			Ipopt::Number* values);

	/** Method to return:
	 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
	 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
	 */
	virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
			bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
			Ipopt::Index* jCol, Ipopt::Number* values);

	//@}

	/** @name Solution Methods */
	//@{
	/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
	virtual void finalize_solution(Ipopt::SolverReturn status,
			Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
			Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
			Ipopt::Number obj_value,
			const Ipopt::IpoptData* ip_data,
			Ipopt::IpoptCalculatedQuantities* ip_cq);
	//@}

	void Setup(MPC_OPTIONS* Opt, bool bWarmStart);

	void SetWarmStart(bool bWarmStart) { m_bWarmStart = bWarmStart; };
	void SetObjective(bool bObjective) { m_bUseObjective = bObjective; };

	void GetSolution(ROM_Policy_Struct* targ_traj, double dt);
	void GetParams(CommandInterface::policy_params_t* params) { opt->GetParams(params); }
private:
	/**@name Methods to block default compiler methods.
	 * The compiler automatically generates the following three methods.
	 *  Since the default compiler implementation is generally not what
	 *  you want (for all but the most simple classes), we usually
	 *  put the declarations of these methods in the private section
	 *  and never implement them. This prevents the compiler from
	 *  implementing an incorrect "default" behavior without us
	 *  knowing. (See Scott Meyers book, "Effective C++")
	 *
	 */
	//@{
	//  MPC_NLP();
	MPC_NLP(const MPC_NLP&);
	MPC_NLP& operator=(const MPC_NLP&);

	void get_hessian_structure(Ipopt::Index n, Ipopt::Index* iRow, Ipopt::Index* jCol);
	void get_jacobian_structure(Ipopt::Index n, Ipopt::Index m, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Index nele_jac);

	MPC_OPTIONS* opt;

	bool m_bWarmStart;
	bool m_bUseObjective;

	// Ipopt interprets any Ipopt::Number greater than nlp_upper_bound_inf as
	// infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
	// is 1e19 and can be changed through ipopt options.
	static constexpr Ipopt::Number NLP_INF = 2.0e19;

	static constexpr double m_dAlphaW = 1e-1;
	static constexpr double m_dJerkW = 1e-6;

	Ipopt::Number x_last[MAX_IPOPT_VARS];

	//@}
};
};

#endif
