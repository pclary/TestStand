// Copyright (C) 2005, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: hs071_nlp.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-16

#include "mpc_nlp.h"

#include <cassert>
#include <iostream>
#include <unistd.h>

using namespace Ipopt;

// constructor
MPC_NLP::MPC_NLP()
{}

//destructor
MPC_NLP::~MPC_NLP()
{}

void MPC_NLP::Setup(MPC_OPTIONS* Opt, bool bWarmStart)
{
	opt = Opt;

	//open variables
	opt->N = opt->num_phases*25;			//4 phases 25 open vars each

	//xyza+vel stitching constraints, xyza terminal
	opt->M = opt->Mc_idx = opt->num_phases*4 + (opt->num_phases)*4;
	//sum of force coefficients = 1
	opt->M += 2*opt->num_phases;
	opt->Mf_idx = opt->M;

	opt->foot_equalities.clear();
//	opt->foot_equalities.empty();
	//foot pos equal previous
	std::vector<Index> foot_zero(3);
	for (int i = 1; i < opt->num_phases; i++)
	{
		foot_zero[0] = i-1;
		foot_zero[1] = i;
		if (opt->phase[i].eType == Double)
		{
			foot_zero[2] = 0;
			opt->foot_equalities.push_back(foot_zero);
			foot_zero[2] = 1;
			opt->foot_equalities.push_back(foot_zero);
		}
		if (opt->phase[i].eType == SS_Left)
		{
			foot_zero[2] = 0;
			opt->foot_equalities.push_back(foot_zero);
		}
		if (opt->phase[i].eType == SS_Right)
		{
			foot_zero[2] = 1;
			opt->foot_equalities.push_back(foot_zero);
		}
	}
	opt->M += 3*opt->foot_equalities.size();
	opt->Mp_idx = opt->M;

	//spring reference equalities (and inequalities r_0 >= z(t))) r_T_end is taken care of by x constraints
	opt->M += (opt->num_phases-1) + opt->num_phases;
	opt->Mr_idx = opt->M;

	//lambda constraints between phases lT- = l0+
	opt->M += 4*(opt->num_phases-1);
	opt->Ml_idx = opt->M;

	//foot bound constraints (both feet within bounds at end of segment)
//	opt->M += 4*(opt->num_phases-1);
//	opt->Mp_nl_idx = opt->M;
	//printf("here 1");
	//TODO: nonlinear foot constraints
	opt->M += 4*opt->num_phases;
	opt->Mp_nl_idx = opt->M;

	for (int i = 0; i < opt->num_phases; i++)
		if (opt->phase[i].eType != Double)
			opt->M += 1;
	opt->Mp_a_idx = opt->M;


//	//foot pos within bounds - phase start/end for left and right and x and y
//	opt->M += opt->num_phases*2*2*2;
//	opt->Mp_idx = opt->M;

	//calculate auxilary variables
	opt->omega = sqrt(opt->stiffness/opt->mass);
	opt->g_omega_2 = opt->g/pow(opt->omega, 2.0);
	for (int i = 0; i < opt->num_phases; i++)
	{
		opt->cwT[i] = cos(opt->phase[i].T*opt->omega);
		opt->swT[i] = sin(opt->phase[i].T*opt->omega);
	}

	m_bWarmStart = bWarmStart;
}

// returns the size of the problem
bool MPC_NLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
		Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = opt->N;
	m = opt->M;

	// count nonzeros in jacobian
	nnz_jac_g = 0;
	//16 nonzeros per x,y,xdot,ydot
	//5 nonzeros per z,zdot
	//4 and 3 nonzeros for a and adot
	nnz_jac_g += (16+16+5+4)*(opt->num_phases-1) + (15+15+4+3) + (15+15+4+2) + (16+16+5+3)*(opt->num_phases-1);

//	printf("Idx: %d\n", nnz_jac_g);
	//sum of lambdas = 1 4*2*opt->num_phases
	nnz_jac_g += 4*2*opt->num_phases;


//	printf("Idx: %d\n", nnz_jac_g);
	//feet pos equal between phases
	nnz_jac_g += 2*(opt->Mp_idx - opt->Mf_idx);


//	printf("Idx: %d\n", nnz_jac_g);
	//spring reference
	nnz_jac_g += 2*(opt->Mr_idx - opt->Mp_idx);


	//printf("Idx: %d\n", nnz_jac_g);
	//lambda constraints between phases
	nnz_jac_g += 2*(opt->Ml_idx - opt->Mr_idx);


	//printf("Idx: %d\n", nnz_jac_g);
	//feet pos within bounds
//	nnz_jac_g += 2*(opt->Mp_nl_idx - opt->Ml_idx);
//	printf("here 1");
	//TODO: nonlinear foot constraint (5 dependent vars per cons)
	nnz_jac_g += 5*(opt->Mp_nl_idx - opt->Ml_idx);
	//printf("Idx: %d\n", nnz_jac_g);

	nnz_jac_g += 4*(opt->Mp_a_idx - opt->Mp_nl_idx);


	//printf("Idx: %d\n", nnz_jac_g);
	//ignore hessian for now... probably big speedup given everything is nonlinear
	nnz_h_lag = 0;

	// use the C style indexing (0-based)
	index_style = TNLP::C_STYLE;

	return true;
}

// returns the variable bounds
bool MPC_NLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
		Index m, Number* g_l, Number* g_u)
{
	// here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
	// If desired, we could assert to make sure they are what we think they are.
	assert(n == opt->N);
	assert(m == opt->M);

	for (int i = 0; i < 2; i++)
	{
		x_l[i] = -100.0;
		x_u[i] = 100.0;
	}
	x_l[2] = std::max(0.25, 0.8*std::min(opt->x0[2],opt->xT[2]));
	x_u[2] = 1.1;
	x_l[3] = -M_PI;
	x_u[3] = M_PI;
	for (int i = 4; i < 8; i++)
	{
		x_l[i] = -10.0;
		x_u[i] = 10.0;
	}
	for (int i = 8; i < 10; i++)
	{
		x_l[i] = x_l[i+3] = -100.0;
		x_u[i] = x_u[i+3] = 100.0;
	}
	x_l[10] = x_l[13] = -M_PI;
	x_u[10] = x_u[13] = M_PI;
	for (int i = 14; i < 18; i++)
	{
		x_l[i] = x_l[i+4] = 0.0;
		x_u[i] = x_u[i+4] = 1.0;
	}
	x_l[22] = x_l[23] = 0.25;
	x_u[22] = x_u[23] = 1.5;
	x_l[24] = -10.0; //alpha_dot
	x_u[24] = 10.0;

	for (int i = 1; i < opt->num_phases; i++)
	{
		for (int j = 0; j < 25; j++)
		{
			x_l[i*25+j] = x_l[j];
			x_u[i*25+j] = x_u[j];
		}
	}

	for (int i = 0; i < 14; i++)
		x_l[i] = x_u[i] = opt->x0[i];

//	//bounding box constraint for swing foot
//	if (opt->phase[0].eType == SS_Left)
//	{
//		x_l[11] = opt->x0[0] + opt->foot_nom[2] - opt->foot_ext[0];
//		x_u[11] = opt->x0[0] + opt->foot_nom[2] + opt->foot_ext[0];
//		x_l[12] = opt->x0[1] + opt->foot_nom[3] - opt->foot_ext[1];
//		x_u[12] = opt->x0[1] + opt->foot_nom[3] + opt->foot_ext[1];
//		x_l[13] = -M_PI;
//		x_u[13] = M_PI;
//	}
//	else if (opt->phase[0].eType == SS_Right)
//	{
//		x_l[8] = opt->x0[0] + opt->foot_nom[0] - opt->foot_ext[0];
//		x_u[8] = opt->x0[0] + opt->foot_nom[0] + opt->foot_ext[0];
//		x_l[9] = opt->x0[1] + opt->foot_nom[1] - opt->foot_ext[1];
//		x_u[9] = opt->x0[1] + opt->foot_nom[1] + opt->foot_ext[1];
//		x_l[10] = -M_PI;
//		x_u[10] = M_PI;
//	}

	//printf("here 21");
	//TODO: nonlinear foot constraint
	//bounding box constraint for swing foot
	if (opt->phase[0].eType == SS_Left)
	{
		x_l[11] = x_l[12] = -100.0;
		x_u[11] = x_u[12] = 100.0;
		x_l[13] = -M_PI;
		x_u[13] = M_PI;
	}
	else if (opt->phase[0].eType == SS_Right)
	{
		x_l[8] = x_l[9] = -100.0;
		x_u[8] = x_u[9] = 100.0;
		x_l[10] = -M_PI;
		x_u[10] = M_PI;
	}

	//limit lambda to only when in contact
	for (int i = 0; i < opt->num_phases; i++)
	{
		if (opt->phase[i].eType == SS_Left)
			x_u[i*25 + 16] = x_u[i*25 + 17] = x_u[i*25 + 20] = x_u[i*25 + 21] = 0.0;
		if (opt->phase[i].eType == SS_Right)
			x_u[i*25 + 14] = x_u[i*25 + 15] = x_u[i*25 + 18] = x_u[i*25 + 19] = 0.0;
	}

	//last r_T should be greater or equal to xT[2]
	x_l[(opt->num_phases-1)*25 + 23] = opt->xT[2];

	//last foot positions within xT bounds
//	x_l[(opt->num_phases-1)*25 + 8] = opt->xT[0] + opt->foot_nom[0] - opt->foot_ext[0];
//	x_u[(opt->num_phases-1)*25 + 8] = opt->xT[0] + opt->foot_nom[0] + opt->foot_ext[0];
//	x_l[(opt->num_phases-1)*25 + 9] = opt->xT[1] + opt->foot_nom[1] - opt->foot_ext[1];
//	x_u[(opt->num_phases-1)*25 + 9] = opt->xT[1] + opt->foot_nom[1] + opt->foot_ext[1];
//	x_l[(opt->num_phases-1)*25 + 11] = opt->xT[0] + opt->foot_nom[2] - opt->foot_ext[0];
//	x_u[(opt->num_phases-1)*25 + 11] = opt->xT[0] + opt->foot_nom[2] + opt->foot_ext[0];
//	x_l[(opt->num_phases-1)*25 + 12] = opt->xT[1] + opt->foot_nom[3] - opt->foot_ext[1];
//	x_u[(opt->num_phases-1)*25 + 12] = opt->xT[1] + opt->foot_nom[3] + opt->foot_ext[1];

	//TODO: nonlinear box constraints
	x_l[(opt->num_phases-1)*25 + 8] = x_u[(opt->num_phases-1)*25 + 8] = opt->xT[0] + opt->foot_nom[0]*cos(opt->xT[3]) - opt->foot_nom[1]*sin(opt->xT[3]);
	x_l[(opt->num_phases-1)*25 + 9] = x_u[(opt->num_phases-1)*25 + 9] = opt->xT[1] + opt->foot_nom[1]*cos(opt->xT[3]) + opt->foot_nom[0]*sin(opt->xT[3]);
	x_l[(opt->num_phases-1)*25 + 11] = x_u[(opt->num_phases-1)*25 + 11] = opt->xT[0] + opt->foot_nom[2]*cos(opt->xT[3]) - opt->foot_nom[3]*sin(opt->xT[3]);
	x_l[(opt->num_phases-1)*25 + 12] = x_u[(opt->num_phases-1)*25 + 12] = opt->xT[1] + opt->foot_nom[3]*cos(opt->xT[3]) + opt->foot_nom[2]*sin(opt->xT[3]);
	x_l[(opt->num_phases-1)*25 + 10] = x_u[(opt->num_phases-1)*25 + 10] = x_l[(opt->num_phases-1)*25 + 13] = x_u[(opt->num_phases-1)*25 + 13] = opt->xT[3];

	//dynamics equality constraints
	for (int i = 0; i < opt->Mc_idx-8; i++)
		g_l[i] = g_u[i] = 0.0;

	//terminal state constraint
	for (int i = opt->Mc_idx-8; i < opt->Mc_idx; i++)
		g_l[i] = g_u[i] = opt->xT[i-(opt->Mc_idx-8)];

	//sum of lambda constraints
	for (int i = opt->Mc_idx; i < opt->Mf_idx; i++)
		g_l[i] = g_u[i] = 1.0;

	//foot phase equalities
	for (int i = opt->Mf_idx; i < opt->Mp_idx; i++)
		g_l[i] = g_u[i] = 0.0;

	//spring reference
	for (int i = opt->Mp_idx; i < opt->Mp_idx+(opt->num_phases-1); i++)
		g_l[i] = g_u[i] = 0.0;
	for (int i = opt->Mp_idx+(opt->num_phases-1); i < opt->Mr_idx; i++)
	{
		g_l[i] = 0.0;
		g_u[i] = 1.0;
	}

	//lambda equality constraints
	for (int i = opt->Mr_idx; i < opt->Ml_idx; i++)
		g_l[i] = g_u[i] = 0.0;

	for (int i = opt->Ml_idx; i < opt->Mp_nl_idx; i+=4)
	{
		g_l[i] = -opt->foot_ext[0] + opt->foot_nom[0];
		g_u[i] = opt->foot_ext[0] + opt->foot_nom[0];
		g_l[i+1] = -opt->foot_ext[1] + opt->foot_nom[1];
		g_u[i+1] = opt->foot_ext[1] + opt->foot_nom[1];
		g_l[i+2] = -opt->foot_ext[0] + opt->foot_nom[2];
		g_u[i+2] = opt->foot_ext[0] + opt->foot_nom[2];
		g_l[i+3] = -opt->foot_ext[1] + opt->foot_nom[3];
		g_u[i+3] = opt->foot_ext[1] + opt->foot_nom[3];

	}

	for (int i = opt->Mp_nl_idx; i < opt->Mp_a_idx; i++)
		g_l[i] = g_u[i] = 0.0;

//	for (int i = 0; i < opt->M; i++)
		//printf("%f\t%f\n", g_l[i], g_u[i]);


	return true;
}

// returns the initial point for the problem
bool MPC_NLP::get_starting_point(Index n, bool init_x, Number* x,
		bool init_z, Number* z_L, Number* z_U,
		Index m, bool init_lambda,
		Number* lambda)
{
	// Here, we assume we only have starting values for x, if you code
	// your own NLP, you can provide starting values for the dual variables
	// if you wish
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);

	if (m_bWarmStart)
	{
		for (int i = 0; i < opt->N; i++)
			x[i] = x_last[i];
		return true;
	}

	for (int i = 0; i < 14; i++)
		x[i] = opt->x0[i];
	for (int i = 14; i < 22; i++)
		x[i] = 0.25;
	for (int i = 22; i < 24; i++)
		x[i] = opt->x0[2];
	x[24] = 0.0;

	for (int i = 1; i < opt->num_phases; i++)
		for (int j = 0; j < 25; j++)
			x[i*25+j] = x[j];

	return true;
}

// returns the value of the objective function
bool MPC_NLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	assert(n == opt->N);

	obj_value = 0.0;
	if (!m_bUseObjective)
		return true;

	//limit addot
	for (int i = 0; i < opt->num_phases; i++)
		obj_value += pow(x[i*25+24],2.0);

	//limit zddot
	for (int i = 1; i < opt->num_phases; i++)
	{
		obj_value += pow((x[i*25+22]-x[i*25+2]) - opt->mass*opt->g/opt->stiffness,2.0);
	}
	obj_value += pow((x[(opt->num_phases-1)*25+23]-opt->xT[2]) - opt->mass*opt->g/opt->stiffness,2.0);

//	for (int i = 1; i < opt->num_phases; i++)
//		obj_value += pow(x[i*25+2] - (opt->xT[2]+opt->x0[2])/2.0,2.0);

	return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool MPC_NLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
	assert(n == opt->N);

	for (int i = 0; i < opt->N; i++)
		grad_f[i] = 0.0;
	if (!m_bUseObjective)
			return true;

	//addot
	for (int i = 0; i < opt->num_phases; i++)
		grad_f[i*25+24] = 2.0*x[i*25+24];

	//zddot
	for (int i = 1; i < opt->num_phases; i++)
	{
		grad_f[i*25+2] = -2.0*x[i*25+22] + 2.0*x[i*25+2] + 2.0*opt->mass*opt->g/opt->stiffness;
		grad_f[i*25+22] = -grad_f[i*25+2];
	}
	grad_f[(opt->num_phases-1)*25+23] = 2.0*x[(opt->num_phases-1)*25+23] - 2.0*opt->xT[2] - 2.0*opt->mass*opt->g/opt->stiffness;

//	for (int i = 1; i < opt->num_phases; i++)
//		grad_f[i*25+2] = 2.0*(x[i*25+2] - (opt->xT[2]+opt->x0[2])/2.0);

	return true;
}

// return the value of the constraints: g(x)
bool MPC_NLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	assert(n == opt->N);
	assert(m == opt->M);

	Number temp[] = {0.0, 0.0, 0.0, 0.0};
	for (int i = 0; i < opt->num_phases; i++)
	{
		PHASE_Params* s = (PHASE_Params*)(&(x[i*25]));
		opt->cos_a[i] = cos(s->c0[3]);
		opt->sin_a[i] = sin(s->c0[3]);
		if (i < opt->num_phases-1)
		{
			PHASE_Params* sp = (PHASE_Params*)(&(x[(i+1)*25]));
			opt->getXY(s, opt->phase[i].T, temp); //get x and xdot
			g[i*8] = temp[0] - sp->c0[0];
			g[i*8 + 1] = temp[1] - sp->c0[1];
			g[i*8 + 4] = temp[2] - sp->c0[4];
			g[i*8 + 5] = temp[3] - sp->c0[5];
			opt->getZ(s, i, temp); //get z and zdot
			g[i*8 + 2] = temp[0] - sp->c0[2];
			g[i*8 + 6] = temp[1] - sp->c0[6];
			opt->getA(s, opt->phase[i].T, temp); //get a and adot
			g[i*8 + 3] = temp[0] - sp->c0[3];
			g[i*8 + 7] = temp[1] - sp->c0[7];
		}
		else
		{
			opt->getXY(s, opt->phase[i].T, temp); //get x and y
			g[i*8] = temp[0];
			g[i*8 + 1] = temp[1];
			g[i*8 + 4] = temp[2];
			g[i*8 + 5] = temp[3];
			opt->getZ(s, i, temp); //get z
			g[i*8 + 2] = temp[0];
			g[i*8 + 6] = temp[1];
			opt->getA(s, opt->phase[i].T, temp); //get a
			g[i*8 + 3] = temp[0];
			g[i*8 + 7] = temp[1];
		}
	}

	for (int i = 0; i < opt->num_phases; i++)
	{
		PHASE_Params* s = (PHASE_Params*)(&(x[i*25]));
		g[opt->Mc_idx + i*2] = 0.0;
		for (int j = 0; j < 4; j++)
			g[opt->Mc_idx + i*2] += s->lam0[j];
		g[opt->Mc_idx + i*2 + 1] = 0.0;
		for (int j = 0; j < 4; j++)
			g[opt->Mc_idx + i*2 + 1] += s->lamT[j];
	}

	for (int i = 0; i < opt->foot_equalities.size(); i++)
	{
		std::vector<Index> foot_eq = opt->foot_equalities[i];
		for (int j = 0; j < 3; j++)
			g[opt->Mf_idx + i*3 + j] = x[foot_eq[0]*25 + 8 + 3*foot_eq[2] + j] - x[foot_eq[1]*25 + 8 + 3*foot_eq[2] + j];
	}

	for (int i = 0; i < opt->num_phases-1; i++)
		g[opt->Mp_idx + i] = x[i*25 + 23] - x[(i+1)*25 + 22];
	for (int i = 0; i < opt->num_phases; i++)
		g[opt->Mp_idx + opt->num_phases-1 + i] = x[i*25 + 22] - x[i*25 + 2];

	for (int i = 0; i < opt->num_phases-1; i++)
		for (int j = 0; j < 4; j++)
			g[opt->Mr_idx + i*4 + j] = x[i*25 + 18 + j] - x[(i+1)*25 + 14 + j];

//	for (int i = 1; i < opt->num_phases; i++)
//	{
//		g[opt->Ml_idx + (i-1)*4] = x[i*25 + 8] - x[i*25];
//		g[opt->Ml_idx + (i-1)*4+1] = x[i*25 + 9] - x[i*25+1];
//		g[opt->Ml_idx + (i-1)*4+2] = x[i*25 + 11] - x[i*25];
//		g[opt->Ml_idx + (i-1)*4+3] = x[i*25 + 12] - x[i*25+1];
//	}
	//TODO: nonlinear box constraints
	//printf("here 2\n");
	for (int i = 0; i < opt->num_phases; i++)
	{
		g[opt->Ml_idx + i*4] = (x[i*25 + 8] - x[i*25])*opt->cos_a[i] + (x[i*25 + 9] - x[i*25+1])*opt->sin_a[i];
		g[opt->Ml_idx + i*4+1] = (x[i*25 + 9] - x[i*25+1])*opt->cos_a[i] - (x[i*25 + 8] - x[i*25])*opt->sin_a[i];
		g[opt->Ml_idx + i*4+2] = (x[i*25 + 11] - x[i*25])*opt->cos_a[i] + (x[i*25 + 12] - x[i*25+1])*opt->sin_a[i];
		g[opt->Ml_idx + i*4+3] = (x[i*25 + 12] - x[i*25+1])*opt->cos_a[i] - (x[i*25 + 11] - x[i*25])*opt->sin_a[i];
		//printf("index: %d, next: %d, max: %d\n", opt->Ml_idx + i*4+3, opt->Mp_nl_idx, opt->M);
	}

	int j = 0;
	for (int i = 0; i < opt->num_phases; i++)
	{
		if (opt->phase[i].eType == SS_Left)
		{
			g[opt->Mp_nl_idx + j] = x[i*25 + 3] + x[i*25 + 7]*opt->phase[i].T + 0.5*x[i*25 + 24]*pow(opt->phase[i].T,2.0) - x[i*25 + 13];
			j++;
		}
		if (opt->phase[i].eType == SS_Right)
		{
			g[opt->Mp_nl_idx + j] = x[i*25 + 3] + x[i*25 + 7]*opt->phase[i].T + 0.5*x[i*25 + 24]*pow(opt->phase[i].T,2.0) - x[i*25 + 10];
			j++;
		}
	}
	return true;
}

void MPC_NLP::get_jacobian_structure(Index n, Index m, Index* iRow, Index* jCol, Index nele_jac)
{
	//what a pain in the ass
	int idx = 0;

	int x_idx[] = {0, 2, 4, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 25};
	int y_idx[] = {1, 2, 5, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 26};
	int z_idx[] = {2, 6, 22, 23, 27};
	int a_idx[] = {3, 7, 24, 28};

	//constraints x,y,z,a,xd,yd,zd,ad,2,3,...
	for (int i = 0; i < opt->num_phases; i++)
	{
		if (i < opt->num_phases-1)
		{
			//x
			for (int j = 0; j < 16; j++)
			{
				iRow[idx] = i*8;
				jCol[idx] = x_idx[j] + i*25;
				idx++;
			}
			//y
			for (int j = 0; j < 16; j++)
			{
				iRow[idx] = i*8 + 1;
				jCol[idx] = y_idx[j] + i*25;
				idx++;
			}
			//z
			for (int j = 0; j < 5; j++)
			{
				iRow[idx] = i*8 + 2;
				jCol[idx] = z_idx[j] + i*25;
				idx++;
			}
			//a
			for (int j = 0; j < 4; j++)
			{
				iRow[idx] = i*8 + 3;
				jCol[idx] = a_idx[j] + i*25;
				idx++;
			}
			//x_dot
			for (int j = 0; j < 16; j++)
			{
				iRow[idx] = i*8 + 4;
				if (j == 15)
					jCol[idx] = x_idx[j] + i*25 + 4;
				else
					jCol[idx] = x_idx[j] + i*25;
				idx++;
			}
			//y_dot
			for (int j = 0; j < 16; j++)
			{
				iRow[idx] = i*8 + 5;
				if (j == 15)
					jCol[idx] = y_idx[j] + i*25 + 4;
				else
					jCol[idx] = y_idx[j] + i*25;
				idx++;
			}
			//z_dot
			for (int j = 0; j < 5; j++)
			{
				iRow[idx] = i*8 + 6;
				if (j == 4)
					jCol[idx] = z_idx[j] + i*25 + 4;
				else
					jCol[idx] = z_idx[j] + i*25;
				idx++;
			}
			//a_dot
			for (int j = 0; j < 3; j++)
			{
				iRow[idx] = i*8 + 7;
				if (j == 2)
					jCol[idx] = a_idx[j+1] + i*25 + 4;
				else
					jCol[idx] = a_idx[j+1] + i*25;
				idx++;
			}
		}
		else
		{
			//x
			for (int j = 0; j < 15; j++)
			{
				iRow[idx] = i*8;
				jCol[idx] = x_idx[j] + i*25;
				idx++;
			}
			//y
			for (int j = 0; j < 15; j++)
			{
				iRow[idx] = i*8 + 1;
				jCol[idx] = y_idx[j] + i*25;
				idx++;
			}
			//z
			for (int j = 0; j < 4; j++)
			{
				iRow[idx] = i*8 + 2;
				jCol[idx] = z_idx[j] + i*25;
				idx++;
			}
			//a
			for (int j = 0; j < 3; j++)
			{
				iRow[idx] = i*8 + 3;
				jCol[idx] = a_idx[j] + i*25;
				idx++;
			}
			//x_dot
			for (int j = 0; j < 15; j++)
			{
				iRow[idx] = i*8 + 4;
				jCol[idx] = x_idx[j] + i*25;
				idx++;
			}
			//y_dot
			for (int j = 0; j < 15; j++)
			{
				iRow[idx] = i*8 + 5;
				jCol[idx] = y_idx[j] + i*25;
				idx++;
			}
			//z_dot
			for (int j = 0; j < 4; j++)
			{
				iRow[idx] = i*8 + 6;
				jCol[idx] = z_idx[j] + i*25;
				idx++;
			}
			//a_dot
			for (int j = 0; j < 2; j++)
			{
				iRow[idx] = i*8 + 7;
				jCol[idx] = a_idx[j+1] + i*25;
				idx++;
			}
		}
	}
//	//printf("Idx: %d | %d\n", idx, nele_jac);

	for (int i = 0; i < opt->num_phases; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			iRow[idx] = opt->Mc_idx + i*2;
			jCol[idx] = i*25 + 14 + j;
			idx++;
		}
		for (int j = 0; j < 4; j++)
		{
			iRow[idx] = opt->Mc_idx + i*2 + 1;
			jCol[idx] = i*25 + 18 + j;
			idx++;
		}
	}
//	//printf("Idx: %d | %d\n", idx, nele_jac);

	for (int i = 0; i < opt->foot_equalities.size(); i++)
	{
		std::vector<Index> foot_eq = opt->foot_equalities[i];
		for (int j = 0; j < 3; j++)
		{
			iRow[idx] = opt->Mf_idx + i*3 + j;
			jCol[idx] = foot_eq[0]*25 + 8 + 3*foot_eq[2] + j;
			idx++;
			iRow[idx] = opt->Mf_idx + i*3 + j;
			jCol[idx] = foot_eq[1]*25 + 8 + 3*foot_eq[2] + j;
			idx++;
		}
	}
//	//printf("Idx: %d | %d\n", idx, nele_jac);

	//rT- = r0+
	for (int i = 0; i < opt->num_phases-1; i++)
	{
		iRow[idx] = opt->Mp_idx + i;
		jCol[idx] = i*25 + 23;
		idx++;
		iRow[idx] = opt->Mp_idx + i;
		jCol[idx] = (i+1)*25 + 22;
		idx++;
	}
//	//printf("Idx: %d | %d\n", idx, nele_jac);
	//r0 < z0
	for (int i = 0; i < opt->num_phases; i++)
	{
		iRow[idx] = opt->Mp_idx + opt->num_phases-1 + i;
		jCol[idx] = i*25 + 22;
		idx++;
		iRow[idx] = opt->Mp_idx + opt->num_phases-1 + i;
		jCol[idx] = i*25 + 2;
		idx++;
	}
//	//printf("Idx: %d | %d\n", idx, nele_jac);

	for (int i = 0; i < opt->num_phases-1; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			iRow[idx] = opt->Mr_idx + i*4 + j;
			jCol[idx] = i*25 + 18 + j;
			idx++;
			iRow[idx] = opt->Mr_idx + i*4 + j;
			jCol[idx] = (i+1)*25 + 14 + j;
			idx++;
		}
	}
//	//printf("Idx: %d | %d\n", idx, nele_jac);

//	for (int i = 1; i < opt->num_phases; i++)
//	{
//		iRow[idx] = opt->Ml_idx + (i-1)*4;
//		jCol[idx] = i*25 + 8;
//		idx++;
//		iRow[idx] = opt->Ml_idx + (i-1)*4;
//		jCol[idx] = i*25;
//		idx++;
//
//		iRow[idx] = opt->Ml_idx + (i-1)*4+1;
//		jCol[idx] = i*25 + 9;
//		idx++;
//		iRow[idx] = opt->Ml_idx + (i-1)*4+1;
//		jCol[idx] = i*25+1;
//		idx++;
//
//		iRow[idx] = opt->Ml_idx + (i-1)*4+2;
//		jCol[idx] = i*25 + 11;
//		idx++;
//		iRow[idx] = opt->Ml_idx + (i-1)*4+2;
//		jCol[idx] = i*25;
//		idx++;
//
//		iRow[idx] = opt->Ml_idx + (i-1)*4+3;
//		jCol[idx] = i*25 + 12;
//		idx++;
//		iRow[idx] = opt->Ml_idx + (i-1)*4+3;
//		jCol[idx] = i*25+1;
//		idx++;
//	}
	//TODO: nonlinear box constraints
	for (int i = 0; i < opt->num_phases; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int const_idx[] = {0, 1, 3, 8, 9};
			for (int k = 0; k < 5; k++)
			{
				iRow[idx] = opt->Ml_idx + i*4 + j;
				int foot_offset = 3*(j / 2);
				jCol[idx] = i*25 + const_idx[k];
				if (k > 2)
					jCol[idx] += foot_offset;
				idx++;
			}
		}
	}
	int j = 0;
	for (int i = 0; i < opt->num_phases; i++)
	{
		if (opt->phase[i].eType == SS_Left)
		{
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 3;
			idx++;
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 7;
			idx++;
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 24;
			idx++;
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 13;
			idx++;
			j++;
		}
		if (opt->phase[i].eType == SS_Right)
		{
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 3;
			idx++;
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 7;
			idx++;
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 24;
			idx++;
			iRow[idx] = opt->Mp_nl_idx + j;
			jCol[idx] = i*25 + 10;
			idx++;
			j++;
		}
	}

//	//printf("Idx: %d | %d\n", idx, nele_jac);

}

// return the structure or values of the jacobian
bool MPC_NLP::eval_jac_g(Index n, const Number* x, bool new_x,
		Index m, Index nele_jac, Index* iRow, Index *jCol,
		Number* values)
{
	if (values == NULL) {
		// return the structure of the jacobian
		get_jacobian_structure(n, m, iRow, jCol, nele_jac);
	}
	else {
		// return the values of the jacobian of the constraints
		//sparse jacobian
		int idx = 0;
		for (int j = 0; j < opt->num_phases; j++)
		{
			PHASE_Params* s = (PHASE_Params*)(&(x[j*25]));
			Number dxy[4*15];
			Number dz[2*4];
			Number da[] = {1.0, opt->phase[j].T, 0.5*pow(opt->phase[j].T,2.0)};
			opt->getXYjacobian(s, opt->phase[j].T, dxy);
			opt->getZjacobian(s, j, dz);
			//x, y, z, alpha, then their derivatives
			for (int i = 0; i < 15; i++)
			{
				values[idx] = dxy[i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
			//y
			for (int i = 0; i < 15; i++)
			{
				values[idx] = dxy[15+i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
			//z
			for (int i = 0; i < 4; i++)
			{
				values[idx] = dz[i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
			//a
			for (int i = 0; i < 3; i++)
			{
				values[idx] = da[i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}

			//dx
			for (int i = 0; i < 15; i++)
			{
				values[idx] = dxy[30+i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
			//dy
			for (int i = 0; i < 15; i++)
			{
				values[idx] = dxy[45+i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
			//dz
			for (int i = 0; i < 4; i++)
			{
				values[idx] = dz[i+4];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
			//da
			for (int i = 0; i < 2; i++)
			{
				values[idx] = da[i];
				idx++;
			}
			if (j < opt->num_phases-1)
			{
				values[idx] = -1;
				idx++;
			}
		}

		for (int i = 0; i < 4*2*opt->num_phases; i++)
		{
			values[idx] = 1.0;
			idx++;
		}

		for (int i = 0; i < opt->foot_equalities.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				values[idx] = 1.0;
				idx++;
				values[idx] = -1.0;
				idx++;
			}
		}

		for (int i = opt->Mp_idx; i < opt->Mr_idx; i++)
		{
			values[idx] = 1.0;
			idx++;
			values[idx] = -1.0;
			idx++;
		}

		for (int i = opt->Mr_idx; i < opt->Ml_idx; i++)
		{
			values[idx] = 1.0;
			idx++;
			values[idx] = -1.0;
			idx++;
		}

//		for (int i = opt->Ml_idx; i < opt->Mp_nl_idx; i++)
//		{
//			values[idx] = 1.0;
//			idx++;
//			values[idx] = -1.0;
//			idx++;
//		}
		//TODO: nonlinear box constraint
//		printf("here 4\n");
		for (int i = 0; i < opt->num_phases; i++)
		{
			double px_l = x[i*25 + 8];
			double py_l = x[i*25 + 9];
			double px_r = x[i*25 + 11];
			double py_r = x[i*25 + 12];
			double foot_jac[] = {-opt->cos_a[i], -opt->sin_a[i], -(px_l - x[i*25])*opt->sin_a[i] + (py_l - x[i*25+1])*opt->cos_a[i], opt->cos_a[i], opt->sin_a[i],
								 opt->sin_a[i], -opt->cos_a[i], -(py_l - x[i*25+1])*opt->sin_a[i] + (x[i*25] - px_l)*opt->cos_a[i], -opt->sin_a[i], opt->cos_a[i],
								 -opt->cos_a[i], -opt->sin_a[i], -(px_r - x[i*25])*opt->sin_a[i] + (py_r - x[i*25+1])*opt->cos_a[i], opt->cos_a[i], opt->sin_a[i],
								 opt->sin_a[i], -opt->cos_a[i], -(py_r - x[i*25+1])*opt->sin_a[i] + (x[i*25] - px_r)*opt->cos_a[i], -opt->sin_a[i], opt->cos_a[i]};
			for (int j = 0; j < 20; j++)
				values[idx++] = foot_jac[j];
		}
//		printf("after eval - %d | %d\n", idx, nele_jac);

		int j = 0;
		for (int i = 0; i < opt->num_phases; i++)
		{
			if (opt->phase[i].eType != Double)
			{
				values[idx] = 1.0;
				idx++;
				values[idx] = opt->phase[i].T;
				idx++;
				values[idx] = 0.5*pow(opt->phase[i].T,2.0);
				idx++;
				values[idx] = -1.0;
				idx++;
			}
		}
		//printf("At idx: %d | %d\n", idx, nele_jac);
	}

	return true;
}

void MPC_NLP::get_hessian_structure(Index n, Index* iRow, Index* jCol)
{
	// return the structure. This is a symmetric matrix, fill the lower left
	// triangle only.
//	//printf("hessian structure...\n");
}

//return the structure or values of the hessian
bool MPC_NLP::eval_h(Index n, const Number* x, bool new_x,
		Number obj_factor, Index m, const Number* lambda,
		bool new_lambda, Index nele_hess, Index* iRow,
		Index* jCol, Number* values)
{
	if (values == NULL) {
		get_hessian_structure(n, iRow, jCol);
	}
	else {
		// return the values. This is a symmetric matrix, fill the lower left
		// triangle only
//		//printf("hessian eval\n");
	}

	return false;
}

void MPC_NLP::finalize_solution(SolverReturn status,
		Index n, const Number* x, const Number* z_L, const Number* z_U,
		Index m, const Number* g, const Number* lambda,
		Number obj_value,
		const IpoptData* ip_data,
		IpoptCalculatedQuantities* ip_cq)
{
	// here is where we would store the solution to variables, or write to a file, etc
	// so we could use the solution.

	// For this example, we write the solution to the console
//	std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
//	for (Index i=0; i<opt->p_idx; i+=10) {
//		//printf("x[%d]: %f\tx[%d]: %f\n", i, x[i], i+1, x[i+1]);
//	}
//	for (Index i=0; i<opt->N; i++) {
//		//printf("x[%d]: %f\n", i, x[i]);
//	}

	for (int i = 0; i < opt->N; i++)
		opt->x_last[i] = x_last[i] = x[i];

	printf("%d\t%f\t%d\t%f\n", opt->phase[0].eType, opt->phase[0].T, opt->phase[1].eType, opt->phase[1].T);
	printf("%f\t%f\t%f\t%f\t%f\n", x[3], x[7], x[24], x[28], x[32]);
//	for (int i = 0; i < opt->num_phases; i++)
//		printf("%f\t", x[i*25 + 24]);
//	printf("\n");

//	std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
//	for (Index i=0; i<n; i++) {
//		std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
//	}
//	for (Index i=0; i<n; i++) {
//		std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
//	}
//
//	std::cout << std::endl << std::endl << "Objective value" << std::endl;
//	std::cout << "f(x*) = " << obj_value << std::endl;

//	std::cout << std::endl << "Final value of the constraints:" << std::endl;
//	for (Index i=0; i<m ;i++) {
//		std::cout << "g(" << i << ") = " << g[i] << std::endl;
//	}
//
//	//printf("%d\t%d\t%d\n", opt->Mp_idx, opt->Mp_nl_idx, opt->Mc_nl_idx);
}

void MPC_NLP::GetSolution(ROM_Policy_Struct* targ_traj, double dt)
{
	//this is for display purposes
	Number xy[6];
	Number z[3];
	Number a[2];
	int nOffset = 0;
	double tOffset = 0.0;
	ROM_TrajPt_Struct traj_pt;
	ContactInfo_Struct con_info;

	for (int i = 0; i < opt->num_phases; i++)
	{
		PHASE_Params* s = (PHASE_Params*)(&(x_last[i*25]));
		int N = int(opt->phase[i].T/dt);
		for (int j = 0; j < N; j++)
		{
			if (nOffset + j >= targ_traj->com_traj.size())
				break;

			double t = double(j)*dt;
			opt->getXY_at_time(s, opt->phase[i].T, xy, t);
			opt->getZ_at_time(s, i, z, t);
			opt->getA(s, t, a);

			traj_pt.com[0] = xy[0];
			traj_pt.com[1] = xy[1];
			traj_pt.com[2] = z[0];
			traj_pt.com[3] = a[0];

			traj_pt.com_xdd[0] = xy[4];
			traj_pt.com_xdd[1] = xy[5];
			traj_pt.com_xdd[2] = z[2];
			traj_pt.com_xdd[3] = s->add;

			targ_traj->com_traj[nOffset + j] = traj_pt;
			targ_traj->numPoints = nOffset+j+1;
		}
		nOffset += N;

		Number* left_foot = s->pfL;
		Number* right_foot = s->pfR;
		con_info.T_start = tOffset;
		tOffset += opt->phase[i].T;
		con_info.T_end = tOffset;
		con_info.con_state = opt->phase[i].eType;
		con_info.left[0] = left_foot[0];
		con_info.left[1] = left_foot[1];
		con_info.left[3] = left_foot[2];
		con_info.right[0] = right_foot[0];
		con_info.right[1] = right_foot[1];
		con_info.right[3] = right_foot[2];
		targ_traj->con_sched[i] = con_info;
	}
	targ_traj->numContactSwitch = opt->num_phases;
	targ_traj->dt_c = dt;
}
