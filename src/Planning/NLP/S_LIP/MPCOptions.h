/*
 * MPCOptions.h
 *
 *  Created on: Jan 23, 2018
 *      Author: tapgar
 */

#ifndef MPCOPTIONS_H_
#define MPCOPTIONS_H_

#include "IpTypes.hpp"

#define MAX_IPOPT_VARS 50000

#pragma pack(push, 1)
typedef struct {
	Ipopt::Number c0[8]; //com pos (3 translation + 1 rotation)
	Ipopt::Number pfL[3]; //left foot pos
	Ipopt::Number pfR[3]; //right foot pos
	Ipopt::Number lam0[4];
	Ipopt::Number lamT[4];
	Ipopt::Number r0;
	Ipopt::Number rT;
	Ipopt::Number add;
}PHASE_Params;
#pragma pack(pop)

class MPC_OPTIONS {

public:
	MPC_OPTIONS() {
		N = 0;
		M = 0;
	}

	void setFoot(Ipopt::Number* x_vec, int idx, Ipopt::Number px, Ipopt::Number py, Ipopt::Number a)
	{
		x_vec[8+idx*3] = px;
		x_vec[9+idx*3] = py;
		x_vec[10+idx*3] = a;
	}

	void calc_cop(const PHASE_Params* p, Ipopt::Number t, Ipopt::Number T, Ipopt::Number* cop)
	{
		const Ipopt::Number* left_foot = p->pfL;
		const Ipopt::Number* right_foot = p->pfR;
		const Ipopt::Number* lam0 = p->lam0;
		const Ipopt::Number* lamT = p->lamT;

		Ipopt::Number lam[] = {0.0, 0.0, 0.0, 0.0};
		for (int i = 0; i < 4; i++)
			lam[i] = lam0[i]*(1 - t/T) + lamT[i]*(t/T);

		Ipopt::Number caL = cos(left_foot[2]);
		Ipopt::Number saL = sin(left_foot[2]);
		Ipopt::Number caR = cos(right_foot[2]);
		Ipopt::Number saR = sin(right_foot[2]);

		cop[0] = lam[0]*(left_foot[0] + cx[0]*caL) +
				lam[1]*(left_foot[0] + cx[1]*caL) +
				lam[2]*(right_foot[0] + cx[0]*caR) +
				lam[3]*(right_foot[0] + cx[1]*caR);

		cop[1] = lam[0]*(left_foot[1] + cx[0]*saL) +
				lam[1]*(left_foot[1] + cx[1]*saL) +
				lam[2]*(right_foot[1] + cx[0]*saR) +
				lam[3]*(right_foot[1] + cx[1]*saR);
	}

	void getXY(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* xy)
	{
		Ipopt::Number p0[] = {0.0, 0.0};
		calc_cop(s, 0.0, T, p0);
		Ipopt::Number pT[] = {0.0, 0.0};
		calc_cop(s, T, T, pT);
		Ipopt::Number alpha = sqrt(g / s->c0[2]);

		for (int i = 0; i < 2; i++)
		{
			Ipopt::Number B1 = (s->c0[i] - p0[i])/2.0 + (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			Ipopt::Number B2 = (s->c0[i] - p0[i])/2.0 - (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			xy[i] = B1*exp(alpha*T) + B2*exp(-alpha*T) + pT[i];
			xy[i+2] = alpha*B1*exp(alpha*T) - alpha*B2*exp(-alpha*T) + (1/T)*(pT[i] - p0[i]);
		}
	}

	void getZ(const PHASE_Params* s, int phase_idx, Ipopt::Number* z)
	{
		Ipopt::Number T = phase[phase_idx].T;
		Ipopt::Number d1 = s->c0[2] - s->r0 + g_omega_2;
		Ipopt::Number d2 = s->c0[6]/omega - (s->rT - s->r0)/(T*omega);
		z[0] = d1*cwT[phase_idx] + d2*swT[phase_idx] + s->rT - g_omega_2;
		z[1] = -d1*omega*swT[phase_idx] + d2*omega*cwT[phase_idx] + (1/T)*(s->rT-s->r0);
	}

	void getXY_at_time(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* xy, Ipopt::Number t)
	{
		Ipopt::Number p0[] = {0.0, 0.0};
		calc_cop(s, 0.0, T, p0);
		Ipopt::Number pT[] = {0.0, 0.0};
		calc_cop(s, T, T, pT);
		Ipopt::Number pt[] = {0.0, 0.0};
		calc_cop(s, t, T, pt);
		Ipopt::Number alpha = sqrt(g / s->c0[2]);

		for (int i = 0; i < 2; i++)
		{
			Ipopt::Number B1 = (s->c0[i] - p0[i])/2.0 + (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			Ipopt::Number B2 = (s->c0[i] - p0[i])/2.0 - (s->c0[i+4]*T - (pT[i] - p0[i]))/(2*alpha*T);
			xy[i] = B1*exp(alpha*t) + B2*exp(-alpha*t) + pt[i];
			xy[i+2] = alpha*B1*exp(alpha*t) - alpha*B2*exp(-alpha*t) + (1/T)*(pT[i] - p0[i]);
			xy[i+4] = alpha*alpha*B1*exp(alpha*t) + alpha*alpha*B2*exp(-alpha*t);
		}
	}

	void getZ_at_time(const PHASE_Params* s, int phase_idx, Ipopt::Number* z, Ipopt::Number t)
	{
		Ipopt::Number T = phase[phase_idx].T;
		Ipopt::Number d1 = s->c0[2] - s->r0 + g_omega_2;
		Ipopt::Number d2 = s->c0[6]/omega - (s->rT - s->r0)/(T*omega);
		z[0] = d1*cos(omega*t) + d2*sin(omega*t) + (t/T)*(s->rT - s->r0) + s->r0 - g_omega_2;
		z[1] = -d1*omega*sin(omega*t) + d2*omega*cos(omega*t) + (1/T)*(s->rT-s->r0);
		z[2] = -d1*omega*omega*cos(omega*t) - d2*omega*omega*sin(omega*t);
	}

	void getZjacobian(const PHASE_Params* s, int phase_idx, Ipopt::Number* dz)
	{
		Ipopt::Number T = phase[phase_idx].T;
		dz[0] = cwT[phase_idx];
		dz[1] = swT[phase_idx]/omega;
		dz[2] = -cwT[phase_idx] + swT[phase_idx]/(T*omega);
		dz[3] = -swT[phase_idx]/(omega*T) + 1;
		dz[4] = -omega*swT[phase_idx];
		dz[5] = cwT[phase_idx];
		dz[6] = omega*swT[phase_idx] + cwT[phase_idx]/T - 1/T;
		dz[7] = -cwT[phase_idx]/T + 1/T;
	}

	void getA(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* a)
	{
		a[1] = s->c0[7] + s->add*T;
		a[0] = s->c0[3] + s->c0[7]*T + 0.5*s->add*T*T;
	}

	void getCOPjacobian(const PHASE_Params* s, Ipopt::Number* dp0dl, Ipopt::Number* dp0dpL, Ipopt::Number* dp0dpR, Ipopt::Number* dpTdl, Ipopt::Number* dpTdpL, Ipopt::Number* dpTdpR)
	{
		const Ipopt::Number* left_foot = s->pfL;
		const Ipopt::Number* right_foot = s->pfR;
		const Ipopt::Number* lam0 = s->lam0;
		const Ipopt::Number* lamT = s->lamT;

		Ipopt::Number caL = cos(left_foot[2]);
		Ipopt::Number saL = sin(left_foot[2]);
		Ipopt::Number caR = cos(right_foot[2]);
		Ipopt::Number saR = sin(right_foot[2]);

		//dp0dl
		dp0dl[0] = left_foot[0] + cx[0]*caL;
		dp0dl[1] = left_foot[1] + cx[0]*saL;
		dp0dl[2] = left_foot[0] + cx[1]*caL;
		dp0dl[3] = left_foot[1] + cx[1]*saL;
		dp0dl[4] = right_foot[0] + cx[0]*caR;
		dp0dl[5] = right_foot[1] + cx[0]*saR;
		dp0dl[6] = right_foot[0] + cx[1]*caR;
		dp0dl[7] = right_foot[1] + cx[1]*saR;
		for (int i = 0; i < 8; i++)
			dpTdl[i] = dp0dl[i];

		//dp0dpL
		dp0dpL[0] = lam0[0] + lam0[1];
		dp0dpL[1] = lam0[0] + lam0[1];
		dp0dpL[2] = -(lam0[0]*cx[0]*saL + lam0[1]*cx[1]*saL);
		dp0dpL[3] = (lam0[0]*cx[0]*caL + lam0[1]*cx[1]*caL);
		//dpTdpL
		dpTdpL[0] = lamT[0] + lamT[1];
		dpTdpL[1] = lamT[0] + lamT[1];
		dpTdpL[2] = -(lamT[0]*cx[0]*saL + lamT[1]*cx[1]*saL);
		dpTdpL[3] = (lamT[0]*cx[0]*caL + lamT[1]*cx[1]*caL);

		//dp0dpR
		dp0dpR[0] = lam0[2] + lam0[3];
		dp0dpR[1] = lam0[2] + lam0[3];
		dp0dpR[2] = -(lam0[2]*cx[0]*saR + lam0[3]*cx[1]*saR);
		dp0dpR[3] = (lam0[2]*cx[0]*caR + lam0[3]*cx[1]*caR);
		//dpTdpR
		dpTdpR[0] = lamT[2] + lamT[3];
		dpTdpR[1] = lamT[2] + lamT[3];
		dpTdpR[2] = -(lamT[2]*cx[0]*saR + lamT[3]*cx[1]*saR);
		dpTdpR[3] = (lamT[2]*cx[0]*caR + lamT[3]*cx[1]*caR);

	}

	void getXYjacobian(const PHASE_Params* s, Ipopt::Number T, Ipopt::Number* xy)
	{
		Ipopt::Number p0[] = {0.0, 0.0};
		calc_cop(s, 0.0, T, p0);
		Ipopt::Number pT[] = {0.0, 0.0};
		calc_cop(s, T, T, pT);
		Ipopt::Number alpha = sqrt(g / s->c0[2]);
		Ipopt::Number eaT = exp(alpha*T);
		Ipopt::Number enaT = exp(-alpha*T);

		//Beta1,2 for x and y
		Ipopt::Number B1[4];
		B1[0] = 0.5*(s->c0[0] - p0[0]);
		B1[1] = 0.5*(s->c0[1] - p0[1]);
		B1[2] = (s->c0[4]*T - (pT[0] - p0[0]))/(2*alpha*T);
		B1[3] = (s->c0[5]*T - (pT[1] - p0[1]))/(2*alpha*T);
		Ipopt::Number B2[4];
		B2[0] = B1[0];
		B2[1] = B1[1];
		B2[2] = -B1[2];
		B2[3] = -B1[3];

		//xy should be 4*15

		//dx/dx0 and dy/dy0
		xy[0] = 0.5*eaT + 0.5*enaT;
		xy[15] = xy[0];
		//dxd/dx0 and dyd/dy0
		xy[30] = 0.5*alpha*eaT - 0.5*alpha*enaT;
		xy[45] = xy[30];

		//dx/dz0 and dy/dz0
		xy[1] = 0.5*(B1[2]/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*(B1[0]+B1[2])*eaT +
				0.5*(B2[2]/s->c0[2])*enaT + (0.5*alpha*T/s->c0[2])*(B2[0]+B2[2])*enaT;
		xy[16] = 0.5*(B1[3]/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*(B1[1]+B1[3])*eaT +
				0.5*(B2[3]/s->c0[2])*enaT + (0.5*alpha*T/s->c0[2])*(B2[1]+B2[3])*enaT;
		//dxd/dz0 and dyd/dz0
		xy[31] = -(0.5*B1[0]*alpha/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*alpha*(B1[0]+B1[2])*eaT +
				+ (0.5*B2[0]*alpha/s->c0[2])*enaT + (-0.5*alpha*T/s->c0[2])*alpha*(B2[0]+B2[2])*enaT;
		xy[46] = -(0.5*B1[1]*alpha/s->c0[2])*eaT + (-0.5*alpha*T/s->c0[2])*alpha*(B1[1]+B1[3])*eaT +
						+ (0.5*B2[1]*alpha/s->c0[2])*enaT + (-0.5*alpha*T/s->c0[2])*alpha*(B2[1]+B2[3])*enaT;

		//dx/dxd0 and dy/dyd0
		xy[2] = (0.5/alpha)*eaT - (0.5/alpha)*enaT;
		xy[17] = xy[2];
		//dxd/dxd0 and dyd/dyd0
		xy[32] = 0.5*eaT + 0.5*enaT;
		xy[47] = xy[32];

		Ipopt::Number dp0dl[8];
		Ipopt::Number dp0dpL[4];
		Ipopt::Number dp0dpR[4];
		Ipopt::Number dpTdl[8];
		Ipopt::Number dpTdpL[4];
		Ipopt::Number dpTdpR[4];

		getCOPjacobian(s, dp0dl, dp0dpL, dp0dpR, dpTdl, dpTdpL, dpTdpR);

		for (int i = 0; i < 2; i++)
		{
			xy[3+i] = (-0.5*dp0dpL[i*2] - 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*eaT +
					(-0.5*dp0dpL[i*2] + 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*enaT + dpTdpL[i*2];
			xy[18+i] = (-0.5*dp0dpL[i*2+1] - 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*eaT +
					(-0.5*dp0dpL[i*2+1] + 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*enaT + dpTdpL[i*2+1];
			xy[33+i] = alpha*(-0.5*dp0dpL[i*2] - 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpL[i*2] + 0.5*(dpTdpL[i*2] - dp0dpL[i*2])/(alpha*T))*enaT + (1/T)*(dpTdpL[i*2]-dp0dpL[i*2]);
			xy[48+i] = alpha*(-0.5*dp0dpL[i*2+1] - 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpL[i*2+1] + 0.5*(dpTdpL[i*2+1] - dp0dpL[i*2+1])/(alpha*T))*enaT + (1/T)*(dpTdpL[i*2+1]-dp0dpL[i*2+1]);
		}

		for (int i = 0; i < 2; i++)
		{
			xy[5+i] = (-0.5*dp0dpR[i*2] - 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*eaT +
					(-0.5*dp0dpR[i*2] + 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*enaT + dpTdpR[i*2];
			xy[20+i] = (-0.5*dp0dpR[i*2+1] - 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*eaT +
					(-0.5*dp0dpR[i*2+1] + 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*enaT + dpTdpR[i*2+1];
			xy[35+i] = alpha*(-0.5*dp0dpR[i*2] - 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpR[i*2] + 0.5*(dpTdpR[i*2] - dp0dpR[i*2])/(alpha*T))*enaT + (1/T)*(dpTdpR[i*2]-dp0dpR[i*2]);
			xy[50+i] = alpha*(-0.5*dp0dpR[i*2+1] - 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dpR[i*2+1] + 0.5*(dpTdpR[i*2+1] - dp0dpR[i*2+1])/(alpha*T))*enaT + (1/T)*(dpTdpR[i*2+1]-dp0dpR[i*2+1]);
		}

		for (int i = 0; i < 4; i++)
		{
			xy[7+i] = (-0.5*dp0dl[i*2] + 0.5*dp0dl[i*2]/(alpha*T))*eaT +
					(-0.5*dp0dl[i*2] - 0.5*dp0dl[i*2]/(alpha*T))*enaT;
			xy[22+i] = (-0.5*dp0dl[i*2+1] + 0.5*dp0dl[i*2+1]/(alpha*T))*eaT +
					(-0.5*dp0dl[i*2+1] - 0.5*dp0dl[i*2+1]/(alpha*T))*enaT;
			xy[37+i] = alpha*(-0.5*dp0dl[i*2] + 0.5*dp0dl[i*2]/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dl[i*2] - 0.5*dp0dl[i*2]/(alpha*T))*enaT - (1/T)*dp0dl[i*2];
			xy[52+i] = alpha*(-0.5*dp0dl[i*2+1] + 0.5*dp0dl[i*2+1]/(alpha*T))*eaT +
					-alpha*(-0.5*dp0dl[i*2+1] - 0.5*dp0dl[i*2+1]/(alpha*T))*enaT - (1/T)*dp0dl[i*2+1];
		}

		for (int i = 0; i < 4; i++)
		{
			xy[11+i] = (-0.5*dpTdl[i*2]/(alpha*T))*eaT +
					(0.5*dpTdl[i*2]/(alpha*T))*enaT + dpTdl[i*2];
			xy[26+i] = (-0.5*dpTdl[i*2+1]/(alpha*T))*eaT +
					(0.5*dpTdl[i*2+1]/(alpha*T))*enaT + dpTdl[i*2+1];
			xy[41+i] = alpha*(-0.5*dpTdl[i*2]/(alpha*T))*eaT +
					-alpha*(0.5*dpTdl[i*2]/(alpha*T))*enaT + (1/T)*dpTdl[i*2];
			xy[56+i] = alpha*(-0.5*dpTdl[i*2+1]/(alpha*T))*eaT +
					-alpha*(0.5*dpTdl[i*2+1]/(alpha*T))*enaT + (1/T)*dpTdl[i*2+1];
		}

	}

	void GetParams(CommandInterface::policy_params_t* p)
	{
		p->num_phases = num_phases;
		for (int i = 0; i < p->num_phases; i++)
		{
			p->phases[i] = phase[i];
			for (int j = 0; j < 25; j++)
				p->x[i*25 + j] = x_last[i*25 + j];
		}
	}

	void GetSolution(double* x, PHASE_Info* phases, int n_phases, ROM_Policy_Struct* targ_traj, double dt)
	{
		//this is for display purposes
		Ipopt::Number xy[6];
		Ipopt::Number z[3];
		Ipopt::Number a[2];
		int nOffset = 0;
		double tOffset = 0.0;
		ROM_TrajPt_Struct traj_pt;
		ContactInfo_Struct con_info;

		for (int i = 0; i < n_phases; i++)
		{
			PHASE_Params* s = (PHASE_Params*)(&(x[i*25]));
			int N = int(phases[i].T/dt);
			for (int j = 0; j < N; j++)
			{
				if (nOffset + j >= targ_traj->com_traj.size())
					break;

				double t = double(j)*dt;
				getXY_at_time(s, phases[i].T, xy, t);
				getZ_at_time(s, i, z, t);
				getA(s, t, a);

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

			Ipopt::Number* left_foot = s->pfL;
			Ipopt::Number* right_foot = s->pfR;
			con_info.T_start = tOffset;
			tOffset += phases[i].T;
			con_info.T_end = tOffset;
			con_info.con_state = phases[i].eType;
			con_info.left[0] = left_foot[0];
			con_info.left[1] = left_foot[1];
			con_info.left[3] = left_foot[2];
			con_info.right[0] = right_foot[0];
			con_info.right[1] = right_foot[1];
			con_info.right[3] = right_foot[2];
			targ_traj->con_sched[i] = con_info;
		}
		targ_traj->numContactSwitch = n_phases;
		targ_traj->dt_c = dt;
	}


	int N; //num open vars
	int M; //num constraints
	int Mc_idx;
	int Mf_idx;
	int Mp_idx;
	int Mr_idx;
	int Ml_idx;
	int Mp_nl_idx;
	int Mp_a_idx;

	int num_phases;

	PHASE_Info phase[MAX_NUM_PHASES];

	Ipopt::Number x0[14]; //start state
	Ipopt::Number xT[8]; //target positions at the end

	std::vector<std::vector<Ipopt::Index>> foot_equalities;

	static constexpr double g = 9.806;
	static constexpr double stiffness = 1000;
	static constexpr double mass = 31.0;

	static constexpr double cx[] = {-0.079, 0.079};
	static constexpr double foot_ext[] = {0.25, 0.075};
	static constexpr double foot_nom[] = {0.0, 0.13, 0.0, -0.13};

	//auxilary variables that you only need to calc once
	Ipopt::Number cwT[MAX_NUM_PHASES];
	Ipopt::Number swT[MAX_NUM_PHASES];
	Ipopt::Number omega;
	Ipopt::Number g_omega_2;

	//expensive vars... only calc once per constraint eval
	Ipopt::Number cos_a[MAX_NUM_PHASES];
	Ipopt::Number sin_a[MAX_NUM_PHASES];

	Ipopt::Number x_last[MAX_IPOPT_VARS];
};



#endif /* MPCOPTIONS_H_ */
