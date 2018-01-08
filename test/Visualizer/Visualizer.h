/*
 * Visualizer.h
 *
 *  Created on: Jul 3, 2017
 *      Author: tapgar
 */

#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include <vector>
#include "Common_Structs.h"
#include <Eigen/Dense>
#include "SharedRobotDefinitions.h"
#include "Command_Structs.h"
#include "CommandInterface.h"
#include <sys/time.h>
#include "HelperFunctions.h"

#define PAUSE_VIS false
#define TRACKING false

using namespace std;
using namespace Eigen;

class Visualizer {
public:
	Visualizer(mjModel *m, bool save_vid, const char* win_title);
	virtual ~Visualizer();

	int Init(bool save_video, const char* win_title);
	void Close();
	void Scroll(double xoffset, double yoffset);
	void Mouse_Button(int button, int act, int mods);
	void Mouse_Move(double xpos, double ypos);
	void Keyboard(int key, int scancode, int act, int mods);

	bool Draw(mjData* data);
	void DrawWithPoints(mjData* data, vector<Vector3d> pts);

	void profilerUpdateXDD(Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd_target, Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd_e);
	void profilerUpdateForces( Eigen::Matrix<double, nCON*DOF, 1> f, Eigen::Matrix<double, nU, 1> u);
	void profilerUpdateQPError(double res);
	void profilerUpdateTracking(Eigen::Matrix<double, 16, 1> tracking);
	void profilerUpdateQDD(Eigen::Matrix<double, nQ, 1> Qdde);
	void profilerUpdateCOP(Eigen::Matrix<double, DOF, 1> COM, Eigen::Matrix<double, DOF, 1> COM_target, Eigen::Matrix<double, DOF, nCON> foot, bool* bContact);

	void GetCOMTargets(double* dx, double* dy, double* dz, double* dphi, double* dpsi);
	void GetDisturbanceForce(double* Fx, double* Fy, double* Fz);
	void UpdateUserParams(USER_Params* user_params);

	void DisplayPlan(vector<Point3D> com_traj, vector<Point> cop_traj, vector<Patch> foot_pos, bool bSuccess)
	{
		plan_com_traj = com_traj;
		plan_cop_traj = cop_traj;
		plan_foot_pos = foot_pos;
		m_bPlanInit = true;
		m_bSuccess = bSuccess;
	}

	void DrawMPCPlan(mjData* data);


	void SetTrajPoints(vector<ROMPosMatrix> pts, int N)
	{
		m_nNumPoints = N;
		traj_pts = pts;
		m_bTrajInit = true;
	}

	void HighlightIndex(int idx)
	{
		m_nTargetIndex = idx;
	}

	void SetMPCDisplay(int run_time);

	void SetMPCPlan(ROM_Policy_Struct* targ_traj)
	{
		Point3D com;
		Patch foot;
		plan_com_traj.clear();
		plan_cop_traj.clear();
		plan_foot_pos.clear();
		for (int i = 0; i < targ_traj->numPoints; i++)
		{
			com.x_m = targ_traj->com_traj[i].com[0];
			com.y_m = targ_traj->com_traj[i].com[1];
			com.z_m = targ_traj->com_traj[i].com[2];
			com.a_rad = targ_traj->com_traj[i].com[3];
			plan_com_traj.push_back(com);
		}

		for (int i = 0; i < targ_traj->numContactSwitch; i++)
		{
			foot.lb.x_m = targ_traj->con_sched[i].left[0] + cx[0]*cos(targ_traj->con_sched[i].left[3]);
			foot.lb.y_m = targ_traj->con_sched[i].left[1] + cx[0]*sin(targ_traj->con_sched[i].left[3]);
			foot.ub.x_m = targ_traj->con_sched[i].left[0] + cx[1]*cos(targ_traj->con_sched[i].left[3]);
			foot.ub.y_m = targ_traj->con_sched[i].left[1] + cx[1]*sin(targ_traj->con_sched[i].left[3]);
			plan_foot_pos.push_back(foot);
			foot.lb.x_m = targ_traj->con_sched[i].right[0] + cx[0]*cos(targ_traj->con_sched[i].right[3]);
			foot.lb.y_m = targ_traj->con_sched[i].right[1] + cx[0]*sin(targ_traj->con_sched[i].right[3]);
			foot.ub.x_m = targ_traj->con_sched[i].right[0] + cx[1]*cos(targ_traj->con_sched[i].right[3]);
			foot.ub.y_m = targ_traj->con_sched[i].right[1] + cx[1]*sin(targ_traj->con_sched[i].right[3]);
			plan_foot_pos.push_back(foot);
		}
		m_bPlanInit = true;
	}

private:

	mjModel *mj_Model;

	void profilerShow(mjrRect vp);
	void profilerShowMPC(mjrRect viewport);
	void profilerInit();
	void DrawTrajPoints(mjData* data);

	mjvFigure figCOMxdd;
	mjvFigure figLeftxdd;
	mjvFigure figRightxdd;
	mjvFigure figForce;
	mjvFigure figTau;
	mjvFigure figRes;
	mjvFigure figPos;
	mjvFigure figVel;
	mjvFigure figQddB;
	mjvFigure figQddL;
	mjvFigure figQddR;
	mjvFigure figCOMXY;
	mjvFigure figCOMXZ;
	mjvFigure figMPC;

	GLFWwindow* m_Window;
	GLFWwindow* m_FigWindow;
	GLFWwindow* m_QddWindow;
	mjvCamera mj_Cam;
	mjvOption mj_Opt;
	mjvScene mj_Scn;
	mjrContext mj_Con;
	mjrContext mj_FigCon;
	mjrContext mj_QddCon;
	unsigned char* m_image_rgb;
	unsigned char* m_image_rgb90;
	float* m_image_depth;

	int m_Width;
	int m_Height;
	bool m_bSaveVideo;
	bool m_bReset;
	int m_nNumPoints;

	FILE *fp;

	bool button_left = false;
	bool button_middle = false;
	bool button_right =  false;
	double cursor_lastx = 0;
	double cursor_lasty = 0;

	bool bOKtoComplete;
	bool bWaitForUserFeedback;


	vector<ROMPosMatrix> traj_pts;
	bool m_bTrajInit;
	int m_nTargetIndex;

	void profilerShowXZ(mjrRect vp);
	void profilerShowXY(mjrRect vp);

	double dCOMdx_targ;
	double dCOMdy_targ;
	double dCOMdz_targ;
	double dCOMdphi_targ;
	double dCOMdpsi_targ;

	bool m_bPlanInit;
	vector<Point3D> plan_com_traj;
	vector<Point> plan_cop_traj;
	vector<Patch> plan_foot_pos;
	bool m_bSuccess;

	USER_Params user_opt;
	int m_nUserMenuSelectIndex;
	static const int m_nNumOptions = 4;

	static constexpr double cx[] = {0.079, -0.079};

	static constexpr double max_frame_rate = 30.0;


};

#endif /* VISUALIZER_H_ */
