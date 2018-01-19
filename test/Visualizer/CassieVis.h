/*
 * CassieVis.h
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

class CassieVis {
public:
	CassieVis(mjModel *m, bool save_vid, const char* win_title);
	virtual ~CassieVis();

	int Init(bool save_video, const char* win_title);
	void Close();
	void Scroll(double xoffset, double yoffset);
	void Mouse_Button(int button, int act, int mods);
	void Mouse_Move(double xpos, double ypos);
	void Keyboard(int key, int scancode, int act, int mods);

	bool Draw(mjData* data, telemetry_t t);

	void profilerUpdateXDD(telemetry_t t);
	void profilerUpdateTorques(telemetry_t t);
	void profilerUpdateCOP(telemetry_t t);

	void HighlightIndex(int idx)
	{
		m_nTargetIndex = idx;
	}

	void SetMPCDisplay(int run_time);

private:

	mjModel *mj_Model;

	void profilerShow(mjrRect vp);
	void profilerShowMPC(mjrRect viewport);
	void profilerInit();
	void DrawTrajPoints(mjData* data);

	mjvFigure figCOMxdd;
	mjvFigure figLeftxdd;
	mjvFigure figRightxdd;
	mjvFigure figTauLeft;
	mjvFigure figTauRight;
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
	static const int m_nNumOptions = 6;

	static constexpr double cx[] = {0.079, -0.079};

	static constexpr double max_frame_rate = 30.0;


};

#endif /* VISUALIZER_H_ */
