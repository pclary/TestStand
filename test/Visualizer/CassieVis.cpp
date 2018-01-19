/*
 * CassieVis.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: tapgar
 */

#include "CassieVis.h"

using namespace std;
using namespace Eigen;

CassieVis::CassieVis(mjModel *m, bool save_vid, const char* win_title) {
	mj_Model = m;
	m_Width = 1200;
	m_Height = 900;

	m_bTrajInit = false;

	Init(save_vid, win_title);

	profilerInit();

	bWaitForUserFeedback = PAUSE_VIS;
	bOKtoComplete = false;
	m_bReset = false;
	m_bPlanInit = false;

	m_nTargetIndex = 0;

	m_nUserMenuSelectIndex = 0;
}

CassieVis::~CassieVis() {
	// TODO Auto-generated destructor stub
}

static void window_close_callback(GLFWwindow* window)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Close();
}
static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Mouse_Move(xpos, ypos);
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Mouse_Button(button, act, mods);
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}


int CassieVis::Init(bool save_video, const char* win_title) {

	if (!glfwInit()) {
		mju_error("Could not initialize GLFW");
		return 1;
	}

	// Create window
	m_Window = glfwCreateWindow(m_Width, m_Height, win_title, NULL, NULL);
	glfwMakeContextCurrent(m_Window);
	glfwSwapInterval(1);
	mjv_defaultCamera(&mj_Cam);
	// Set up mujoco visualization objects
	mj_Cam.lookat[0] = mj_Model->stat.center[0];
	mj_Cam.lookat[1] = mj_Model->stat.center[1];
	mj_Cam.lookat[2] = 0.3 + mj_Model->stat.center[2];
	mj_Cam.type = mjCAMERA_FREE;
	mj_Cam.distance = 2.0;

#if TRACKING

	mj_Cam.azimuth = 135;

	mj_Cam.distance = 3.0;
	mj_Cam.lookat[2] += 0.8;
	mj_Cam.type = mjCAMERA_TRACKING;
	mj_Cam.trackbodyid = 1;

#endif

	mjv_defaultOption(&mj_Opt);
	mjr_defaultContext(&mj_Con);
	mjv_makeScene(&mj_Scn, 1E5);

	mjr_makeContext(mj_Model, &mj_Con, mjFONTSCALE_100);

	mj_Cam.elevation = 0.0;
	mjv_moveCamera(mj_Model, mjMOUSE_ROTATE_H, 0.0, 0.0, &mj_Scn, &mj_Cam);

	// Set callback for user-initiated window close events
	glfwSetWindowUserPointer(m_Window, this);
	glfwSetWindowCloseCallback(m_Window, window_close_callback);
	glfwSetCursorPosCallback(m_Window, mouse_move);
	glfwSetMouseButtonCallback(m_Window, mouse_button);
	glfwSetScrollCallback(m_Window, scroll);
	glfwSetKeyCallback(m_Window, keyboard);

	if (save_video)
	{
		m_image_rgb = (unsigned char*)malloc(3*m_Width*m_Height);
		m_image_depth = (float*)malloc(sizeof(float)*m_Width*m_Height);

		// create output rgb file
		fp = fopen("out/temp.out", "wb");
		if( !fp )
			mju_error("Could not open rgbfile for writing");
	}

	m_bSaveVideo = save_video;

	return 0;
}

void CassieVis::profilerInit()
{

	mjv_defaultFigure(&figCOMxdd);
	mjv_defaultFigure(&figLeftxdd);
	mjv_defaultFigure(&figRightxdd);
	mjv_defaultFigure(&figTauLeft);
	mjv_defaultFigure(&figTauRight);
	mjv_defaultFigure(&figCOMXY);
	mjv_defaultFigure(&figCOMXZ);
	mjv_defaultFigure(&figMPC);


	//title
	strcpy(figCOMxdd.title, "COM xdd");
	strcpy(figRightxdd.title, "Right xdd");
	strcpy(figLeftxdd.title, "Left xdd");
	strcpy(figTauLeft.title, "Tau Left");
	strcpy(figTauRight.title, "Tau Right");
	strcpy(figCOMXY.title, "COM XY");
	strcpy(figCOMXZ.title, "COM XZ");
	strcpy(figMPC.title, "Stats");

	//x-labels
	strcpy(figCOMxdd.xlabel, "time");
	strcpy(figRightxdd.xlabel, "time");
	strcpy(figLeftxdd.xlabel, "time");
	strcpy(figTauLeft.xlabel, "time");
	strcpy(figTauRight.xlabel, "time");
	strcpy(figMPC.xlabel, "time");
	strcpy(figCOMXY.xlabel, "X");
	strcpy(figCOMXZ.xlabel, "X");

	//y-tick number format
	strcpy(figCOMxdd.yformat, "%.4f");
	strcpy(figLeftxdd.yformat, "%.4f");
	strcpy(figRightxdd.yformat, "%.4f");
	strcpy(figTauLeft.yformat, "%.4f");
	strcpy(figTauRight.yformat, "%.4f");
	strcpy(figCOMXZ.yformat, "%.4f");
	strcpy(figCOMXY.yformat, "%.4f");
	strcpy(figMPC.yformat, "%.2f");


	strcpy(figCOMxdd.linename[0], "xdd");
	strcpy(figCOMxdd.linename[1], "ydd");
	strcpy(figCOMxdd.linename[2], "zdd");

	strcpy(figLeftxdd.linename[0], "xf");
	strcpy(figLeftxdd.linename[1], "yf");
	strcpy(figLeftxdd.linename[2], "zf");
	strcpy(figLeftxdd.linename[3], "xr");
	strcpy(figLeftxdd.linename[4], "yr");
	strcpy(figLeftxdd.linename[5], "zr");

	strcpy(figRightxdd.linename[0], "xf");
	strcpy(figRightxdd.linename[1], "yf");
	strcpy(figRightxdd.linename[2], "zf");
	strcpy(figRightxdd.linename[3], "xr");
	strcpy(figRightxdd.linename[4], "yr");
	strcpy(figRightxdd.linename[5], "zr");

	strcpy(figTauLeft.linename[0], "abd");
	strcpy(figTauLeft.linename[1], "yaw");
	strcpy(figTauLeft.linename[2], "hip");
	strcpy(figTauLeft.linename[3], "knee");
	strcpy(figTauLeft.linename[4], "toe");

	strcpy(figTauRight.linename[0], "abd");
	strcpy(figTauRight.linename[1], "yaw");
	strcpy(figTauRight.linename[2], "hip");
	strcpy(figTauRight.linename[3], "knee");
	strcpy(figTauRight.linename[4], "toe");


	figCOMxdd.gridsize[0] = 5;
	figCOMxdd.gridsize[1] = 5;
	figLeftxdd.gridsize[0] = 5;
	figLeftxdd.gridsize[1] = 5;
	figRightxdd.gridsize[0] = 5;
	figRightxdd.gridsize[1] = 5;
	figTauRight.gridsize[0] = 5;
	figTauRight.gridsize[1] = 5;
	figTauLeft.gridsize[0] = 5;
	figTauLeft.gridsize[1] = 5;
	figCOMXY.gridsize[0] = 5;
	figCOMXY.gridsize[1] = 5;
	figCOMXZ.gridsize[0] = 5;
	figCOMXZ.gridsize[1] = 5;
	figMPC.gridsize[0] = 5;
	figMPC.gridsize[1] = 5;

	//ranges
	figCOMxdd.range[0][0] = -200;
	figCOMxdd.range[0][1] = 0;
	figCOMxdd.range[1][0] = 0;
	figCOMxdd.range[1][1] = 5000;
	figLeftxdd.range[0][0] = -200;
	figLeftxdd.range[0][1] = 0;
	figLeftxdd.range[1][0] = 0;
	figLeftxdd.range[1][1] = 5000;
	figRightxdd.range[0][0] = -200;
	figRightxdd.range[0][1] = 0;
	figRightxdd.range[1][0] = 0;
	figRightxdd.range[1][1] = 5000;
	figTauLeft.range[0][0] = -200;
	figTauLeft.range[0][1] = 0;
	figTauLeft.range[1][0] = 0;
	figTauLeft.range[1][1] = 5000;
	figTauRight.range[0][0] = -200;
	figTauRight.range[0][1] = 0;
	figTauRight.range[1][0] = 0;
	figTauRight.range[1][1] = 5000;

	figCOMXY.range[0][0] = 0;
	figCOMXY.range[0][1] = 0;
	figCOMXY.range[1][0] = 0;
	figCOMXY.range[1][1] = 0;
	figCOMXZ.range[0][0] = 0;
	figCOMXZ.range[0][1] = 0;
	figCOMXZ.range[1][0] = 0;
	figCOMXZ.range[1][1] = 0;

	figMPC.range[0][0] = -200;
	figMPC.range[0][1] = 0;
	figMPC.range[1][0] = 0;
	figMPC.range[1][1] = 100;


	//hide history plots
	for (int n = 0; n < 10; n++)
	{
		for (int i = 0; i < mjMAXLINEPNT; i++)
		{
			figCOMxdd.linedata[n][2*i] = (float)-i;
			figLeftxdd.linedata[n][2*i] = (float)-i;
			figRightxdd.linedata[n][2*i] = (float)-i;
			figTauRight.linedata[n][2*i] = (float)-i;
			figTauRight.linedata[n][2*i] = (float)-i;
			figMPC.linedata[n][2*i] = (float)-i;
		}
	}

}

void CassieVis::profilerUpdateXDD(telemetry_t t)
{
	int pnt = mjMIN(201, figCOMxdd.linepnt[0]+1);

	float dMax = -10000;
	float dMin = 10000;
	for (int n = 0; n < 3; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figCOMxdd.linedata[n][2*i+1] = figCOMxdd.linedata[n][2*i-1];
			dMax = max(figCOMxdd.linedata[n][2*i+1], dMax);
			dMin = min(figCOMxdd.linedata[n][2*i+1], dMin);
		}
		figCOMxdd.linepnt[n] = pnt;
		figCOMxdd.linedata[n][1] = t.accels[n];
	}
	figCOMxdd.range[1][0] = dMin - 0.1*fabs(dMin);
	figCOMxdd.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(201, figLeftxdd.linepnt[0]+1);
	for (int n = 0; n < 6; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figLeftxdd.linedata[n][2*i+1] = figLeftxdd.linedata[n][2*i-1];
			dMax = max(figLeftxdd.linedata[n][2*i+1], dMax);
			dMin = min(figLeftxdd.linedata[n][2*i+1], dMin);
		}
		figLeftxdd.linepnt[n] = pnt;
		figLeftxdd.linedata[n][1] = t.accels[3+n];
	}
	figLeftxdd.range[1][0] = dMin - 0.1*fabs(dMin);
	figLeftxdd.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(201, figRightxdd.linepnt[0]+1);
	for (int n = 0; n < 6; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figRightxdd.linedata[n][2*i+1] = figRightxdd.linedata[n][2*i-1];
			dMax = max(figRightxdd.linedata[n][2*i+1], dMax);
			dMin = min(figRightxdd.linedata[n][2*i+1], dMin);
		}
		figRightxdd.linepnt[n] = pnt;
		figRightxdd.linedata[n][1] = t.accels[9+n];
	}
	figRightxdd.range[1][0] = dMin - 0.1*fabs(dMin);
	figRightxdd.range[1][1] = dMax + 0.1*fabs(dMax);
}

void CassieVis::profilerUpdateTorques(telemetry_t t)
{
	float dMax = -10000;
	float dMin = 10000;

	int pnt = mjMIN(201, figTauLeft.linepnt[0]+1);
	for (int n = 0; n < 5; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figTauLeft.linedata[n][2*i+1] = figTauLeft.linedata[n][2*i-1];
			dMax = max(figTauLeft.linedata[n][2*i+1], dMax);
			dMin = min(figTauLeft.linedata[n][2*i+1], dMin);
		}
		figTauLeft.linepnt[n] = pnt;
		figTauLeft.linedata[n][1] = t.torques[n];
	}
	figTauLeft.range[1][0] = dMin - 0.1*fabs(dMin);
	figTauLeft.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;

	pnt = mjMIN(201, figTauRight.linepnt[0]+1);
	for (int n = 0; n < 5; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figTauRight.linedata[n][2*i+1] = figTauRight.linedata[n][2*i-1];
			dMax = max(figTauRight.linedata[n][2*i+1], dMax);
			dMin = min(figTauRight.linedata[n][2*i+1], dMin);
		}
		figTauRight.linepnt[n] = pnt;
		figTauRight.linedata[n][1] = t.torques[n+5];
	}
	figTauRight.range[1][0] = dMin - 0.1*fabs(dMin);
	figTauRight.range[1][1] = dMax + 0.1*fabs(dMax);
}

void CassieVis::profilerUpdateCOP(telemetry_t t)
{
	float dMaxX = -10000;
	float dMinX = 10000;
	float dMaxY = -10000;
	float dMinY = 10000;
	float dMaxZ = 1.0;
	float dMinZ = 10000;

	int pnt = 0;
	for (int i = 0; i < nCON; i++)
	{
		figCOMXY.linedata[0][pnt*2] = t.targ_pos[3 + i*3 + 0];
		figCOMXY.linedata[0][pnt*2+1] = t.targ_pos[3 + i*3 + 1];
		figCOMXZ.linedata[0][pnt*2] = t.targ_pos[3 + i*3 + 0];
		figCOMXZ.linedata[0][pnt*2+1] = t.targ_pos[3 + i*3 + 2];


		dMaxX = max(figCOMXY.linedata[0][pnt*2],dMaxX);
		dMinX = min(figCOMXY.linedata[0][pnt*2],dMinX);
		dMaxY = max(figCOMXY.linedata[0][pnt*2+1],dMaxY);
		dMinY = min(figCOMXY.linedata[0][pnt*2+1],dMinY);
		dMaxZ = max(figCOMXZ.linedata[0][pnt*2+1],dMaxZ);
		dMinZ = min(figCOMXZ.linedata[0][pnt*2+1],dMinZ);

		pnt++;
	}
	figCOMXY.linedata[0][pnt*2] = figCOMXY.linedata[0][0];
	figCOMXY.linedata[0][pnt*2+1] = figCOMXY.linedata[0][1];
	figCOMXZ.linedata[0][pnt*2] = figCOMXZ.linedata[0][0];
	figCOMXZ.linedata[0][pnt*2+1] = figCOMXZ.linedata[0][1];
	pnt++;

	figCOMXY.linepnt[0] = pnt;
	figCOMXZ.linepnt[0] = pnt;

	pnt = mjMIN(mjMAXLINEPNT, figCOMXY.linepnt[1]+1);

	for (int n = 1; n < 3; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figCOMXY.linedata[n][2*i+1] = figCOMXY.linedata[n][2*i-1];
			figCOMXY.linedata[n][2*i] = figCOMXY.linedata[n][2*i-2];

			figCOMXZ.linedata[n][2*i+1] = figCOMXZ.linedata[n][2*i-1];
			figCOMXZ.linedata[n][2*i] = figCOMXZ.linedata[n][2*i-2];
		}
	}

	figCOMXY.linedata[1][0] = t.qpos[0];
	figCOMXY.linedata[1][1] = t.qpos[1];
	figCOMXZ.linedata[1][0] = t.qpos[0];
	figCOMXZ.linedata[1][1] = t.qpos[2];

	figCOMXY.linedata[2][0] = t.targ_pos[0];
	figCOMXY.linedata[2][1] = t.targ_pos[1];
	figCOMXZ.linedata[2][0] = t.targ_pos[0];
	figCOMXZ.linedata[2][1] = t.targ_pos[2];

	figCOMXY.linepnt[1] = pnt;
	figCOMXZ.linepnt[1] = pnt;
	figCOMXY.linepnt[2] = pnt;
	figCOMXZ.linepnt[2] = pnt;

	figCOMXY.range[0][0] = dMinX - 0.1*fabs(dMinX);
	figCOMXY.range[0][1] = dMaxX + 0.1*fabs(dMaxX);
	figCOMXZ.range[0][0] = dMinX - 0.1*fabs(dMinX);
	figCOMXZ.range[0][1] = dMaxX + 0.1*fabs(dMaxX);

	figCOMXY.range[1][0] = dMinY - 0.1*fabs(dMinY);
	figCOMXY.range[1][1] = dMaxY + 0.1*fabs(dMaxY);
	figCOMXZ.range[1][0] = dMinZ - 0.1*fabs(dMinZ);
	figCOMXZ.range[1][1] = dMaxZ + 0.1*fabs(dMaxZ);

}

void CassieVis::profilerShow(mjrRect vp)
{
	mjrRect small_vp = vp;
	small_vp.width /= 4;
	small_vp.height /= 4;

	mjr_render(vp, &mj_Scn, &mj_Con);
	small_vp.bottom = 3*vp.height/4;
	small_vp.left = 3*vp.width/4;
	mjr_figure(small_vp, &figLeftxdd, &mj_Con);
	small_vp.bottom = 2*vp.height/4;
	mjr_figure(small_vp, &figRightxdd, &mj_Con);
	small_vp.bottom = 1*vp.height/4;
	mjr_figure(small_vp, &figCOMxdd, &mj_Con);

	small_vp.bottom = 3*vp.height/4;
	small_vp.left = 1*vp.height/4;
	mjr_figure(small_vp, &figCOMXY, &mj_Con);
	small_vp.left = 2*vp.height/4;
	mjr_figure(small_vp, &figCOMXZ, &mj_Con);

	small_vp.left = 0;
	mjr_figure(small_vp, &figTauLeft, &mj_Con);

	small_vp.bottom = 2*vp.height/4;
	mjr_figure(small_vp, &figTauRight, &mj_Con);

	glfwSwapBuffers(m_Window);
	glfwPollEvents();
}


bool CassieVis::Draw(mjData* data, telemetry_t t)
{
	if (!m_Window)
		return false;

	profilerUpdateXDD(t);
	profilerUpdateTorques(t);
	profilerUpdateCOP(t);

	timespec ts;
	static timespec tf;
	static bool bFirstTime = true;

	clock_gettime(CLOCK_REALTIME, &ts);
	double freq = 1e9/double(diff(tf,ts).tv_nsec);
	if (freq > max_frame_rate && !bFirstTime)
		return false;
	bFirstTime = false;
	clock_gettime(CLOCK_REALTIME, &tf);

	bool doOnce = true;
	mjrRect viewport = {0, 0, 0, 0};

	while ((bWaitForUserFeedback && !bOKtoComplete) || doOnce)
	{
		doOnce = false;
		// Set up for rendering
		glfwMakeContextCurrent(m_Window);
		glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);

		mjv_updateScene(mj_Model, data, &mj_Opt, NULL, &mj_Cam, mjCAT_ALL, &mj_Scn);

		mjtNum pos[3];
		pos[0] = t.targ_pos[0];
		pos[1] = t.targ_pos[1];
		pos[2] = t.targ_pos[2];

		double sphereSize = 0.01;
		float rgba[4] = {1.0, 0.0, 0.0, 1.0};
		mjtNum size[3] = {sphereSize,sphereSize,sphereSize};
		for (int i = 0; i < 3; i++)
		{
			size[i] = 0.5;
			if (i > 0)
				size[i-1] = sphereSize;
			mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
			mj_Scn.ngeom++;
		}
		glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);


		mjr_render(viewport, &mj_Scn, &mj_Con);

		profilerShow(viewport);

		mjrRect smallrect = viewport;
		smallrect.width = viewport.width - viewport.width/5;
		int calib = 0;
		int power = 0;
		if (t.op_state & OpState_Calibrated)
			calib = 1;
		if (t.op_state & OpState_MotorPower)
			power = 1;
		char user_info[1000] = "";
		sprintf(user_info, "%d\n%d\n%f\t%f\n%f\t%f\n%f\t%f\n%f\t%f\n%f\t%f\n%f\t%f",
				calib, power, t.Kp[0], t.Kd[0], t.Kp[1], t.Kd[1], t.Kp[2], t.Kd[2],
				t.Kp[3], t.Kd[3], t.Kp[4], t.Kd[4], t.Kp[5], t.Kd[5] );
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
				"Calibrated:\nMotor Power:\nCOM_X PD:\nCOM_Y PD:\nCOM_Z PD:\nFoot PD:\nFoot Z:\nRot PD:", user_info, &mj_Con);

		smallrect.left = 5;
		smallrect.height = 22;
		smallrect.bottom = 5 + smallrect.height*t.select_index;
		smallrect.width = 127;
		mjr_rectangle(smallrect, 1.0, 0.0, 0.0, 0.1);

		// Show updated scene
		glfwSwapBuffers(m_Window);
		glfwPollEvents();

	}
	bOKtoComplete = false;


	if (m_bReset)
	{
		m_bReset = false;
		return true;
	}

	return false;
}

// mouse button
void CassieVis::Mouse_Button(int button, int act, int mods)
{
	// update button state
	button_left =   (glfwGetMouseButton(m_Window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	button_middle = (glfwGetMouseButton(m_Window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	button_right =  (glfwGetMouseButton(m_Window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

	// update mouse position
	glfwGetCursorPos(m_Window, &cursor_lastx, &cursor_lasty);
}

// mouse move
void CassieVis::Mouse_Move(double xpos, double ypos)
{
	// no buttons down: nothing to do
	if( !button_left && !button_middle && !button_right )
		return;

	// compute mouse displacement, save
	double dx = xpos - cursor_lastx;
	double dy = ypos - cursor_lasty;
	cursor_lastx = xpos;
	cursor_lasty = ypos;

	// get current window size
	int width, height;
	glfwGetWindowSize(m_Window, &width, &height);

	// get shift key state
	bool mod_shift = (glfwGetKey(m_Window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
			glfwGetKey(m_Window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

	// determine action based on mouse button
	mjtMouse action;
	if( button_right )
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if( button_left )
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else
		action = mjMOUSE_ZOOM;

	mjv_moveCamera(mj_Model, action, dx/height, dy/height, &mj_Scn, &mj_Cam);
}

void CassieVis::Scroll(double xoffset, double yoffset)
{
	// scroll: emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(mj_Model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mj_Scn, &mj_Cam);
	//    printf("scroll callback %f, %f\n", xoffset, yoffset);
}

// keyboard
void CassieVis::Keyboard(int key, int scancode, int act, int mods)
{
	// do not act on release
	if( act==GLFW_RELEASE )
		return;

	if (key == GLFW_KEY_W)
		mj_Cam.azimuth += 45;
	if (key == GLFW_KEY_Q)
		mj_Cam.azimuth -= 45;

	if (key == GLFW_KEY_SPACE && !PAUSE_VIS)
	{
		bWaitForUserFeedback = !bWaitForUserFeedback;
	}
	if (key == GLFW_KEY_BACKSPACE)
		m_bReset = true;

	bOKtoComplete = true;
}

void CassieVis::Close() {
	// Free mujoco objects
	mjv_freeScene(&mj_Scn);
	mjr_freeContext(&mj_Con);

	// Close window
	glfwDestroyWindow(m_Window);
	m_Window = NULL;
}
