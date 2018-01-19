/*
 * Visualizer.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: tapgar
 */

#include "Visualizer.h"

using namespace std;
using namespace Eigen;

Visualizer::Visualizer(mjModel *m, bool save_vid, const char* win_title) {
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

	dCOMdx_targ = 0.0;
	dCOMdy_targ = 0.0;
	dCOMdz_targ = 0.0;
	dCOMdphi_targ = 0.0;
	dCOMdpsi_targ = 0.0;

	user_opt.xT.x_m = 0.0;
	user_opt.xT.y_m = 0.0;
	user_opt.xT.z_m = 0.0;
	user_opt.heading = 0.0;
	user_opt.ds_perc = 0.2;
	user_opt.step_time = 0.25;
	user_opt.num_steps = 4;
	user_opt.bStartLeft = true;
	user_opt.step_height = 0.15;

	m_nUserMenuSelectIndex = 0;
}

Visualizer::~Visualizer() {
	// TODO Auto-generated destructor stub
}

static void window_close_callback(GLFWwindow* window)
{
	((Visualizer*)(glfwGetWindowUserPointer(window)))->Close();
}
static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	((Visualizer*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	((Visualizer*)(glfwGetWindowUserPointer(window)))->Mouse_Move(xpos, ypos);
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	((Visualizer*)(glfwGetWindowUserPointer(window)))->Mouse_Button(button, act, mods);
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	((Visualizer*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}


int Visualizer::Init(bool save_video, const char* win_title) {

	if (!glfwInit()) {
		mju_error("Could not initialize GLFW");
		return 1;
	}


//	m_FigWindow = glfwCreateWindow(m_Width, m_Height, "figures", NULL, NULL);
//	glfwMakeContextCurrent(m_FigWindow);
//	glfwSwapInterval(1);
//	mjr_defaultContext(&mj_FigCon);
//	mjr_makeContext(mj_Model, &mj_FigCon, mjFONTSCALE_100);
//
//	m_QddWindow = glfwCreateWindow(m_Width, m_Height, "Qdd Errors", NULL, NULL);
//	glfwMakeContextCurrent(m_QddWindow);
//	glfwSwapInterval(1);
//	mjr_defaultContext(&mj_QddCon);
//	mjr_makeContext(mj_Model, &mj_QddCon, mjFONTSCALE_100);

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
//	mj_Cam.azimuth = -90;

#if TRACKING

	mj_Cam.azimuth = 135;

	mj_Cam.distance = 3.0;
	mj_Cam.lookat[2] += 0.8;
	mj_Cam.type = mjCAMERA_TRACKING;
	mj_Cam.trackbodyid = 1;

#endif

	mjv_defaultOption(&mj_Opt);
//	mj_Opt.flags[10] = 1;
//	mj_Opt.flags[11] = 1;
//	mj_Opt.flags[8] = 1;
//	mj_Opt.flags[15] = 1;
//	mj_Opt.flags[2] = 1;
//	mj_Opt.flags[3] = 1;

	//int present = glfwJoystickPresent(GLFW_JOYSTICK_1);
	//printf("Joystick present %d\n", present);

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
		m_image_rgb90 = (unsigned char*)malloc(3*m_Width*m_Height);
		m_image_depth = (float*)malloc(sizeof(float)*m_Width*m_Height);

		// create output rgb file
		fp = fopen("out/temp.out", "wb");
		if( !fp )
			mju_error("Could not open rgbfile for writing");
	}

	m_bSaveVideo = save_video;

	return 0;
}

void Visualizer::GetDisturbanceForce(double* Fx, double* Fy, double* Fz)
{
	int count;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);

	(*Fx) = -axes[2]*200.0;
	(*Fy) = -axes[0]*200.0;
	(*Fz) = -axes[3]*200.0;
}

void Visualizer::GetCOMTargets(double* dx, double* dy, double* dz, double* dphi, double* dpsi)
{
	int count;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);
//	for (int i = 0; i < count; i++)
//		printf("%f\t", axes[i]);
//	printf("\n");

	dCOMdx_targ -= 0.0005*axes[1];
	dCOMdy_targ -= 0.0005*axes[0];
	dCOMdz_targ += 0.005*(axes[13]+1.0 - (axes[12]+1.0));
	dCOMdphi_targ -= 0.001*axes[3];
	dCOMdpsi_targ -= 0.001*axes[2];

	(*dx) = dCOMdx_targ;
	(*dy) = dCOMdy_targ;
	(*dz) = dCOMdz_targ;
	(*dphi) = dCOMdphi_targ;
	(*dpsi) = dCOMdpsi_targ;
}

void Visualizer::UpdateUserParams(USER_Params* user_params)
{
	static bool bNewPlan = false;
	static bool dpad_pressed = false;

	int count;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);

	user_opt.xT.x_m -= 0.05*axes[1];
	user_opt.xT.y_m -= 0.05*axes[0];
	user_opt.xT.z_m += 0.05*(axes[13]+1.0 - (axes[12]+1.0));
	user_opt.heading += 0.05*(axes[14]+1.0 - (axes[15]+1.0));

	user_opt.bNewPlan = false;

	if (axes[8] > 0.2 && !dpad_pressed)
	{
		m_nUserMenuSelectIndex++;
		m_nUserMenuSelectIndex = min(m_nUserMenuSelectIndex, m_nNumOptions-1);
		dpad_pressed = true;
	}
	else if (axes[10] > 0.2 && !dpad_pressed)
	{
		m_nUserMenuSelectIndex--;
		m_nUserMenuSelectIndex = max(m_nUserMenuSelectIndex, 0);
		dpad_pressed = true;
	}
	else if (axes[16] > 0.2 && !dpad_pressed)
	{
		if (m_nUserMenuSelectIndex == 0)
			user_opt.num_steps += 5;
		else if (m_nUserMenuSelectIndex == 1)
			user_opt.ds_perc += 0.05;
		else if (m_nUserMenuSelectIndex == 2)
			user_opt.step_time += 0.05;
		else
			user_opt.step_height += 0.025;
		dpad_pressed = true;
	}
	else if (axes[17] > 0.2 && !dpad_pressed)
	{
		if (m_nUserMenuSelectIndex == 0)
			user_opt.num_steps -= 5;
		else if (m_nUserMenuSelectIndex == 1)
			user_opt.ds_perc -= 0.05;
		else if (m_nUserMenuSelectIndex == 2)
			user_opt.step_time -= 0.05;
		else
			user_opt.step_height -= 0.025;
		dpad_pressed = true;
	}
	else if (axes[8] < -0.5 && axes[10] < -0.5 && axes[16] < -0.5 && axes[17] < -0.5)
		dpad_pressed = false;


	if ((axes[18] > 0.5 && !bNewPlan))
	{
		user_opt.bNewPlan = true;
		*user_params = user_opt;
		bNewPlan = true; //just some debouncing
	}
	if (axes[18] < -0.5)
		bNewPlan = false;



//	dCOMdx_targ -= 0.0005*axes[1];
//	dCOMdy_targ -= 0.0005*axes[0];
//	dCOMdz_targ += 0.005*(axes[13]+1.0 - (axes[12]+1.0));
//	dCOMdphi_targ -= 0.001*axes[3];
//	dCOMdpsi_targ -= 0.001*axes[2];
//
//	(*dx) = dCOMdx_targ;
//	(*dy) = dCOMdy_targ;
//	(*dz) = dCOMdz_targ;
//	(*dphi) = dCOMdphi_targ;
//	(*dpsi) = dCOMdpsi_targ;
}

void Visualizer::DrawWithPoints(mjData* data, vector<Vector3d> pts)
{
	// Return early if window is closed
	if (!m_Window)
		return;

	// Set up for rendering
	glfwMakeContextCurrent(m_Window);
	mjrRect viewport = {0, 0, 0, 0};
	glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);

	// Render scene
	mjv_updateScene(mj_Model, data, &mj_Opt, NULL, &mj_Cam, mjCAT_ALL, &mj_Scn);

	for (unsigned int i = 0; i < pts.size(); i++)
	{
		mjtNum pos[3];
		for (int j = 0; j < 3; j++)
			pos[j] = pts[i](j);
		mjtNum size[3] = {0.05,0.05,0.05};
		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_SPHERE, size, pos, NULL, NULL );
		mj_Scn.ngeom++;
	}
	mjv_addGeoms(mj_Model, data, &mj_Opt, NULL, mjCAT_DECOR, &mj_Scn);
	mjr_render(viewport, &mj_Scn, &mj_Con);

	// Show updated scene
	glfwSwapBuffers(m_Window);
	glfwPollEvents();

	if (m_bSaveVideo)
	{
		mjr_readPixels(m_image_rgb, m_image_depth, viewport, &mj_Con);
		// insert subsampled depth image in lower-left corner of rgb image
		const int NS = 3;           // depth image sub-sampling
		for( int r=0; r<m_Height; r+=NS )
		   for( int c=0; c<m_Width; c+=NS )
		   {
			  int adr = (r/NS)*m_Width + c/NS;
			  m_image_rgb[3*adr] = m_image_rgb[3*adr+1] = m_image_rgb[3*adr+2] = (unsigned char)((1.0f-m_image_depth[r*m_Width+c])*255.0f);
		   }

		 // write rgb image to file
		 fwrite(m_image_rgb, 3, m_Width*m_Height, fp);
	}

	if (bWaitForUserFeedback)
	{
		while (!bOKtoComplete)
			glfwPollEvents();
		bOKtoComplete = false;
	}
}

void Visualizer::profilerInit()
{

	mjv_defaultFigure(&figCOMxdd);
	mjv_defaultFigure(&figLeftxdd);
	mjv_defaultFigure(&figRightxdd);
	mjv_defaultFigure(&figForce);
	mjv_defaultFigure(&figTau);
	mjv_defaultFigure(&figRes);
	mjv_defaultFigure(&figPos);
	mjv_defaultFigure(&figVel);
	mjv_defaultFigure(&figQddB);
	mjv_defaultFigure(&figQddL);
	mjv_defaultFigure(&figQddR);
	mjv_defaultFigure(&figCOMXY);
	mjv_defaultFigure(&figCOMXZ);

	mjv_defaultFigure(&figMPC);


	mjv_defaultFigure(&figXDD);
	mjv_defaultFigure(&figTorque);


	//title
	strcpy(figCOMxdd.title, "COM xdd");
	strcpy(figRightxdd.title, "Right xdd");
	strcpy(figLeftxdd.title, "Left xdd");
	strcpy(figForce.title, "Force");
	strcpy(figTau.title, "Tau");
	strcpy(figRes.title, "QP Min");
	strcpy(figPos.title, "Pos");
	strcpy(figVel.title, "Vel");
	strcpy(figQddB.title, "Qdd Body");
	strcpy(figQddL.title, "Qdd Left");
	strcpy(figQddR.title, "Qdd Right");

	strcpy(figXDD.title, "XDD");
	strcpy(figTorque.title, "Torque");

	strcpy(figMPC.title, "MPC Stats");

	//x-labels
	strcpy(figCOMxdd.xlabel, "time");
	strcpy(figRightxdd.xlabel, "time");
	strcpy(figLeftxdd.xlabel, "time");
	strcpy(figForce.xlabel, "time");
	strcpy(figTau.xlabel, "time");
	strcpy(figRes.xlabel, "time");
	strcpy(figPos.xlabel, "time");
	strcpy(figVel.xlabel, "time");
	strcpy(figQddB.xlabel, "time");
	strcpy(figQddL.xlabel, "time");
	strcpy(figQddR.xlabel, "time");
	strcpy(figMPC.xlabel, "time");
	strcpy(figCOMXY.xlabel, "X");
	strcpy(figCOMXZ.xlabel, "X");
	strcpy(figTorque.xlabel, "time");
	strcpy(figXDD.xlabel, "time");

	//y-tick number format
	strcpy(figCOMxdd.yformat, "%.4f");
	strcpy(figLeftxdd.yformat, "%.4f");
	strcpy(figRightxdd.yformat, "%.4f");
	strcpy(figForce.yformat, "%.4f");
	strcpy(figTau.yformat, "%.4f");
	strcpy(figRes.yformat, "%.4f");
	strcpy(figPos.yformat, "%.4f");
	strcpy(figVel.yformat, "%.4f");
	strcpy(figQddB.yformat, "%.4f");
	strcpy(figQddL.yformat, "%.4f");
	strcpy(figQddR.yformat, "%.4f");
	strcpy(figCOMXZ.yformat, "%.4f");
	strcpy(figCOMXY.yformat, "%.4f");
	strcpy(figMPC.yformat, "%.2f");
	strcpy(figXDD.yformat, "%.2f");
	strcpy(figTorque.yformat, "%.2f");

	//labels
	if (DOF == 2)
	{
		strcpy(figCOMxdd.linename[0], "xdd");
		strcpy(figCOMxdd.linename[1], "zdd");
		strcpy(figCOMxdd.linename[2], "xdd_t");
		strcpy(figCOMxdd.linename[3], "zdd_t");
		strcpy(figCOMxdd.linename[4], "xdd_e");
		strcpy(figCOMxdd.linename[5], "zdd_e");

		strcpy(figLeftxdd.linename[0], "xdd");
		strcpy(figLeftxdd.linename[1], "zdd");
		strcpy(figLeftxdd.linename[2], "xdd_t");
		strcpy(figLeftxdd.linename[3], "zdd_t");
		strcpy(figLeftxdd.linename[4], "xdd_e");
		strcpy(figLeftxdd.linename[5], "zdd_e");

		strcpy(figRightxdd.linename[0], "xdd");
		strcpy(figRightxdd.linename[1], "zdd");
		strcpy(figRightxdd.linename[2], "xdd_t");
		strcpy(figRightxdd.linename[3], "zdd_t");
		strcpy(figRightxdd.linename[4], "xdd_e");
		strcpy(figRightxdd.linename[5], "zdd_e");

		figCOMxdd.linewidth[2] *= 2.0;
		figCOMxdd.linewidth[3] *= 2.0;
		figLeftxdd.linewidth[2] *= 2.0;
		figLeftxdd.linewidth[3] *= 2.0;
		figRightxdd.linewidth[2] *= 2.0;
		figRightxdd.linewidth[3] *= 2.0;
	}
	else
	{
		strcpy(figCOMxdd.linename[0], "xdd");
		strcpy(figCOMxdd.linename[1], "ydd");
		strcpy(figCOMxdd.linename[2], "zdd");
		strcpy(figCOMxdd.linename[3], "xdd_t");
		strcpy(figCOMxdd.linename[4], "ydd_t");
		strcpy(figCOMxdd.linename[5], "zdd_t");
		strcpy(figCOMxdd.linename[6], "xdd_e");
		strcpy(figCOMxdd.linename[7], "ydd_e");
		strcpy(figCOMxdd.linename[8], "zdd_e");

		strcpy(figLeftxdd.linename[0], "xdd");
		strcpy(figLeftxdd.linename[1], "ydd");
		strcpy(figLeftxdd.linename[2], "zdd");
		strcpy(figLeftxdd.linename[3], "xdd_t");
		strcpy(figLeftxdd.linename[4], "ydd_t");
		strcpy(figLeftxdd.linename[5], "zdd_t");
		strcpy(figLeftxdd.linename[6], "xdd_e");
		strcpy(figLeftxdd.linename[7], "ydd_e");
		strcpy(figLeftxdd.linename[8], "zdd_e");

		strcpy(figRightxdd.linename[0], "xdd");
		strcpy(figRightxdd.linename[1], "ydd");
		strcpy(figRightxdd.linename[2], "zdd");
		strcpy(figRightxdd.linename[3], "xdd_t");
		strcpy(figRightxdd.linename[4], "ydd_t");
		strcpy(figRightxdd.linename[5], "zdd_t");
		strcpy(figRightxdd.linename[6], "xdd_e");
		strcpy(figRightxdd.linename[7], "ydd_e");
		strcpy(figRightxdd.linename[8], "zdd_e");

		figCOMxdd.linewidth[3] *= 2.0;
		figCOMxdd.linewidth[4] *= 2.0;
		figCOMxdd.linewidth[5] *= 2.0;
		figLeftxdd.linewidth[3] *= 2.0;
		figLeftxdd.linewidth[4] *= 2.0;
		figLeftxdd.linewidth[5] *= 2.0;
		figRightxdd.linewidth[3] *= 2.0;
		figRightxdd.linewidth[4] *= 2.0;
		figRightxdd.linewidth[5] *= 2.0;
	}
	strcpy(figForce.linename[0], "flx");
	strcpy(figForce.linename[1], "flz");
	strcpy(figForce.linename[2], "frx");
	strcpy(figForce.linename[3], "frz");

	strcpy(figTau.linename[0], "hipl");
	strcpy(figTau.linename[1], "legl");
	strcpy(figTau.linename[2], "hipr");
	strcpy(figTau.linename[3], "legr");

	strcpy(figPos.linename[0], "comx_t");
	strcpy(figPos.linename[1], "comz_t");
	strcpy(figPos.linename[2], "comx");
	strcpy(figPos.linename[3], "comz");
	strcpy(figPos.linename[4], "fx_t");
	strcpy(figPos.linename[5], "fz_t");
	strcpy(figPos.linename[6], "fx");
	strcpy(figPos.linename[7], "fz");

	strcpy(figVel.linename[0], "comx_t");
	strcpy(figVel.linename[1], "comz_t");
	strcpy(figVel.linename[2], "comx");
	strcpy(figVel.linename[3], "comz");
	strcpy(figVel.linename[4], "fx_t");
	strcpy(figVel.linename[5], "fz_t");
	strcpy(figVel.linename[6], "fx");
	strcpy(figVel.linename[7], "fz");

	strcpy(figQddB.linename[0], "x");
	strcpy(figQddB.linename[1], "z");
	strcpy(figQddB.linename[2], "t");

	strcpy(figQddL.linename[0], "h");
	strcpy(figQddL.linename[1], "k");
	strcpy(figQddL.linename[2], "a");
	strcpy(figQddL.linename[3], "t");
	strcpy(figQddL.linename[4], "c");

	strcpy(figQddR.linename[0], "h");
	strcpy(figQddR.linename[1], "k");
	strcpy(figQddR.linename[2], "a");
	strcpy(figQddR.linename[3], "t");
	strcpy(figQddR.linename[4], "c");




	figCOMxdd.gridsize[0] = 5;
	figCOMxdd.gridsize[1] = 5;
	figLeftxdd.gridsize[0] = 5;
	figLeftxdd.gridsize[1] = 5;
	figRightxdd.gridsize[0] = 5;
	figRightxdd.gridsize[1] = 5;
	figForce.gridsize[0] = 5;
	figForce.gridsize[1] = 5;
	figTau.gridsize[0] = 5;
	figTau.gridsize[1] = 5;
	figRes.gridsize[0] = 5;
	figRes.gridsize[1] = 5;
	figPos.gridsize[0] = 5;
	figPos.gridsize[1] = 5;
	figVel.gridsize[0] = 5;
	figVel.gridsize[1] = 5;
	figQddB.gridsize[0] = 5;
	figQddB.gridsize[1] = 5;
	figQddL.gridsize[0] = 5;
	figQddL.gridsize[1] = 5;
	figQddR.gridsize[0] = 5;
	figQddR.gridsize[1] = 5;
	figCOMXY.gridsize[0] = 5;
	figCOMXY.gridsize[1] = 5;
	figCOMXZ.gridsize[0] = 5;
	figCOMXZ.gridsize[1] = 5;
	figXDD.gridsize[0] = 5;
	figXDD.gridsize[1] = 5;
	figTorque.gridsize[0] = 5;
	figTorque.gridsize[1] = 5;

	//ranges
	figCOMxdd.range[0][0] = -20;
	figCOMxdd.range[0][1] = 0;
	figCOMxdd.range[1][0] = 0;
	figCOMxdd.range[1][1] = 5000;
	figLeftxdd.range[0][0] = -20;
	figLeftxdd.range[0][1] = 0;
	figLeftxdd.range[1][0] = 0;
	figLeftxdd.range[1][1] = 5000;
	figRightxdd.range[0][0] = -20;
	figRightxdd.range[0][1] = 0;
	figRightxdd.range[1][0] = 0;
	figRightxdd.range[1][1] = 5000;
	figForce.range[0][0] = -200;
	figForce.range[0][1] = 0;
	figForce.range[1][0] = 0;
	figForce.range[1][1] = 5000;
	figTau.range[0][0] = -20;
	figTau.range[0][1] = 0;
	figTau.range[1][0] = 0;
	figTau.range[1][1] = 5000;
	figRes.range[0][0] = -20;
	figRes.range[0][1] = 0;
	figRes.range[1][0] = 0;
	figRes.range[1][1] = 5000;
	figPos.range[0][0] = -200;
	figPos.range[0][1] = 0;
	figPos.range[1][0] = 0;
	figPos.range[1][1] = 5000;
	figVel.range[0][0] = -200;
	figVel.range[0][1] = 0;
	figVel.range[1][0] = 0;
	figVel.range[1][1] = 5000;
	figQddB.range[0][0] = -20;
	figQddB.range[0][1] = 0;
	figQddB.range[1][0] = 0;
	figQddB.range[1][1] = 5000;
	figQddL.range[0][0] = -20;
	figQddL.range[0][1] = 0;
	figQddL.range[1][0] = 0;
	figQddL.range[1][1] = 5000;
	figQddR.range[0][0] = -20;
	figQddR.range[0][1] = 0;
	figQddR.range[1][0] = 0;
	figQddR.range[1][1] = 5000;
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

	figTorque.range[0][0] = -200;
	figTorque.range[0][1] = 0;
	figXDD.range[1][0] = 0;
	figXDD.range[1][1] = 100;



	//hide history plots
	for (int n = 0; n < 10; n++)
	{
		for (int i = 0; i < mjMAXLINEPNT; i++)
		{
			figCOMxdd.linedata[n][2*i] = (float)-i;
			figLeftxdd.linedata[n][2*i] = (float)-i;
			figRightxdd.linedata[n][2*i] = (float)-i;
			figForce.linedata[n][2*i] = (float)-i;
			figTau.linedata[n][2*i] = (float)-i;
			figRes.linedata[n][2*i] = (float)-i;
			figVel.linedata[n][2*i] = (float)-i;
			figPos.linedata[n][2*i] = (float)-i;
			figQddB.linedata[n][2*i] = (float)-i;
			figQddL.linedata[n][2*i] = (float)-i;
			figQddR.linedata[n][2*i] = (float)-i;
			figMPC.linedata[n][2*i] = (float)-i;
			figTorque.linedata[n][2*i] = (float)-i;
			figXDD.linedata[n][2*i] = (float)-i;
		}
	}

}

void Visualizer::profilerUpdateXDD(Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd_target, Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd_e)
{
	int pnt = mjMIN(21, figCOMxdd.linepnt[0]+1);
	double COMdata[3*DOF];
	for (int i = 0; i < DOF; i++)
	{
		COMdata[i] = xdd(i,0);
		COMdata[i + DOF] = xdd_target(i,0);
		COMdata[i + 2*DOF] = xdd_e(i,0);
	}

	float dMax = -10000;
	float dMin = 10000;
	for (int n = 0; n < 3*DOF; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figCOMxdd.linedata[n][2*i+1] = figCOMxdd.linedata[n][2*i-1];
			dMax = max(figCOMxdd.linedata[n][2*i+1], dMax);
			dMin = min(figCOMxdd.linedata[n][2*i+1], dMin);
		}
		figCOMxdd.linepnt[n] = pnt;
		figCOMxdd.linedata[n][1] = COMdata[n];
	}
	figCOMxdd.range[1][0] = dMin - 0.1*fabs(dMin);
	figCOMxdd.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(21, figLeftxdd.linepnt[0]+1);
	double Leftdata[3*DOF];
	for (int i = 0; i < DOF; i++)
	{
		Leftdata[i] = xdd(DOF+i,0);
		Leftdata[i + DOF] = xdd_target(DOF+i,0);
		Leftdata[i + 2*DOF] = xdd_e(DOF+i,0);
	}
	for (int n = 0; n < 3*DOF; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figLeftxdd.linedata[n][2*i+1] = figLeftxdd.linedata[n][2*i-1];
			dMax = max(figLeftxdd.linedata[n][2*i+1], dMax);
			dMin = min(figLeftxdd.linedata[n][2*i+1], dMin);
		}
		figLeftxdd.linepnt[n] = pnt;
		figLeftxdd.linedata[n][1] = Leftdata[n];
	}
	figLeftxdd.range[1][0] = dMin - 0.1*fabs(dMin);
	figLeftxdd.range[1][1] = dMax + 0.1*fabs(dMax);

	if (XDD_TARGETS < 3)
		return;

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(21, figRightxdd.linepnt[0]+1);
	double Rightdata[3*DOF];
	for (int i = 0; i < DOF; i++)
	{
		Rightdata[i] = xdd(2*DOF+i,0);
		Rightdata[i + DOF] = xdd_target(2*DOF+i,0);
		Rightdata[i + 2*DOF] = xdd_e(2*DOF+i,0);
	}
	for (int n = 0; n < 2*DOF; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figRightxdd.linedata[n][2*i+1] = figRightxdd.linedata[n][2*i-1];
			dMax = max(figRightxdd.linedata[n][2*i+1], dMax);
			dMin = min(figRightxdd.linedata[n][2*i+1], dMin);
		}
		figRightxdd.linepnt[n] = pnt;
		figRightxdd.linedata[n][1] = Rightdata[n];
	}
	figRightxdd.range[1][0] = dMin - 0.1*fabs(dMin);
	figRightxdd.range[1][1] = dMax + 0.1*fabs(dMax);
}

void Visualizer::profilerUpdateForces( Matrix<double, nCON*DOF, 1> f, Matrix<double, nU, 1> u)
{
	float dMax = -10000;
	float dMin = 10000;
	int pnt = mjMIN(201, figForce.linepnt[0]+1);
	for (int n = 0; n < nCON*DOF; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figForce.linedata[n][2*i+1] = figForce.linedata[n][2*i-1];
			dMax = max(figForce.linedata[n][2*i+1], dMax);
			dMin = min(figForce.linedata[n][2*i+1], dMin);
		}
		figForce.linepnt[n] = pnt;
		figForce.linedata[n][1] = f(n,0);
	}
	figForce.range[1][0] = dMin - 0.1*fabs(dMin);
	figForce.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(21, figTau.linepnt[0]+1);
	for (int n = 0; n < nU; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figTau.linedata[n][2*i+1] = figTau.linedata[n][2*i-1];
			dMax = max(figTau.linedata[n][2*i+1], dMax);
			dMin = min(figTau.linedata[n][2*i+1], dMin);
		}
		figTau.linepnt[n] = pnt;
		figTau.linedata[n][1] = u(n,0);
	}
	figTau.range[1][0] = dMin - 0.1*fabs(dMin);
	figTau.range[1][1] = dMax + 0.1*fabs(dMax);
}

void Visualizer::profilerUpdateQPError(double res)
{
	float dMax = -10000;
	float dMin = 10000;
	int pnt = mjMIN(21, figRes.linepnt[0]+1);
	for (int n = 0; n < 1; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figRes.linedata[n][2*i+1] = figRes.linedata[n][2*i-1];

			dMax = max(figRes.linedata[n][2*i+1], dMax);
			dMin = min(figRes.linedata[n][2*i+1], dMin);
		}
		figRes.linepnt[n] = pnt;
		figRes.linedata[n][1] = res;
	}
	figRes.range[1][0] = dMin - 0.1*fabs(dMin);
	figRes.range[1][1] = dMax + 0.1*fabs(dMax);
}

void Visualizer::profilerUpdateTracking(Matrix<double, 16, 1> tracking)
{
	float dMax = -10000;
	float dMin = 10000;
	int pnt = mjMIN(201, figPos.linepnt[0]+1);
	int idx = 0;
	for (int n = 0; n < 8; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figPos.linedata[n][2*i+1] = figPos.linedata[n][2*i-1];
			dMax = max(figPos.linedata[n][2*i+1], dMax);
			dMin = min(figPos.linedata[n][2*i+1], dMin);
		}
		figPos.linepnt[n] = pnt;
		idx = n/2;
		figPos.linedata[n][1] = tracking(idx*4+n%2,0);

	}
	figPos.range[1][0] = dMin - 0.1*fabs(dMin);
	figPos.range[1][1] = dMax + 0.1*fabs(dMax);


	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(201, figVel.linepnt[0]+1);
	for (int n = 0; n < 8; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figVel.linedata[n][2*i+1] = figVel.linedata[n][2*i-1];
			dMax = max(figVel.linedata[n][2*i+1], dMax);
			dMin = min(figVel.linedata[n][2*i+1], dMin);
		}
		figVel.linepnt[n] = pnt;
		idx = n/2;
		figVel.linedata[n][1] = tracking(2+idx*4+n%2,0);
	}
	figVel.range[1][0] = dMin - 0.1*fabs(dMin);
	figVel.range[1][1] = dMax + 0.1*fabs(dMax);
}

void Visualizer::profilerUpdateCOP(Matrix<double, DOF, 1> COM, Matrix<double, DOF, 1> COM_target, Matrix<double, DOF, nCON> foot, bool* bContact)
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
		if (bContact[i])
		{
			figCOMXY.linedata[0][pnt*2] = foot(0,i);
			figCOMXY.linedata[0][pnt*2+1] = foot(1,i);
			figCOMXZ.linedata[0][pnt*2] = foot(0,i);
			figCOMXZ.linedata[0][pnt*2+1] = foot(2,i);


			dMaxX = max(figCOMXY.linedata[0][pnt*2],dMaxX);
			dMinX = min(figCOMXY.linedata[0][pnt*2],dMinX);
			dMaxY = max(figCOMXY.linedata[0][pnt*2+1],dMaxY);
			dMinY = min(figCOMXY.linedata[0][pnt*2+1],dMinY);
			dMaxZ = max(figCOMXZ.linedata[0][pnt*2+1],dMaxZ);
			dMinZ = min(figCOMXZ.linedata[0][pnt*2+1],dMinZ);

			pnt++;
		}
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

	figCOMXY.linedata[1][0] = COM(0,0);
	figCOMXY.linedata[1][1] = COM(1,0);
	figCOMXZ.linedata[1][0] = COM(0,0);
	figCOMXZ.linedata[1][1] = COM(2,0);

	figCOMXY.linedata[2][0] = COM_target(0,0);
	figCOMXY.linedata[2][1] = COM_target(1,0);
	figCOMXZ.linedata[2][0] = COM_target(0,0);
	figCOMXZ.linedata[2][1] = COM_target(2,0);

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

void Visualizer::profilerUpdateQDD(Matrix<double, nQ, 1> QddE)
{
	float dMax = -10000;
	float dMin = 10000;
	int pnt = mjMIN(21, figQddB.linepnt[0]+1);
	for (int n = 0; n < 3; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figQddB.linedata[n][2*i+1] = figQddB.linedata[n][2*i-1];
			dMax = max(figQddB.linedata[n][2*i+1], dMax);
			dMin = min(figQddB.linedata[n][2*i+1], dMin);
		}
		figQddB.linepnt[n] = pnt;
		figQddB.linedata[n][1] = QddE(n,0);
	}
	figQddB.range[1][0] = dMin - 0.1*fabs(dMin);
	figQddB.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(21, figQddL.linepnt[0]+1);
	for (int n = 0; n < nQ; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figQddL.linedata[n][2*i+1] = figQddL.linedata[n][2*i-1];
			dMax = max(figQddL.linedata[n][2*i+1], dMax);
			dMin = min(figQddL.linedata[n][2*i+1], dMin);
		}
		figQddL.linepnt[n] = pnt;
		figQddL.linedata[n][1] = QddE(n,0);
	}
	figQddL.range[1][0] = dMin - 0.1*fabs(dMin);
	figQddL.range[1][1] = dMax + 0.1*fabs(dMax);
return;
	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(21, figQddR.linepnt[0]+1);
	for (int n = 0; n < 5; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figQddR.linedata[n][2*i+1] = figQddR.linedata[n][2*i-1];
			dMax = max(figQddR.linedata[n][2*i+1], dMax);
			dMin = min(figQddR.linedata[n][2*i+1], dMin);
		}
		figQddR.linepnt[n] = pnt;
		figQddR.linedata[n][1] = QddE(8+n,0);
	}
	figQddR.range[1][0] = dMin - 0.1*fabs(dMin);
	figQddR.range[1][1] = dMax + 0.1*fabs(dMax);
}

void Visualizer::profilerShowXZ(mjrRect vp)
{
	double div = 4.0;
#if VIEW90
	div = 2.0;
#endif
	vp.width /= div;
	vp.height /= 4;
	vp.bottom += 3*vp.height;
	mjr_figure(vp, &figCOMXZ, &mj_Con);
}

void Visualizer::profilerShowXY(mjrRect vp)
{
	double div = 4.0;
#if VIEW90
	div = 2.0;
#endif
	vp.left = vp.width - vp.width/div;
	vp.width /= div;
	vp.height /= 4;
	vp.bottom += 3*vp.height;
	mjr_figure(vp, &figCOMXY, &mj_Con);
}

void Visualizer::profilerShow(mjrRect vp)
{
	mjr_render(vp, &mj_Scn, &mj_Con);
	vp.width /= 4;
	vp.height /= 4;
	mjr_figure(vp, &figCOMxdd, &mj_Con);
	vp.bottom += vp.height;
	mjr_figure(vp, &figLeftxdd, &mj_Con);
	vp.bottom += vp.height;
	mjr_figure(vp, &figRightxdd, &mj_Con);
	vp.bottom += vp.height;
	mjr_figure(vp, &figForce, &mj_Con);
	vp.left += vp.width;
	mjr_figure(vp, &figTau, &mj_Con);
	vp.left += vp.width;
	//vp.bottom -= 3.0*vp.height;
	mjr_figure(vp, &figRes, &mj_Con);
	vp.left += vp.width;
	vp.bottom -= vp.height;

	glfwSwapBuffers(m_Window);
	glfwPollEvents();

	glfwMakeContextCurrent(m_FigWindow);

	mjrRect viewport = {0, 0, 0, 0};
	glfwGetFramebufferSize(m_FigWindow, &viewport.width, &viewport.height);
	//mjr_render(viewport, &mj_Scn, &mj_Con);

	viewport.height/=2;
	mjr_figure(viewport, &figPos, &mj_FigCon);
	viewport.bottom += viewport.height;
	mjr_figure(viewport, &figVel, &mj_FigCon);
	glfwSwapBuffers(m_FigWindow);
	glfwPollEvents();

	glfwMakeContextCurrent(m_QddWindow);

	glfwGetFramebufferSize(m_QddWindow, &viewport.width, &viewport.height);
	//mjr_render(viewport, &mj_Scn, &mj_Con);

	viewport.bottom = 0;
	viewport.height/=3;
	mjr_figure(viewport, &figQddB, &mj_QddCon);
	viewport.bottom += viewport.height;
	mjr_figure(viewport, &figQddL, &mj_QddCon);
	viewport.bottom += viewport.height;
	mjr_figure(viewport, &figQddR, &mj_QddCon);
	glfwSwapBuffers(m_QddWindow);
	glfwPollEvents();


//	glfwMakeContextCurrent(m_Window);

}

void Visualizer::DrawTrajPoints(mjData* data) {
	for (int i = 0; i < m_nNumPoints; i++)
	{
		mjtNum pos[3];
		pos[0] = traj_pts[i](0) + traj_pts[i](8);
		pos[1] = 0.0;
		pos[2] = traj_pts[i](1) + traj_pts[i](9);

		double sphereSize = 0.01;
		float rgba[4] = {0.5, 0.5, 0.5, 1.0};

		mjtNum size[3] = {sphereSize,sphereSize,sphereSize};

		if (i == m_nTargetIndex)
		{
			rgba[0] = 1.0;
			size[1]= 0.5;
		}
		else
		{
			rgba[0] = 0.5;
			size[1] = sphereSize;
		}


		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
		mj_Scn.ngeom++;

		pos[0] = traj_pts[i](2) + traj_pts[i](8); pos[2] = traj_pts[i](3) + traj_pts[i](9);
		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
		mj_Scn.ngeom++;
	}
	mjv_addGeoms(mj_Model, data, &mj_Opt, NULL, mjCAT_DECOR, &mj_Scn);
}

void Visualizer::DrawMPCPlan(mjData* data) {
	for (unsigned int i = 0; i < plan_com_traj.size(); i++)
	{
		if (i % 5)
			continue;

		mjtNum pos[3];
		pos[0] = plan_com_traj[i].x_m;
		pos[1] = plan_com_traj[i].y_m;
		pos[2] = plan_com_traj[i].z_m;

		double sphereSize = 0.01;

		float rgba[4] = {1.0, 0.1, 0.1, 1.0};

		if (m_bSuccess)
		{
			rgba[0] = 0.1;
			rgba[1] = 1.0;
		}
		mjtNum size[3] = {sphereSize,sphereSize,sphereSize*10};

		mjtNum rot_mat[9];
		mju_zero(rot_mat,9);
		rot_mat[6] = 1.0;
		double angle = plan_com_traj[i].a_rad;
		rot_mat[2] = cos(angle);
		rot_mat[1] = sin(angle);
		rot_mat[5] = sin(angle);
		rot_mat[4] = -cos(angle);
		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ARROW, size, pos, rot_mat, rgba);

//		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
		mj_Scn.ngeom++;
	}

	for (unsigned int i = 0; i < plan_cop_traj.size(); i++)
	{
		mjtNum pos[3];
		pos[0] = plan_cop_traj[i].x_m;
		pos[1] = plan_cop_traj[i].y_m;
		pos[2] = 0.0;

		double sphereSize = 0.01;
		float rgba[4] = {0.0, 0.0, 0.0, 1.0};

		mjtNum size[3] = {sphereSize,sphereSize,sphereSize};

		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
		mj_Scn.ngeom++;
	}

	for (unsigned int i = 0; i < plan_foot_pos.size(); i++)
	{
		mjtNum pos[3];
		pos[0] = plan_foot_pos[i].lb.x_m;
		pos[1] = plan_foot_pos[i].lb.y_m;
		pos[2] = 0.0;

		double sphereSize = 0.01;
		float rgba[4] = {0.1, 1.0, 1.0, 1.0};

		rgba[2] *= double(i % 2);

		if (i < 8)
		{
			rgba[0] = 1.0;
			rgba[1] = rgba[2] = 0.1;
		}

		mjtNum size[3] = {sphereSize,sphereSize,sphereSize};

		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
		mj_Scn.ngeom++;

		pos[0] = plan_foot_pos[i].ub.x_m;
		pos[1] = plan_foot_pos[i].ub.y_m;
		mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
		mj_Scn.ngeom++;
	}

	//add user opt geometry
	float rgba[4] = {1.0, 1.0, 1.0, 1.0};
	mjtNum size[3] = {0.01, 0.01, 0.1};
	mjtNum pos[3];
	pos[0] = plan_com_traj[0].x_m + user_opt.xT.x_m;
	pos[1] = plan_com_traj[0].y_m + user_opt.xT.y_m;
	pos[2] = plan_com_traj[0].z_m + user_opt.xT.z_m;
	mjtNum rot_mat[9];
	mju_zero(rot_mat,9);
	rot_mat[6] = 1.0;
	rot_mat[2] = cos(user_opt.heading);
	rot_mat[1] = sin(user_opt.heading);
	rot_mat[5] = sin(user_opt.heading);
	rot_mat[4] = -cos(user_opt.heading);
	mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ARROW, size, pos, rot_mat, rgba);
	mj_Scn.ngeom++;
	mjv_addGeoms(mj_Model, data, &mj_Opt, NULL, mjCAT_DECOR, &mj_Scn);

	mjrRect viewport = {0, 0, 0, 0};
	glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);


}

void Visualizer::profilerShowMPC(mjrRect viewport)
{
	mjrRect smallrect = viewport;
	smallrect.width = viewport.width - viewport.width/5;
	char user_info[1000] = "";
	sprintf(user_info, "%.2f\n%.2f\n%.2f\n%d", user_opt.step_height, user_opt.step_time, user_opt.ds_perc, user_opt.num_steps);
	mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
	      "Height\nStep Time\nDS_perc\nNum Steps:", user_info, &mj_Con);

	smallrect.left = 5;
	smallrect.height = 22;
	smallrect.bottom += 5 + smallrect.height*m_nUserMenuSelectIndex;
	smallrect.width = 127;
	mjr_rectangle(smallrect, 1.0, 0.0, 0.0, 0.1);

	viewport.width /= 4;
	viewport.left += 3*viewport.width;
	viewport.height /= 4;
	mjr_figure(viewport, &figMPC, &mj_Con);
}

void Visualizer::SetMPCDisplay(int run_time) {
	float maxT = -1000;
	float minT = 1000;
	int pnt = mjMIN(201, figMPC.linepnt[0]+1);
	for (int n = 0; n < 1; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figMPC.linedata[n][2*i+1] = figMPC.linedata[n][2*i-1];
			maxT = max(figMPC.linedata[n][2*i+1], maxT);
			minT = min(figMPC.linedata[n][2*i+1], minT);
		}
		figMPC.linepnt[n] = pnt;
		figMPC.linedata[n][1] = run_time;
		figMPC.range[1][0] = minT;
		figMPC.range[1][1] = maxT + 5;
	}


}

bool Visualizer::DrawPinned(mjData* data, telemetry_t t)
{
	if (!m_Window)
		return false;

	float dMax = -10000;
	float dMin = 10000;
	int pnt = mjMIN(201, figXDD.linepnt[0]+1);
	for (int n = 0; n < XDD_TARGETS*DOF; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figXDD.linedata[n][2*i+1] = figXDD.linedata[n][2*i-1];
			dMax = max(figXDD.linedata[n][2*i+1], dMax);
			dMin = min(figXDD.linedata[n][2*i+1], dMin);
		}
		figXDD.linepnt[n] = pnt;
		figXDD.linedata[n][1] = t.accels[n];
	}
	figXDD.range[1][0] = dMin - 0.1*fabs(dMin);
	figXDD.range[1][1] = dMax + 0.1*fabs(dMax);

	dMax = -10000;
	dMin = 10000;
	pnt = mjMIN(201, figTorque.linepnt[0]+1);
	for (int n = 0; n < nU; n++)
	{
		for (int i = pnt - 1; i > 0; i--)
		{
			figTorque.linedata[n][2*i+1] = figTorque.linedata[n][2*i-1];
			dMax = max(figTorque.linedata[n][2*i+1], dMax);
			dMin = min(figTorque.linedata[n][2*i+1], dMin);
		}
		figTorque.linepnt[n] = pnt;
		figTorque.linedata[n][1] = t.torques[n];
	}
	figTorque.range[1][0] = dMin - 0.1*fabs(dMin);
	figTorque.range[1][1] = dMax + 0.1*fabs(dMax);



	static int frame = 0;
	timespec ts;
	static timespec tf;
	static bool bFirstTime = true;

    clock_gettime(CLOCK_REALTIME, &ts);
    double freq = 1e9/double(diff(tf,ts).tv_nsec);
    if (freq > max_frame_rate && !bFirstTime)
    	return false;
    bFirstTime = false;
    clock_gettime(CLOCK_REALTIME, &tf);


//	if (frame++ <= 34 && !bWaitForUserFeedback && !m_bSaveVideo)
//		return false;
	frame = 0;
	bool doOnce = true;
	mjrRect viewport = {0, 0, 0, 0};

	while ((bWaitForUserFeedback && !bOKtoComplete) || doOnce)
	{
		doOnce = false;
		// Set up for rendering
		glfwMakeContextCurrent(m_Window);
		glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);

		mjv_updateScene(mj_Model, data, &mj_Opt, NULL, &mj_Cam, mjCAT_ALL, &mj_Scn);

		for (unsigned int i = 0; i < XDD_TARGETS; i++)
		{
			mjtNum pos[3];
			pos[0] = t.targ_pos[i*DOF];
			pos[1] = t.targ_pos[i*DOF+1];
			pos[2] = t.targ_pos[i*DOF+2];

			double sphereSize = 0.01;
			float rgba[4] = {1.0, 0.0, 0.0, 1.0};

			mjtNum size[3] = {sphereSize,sphereSize,sphereSize};

			mjv_initGeom(&(mj_Scn.geoms[mj_Scn.ngeom]), mjGEOM_ELLIPSOID, size, pos, NULL, rgba );
			mj_Scn.ngeom++;
		}

		glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);


		mjr_render(viewport, &mj_Scn, &mj_Con);

		double div = 4.0;
		viewport.width /= div;
		viewport.height /= 4;
		viewport.bottom += 3*viewport.height;
		mjr_figure(viewport, &figTorque, &mj_Con);

		viewport.left = 3.0*viewport.width;
		mjr_figure(viewport, &figXDD, &mj_Con);


		mjrRect smallrect = viewport;
        	smallrect.width = viewport.width - viewport.width/5;
		int calib = 0;
		int power = 0;
		if (t.op_state & OpState_Calibrated)
			calib = 1;
		if (t.op_state & OpState_MotorPower)
			power = 1;

		char user_info[100] = "";
		sprintf(user_info, "%d\n%d", calib, power);
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect,
				"Calibrated:\nMotor Power:", user_info, &mj_Con);

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

bool Visualizer::Draw(mjData* data) {
	// Return early if window is closed
	if (!m_Window)
		return false;
	static int frame = 0;
	timespec ts;
	static timespec tf;
	static bool bFirstTime = true;

    clock_gettime(CLOCK_REALTIME, &ts);
    double freq = 1e9/double(diff(tf,ts).tv_nsec);
    if (freq > max_frame_rate && !bFirstTime)
    	return false;
    bFirstTime = false;
    clock_gettime(CLOCK_REALTIME, &tf);


//	if (frame++ <= 34 && !bWaitForUserFeedback && !m_bSaveVideo)
//		return false;
	frame = 0;
	bool doOnce = true;
	mjrRect viewport = {0, 0, 0, 0};

	while ((bWaitForUserFeedback && !bOKtoComplete) || doOnce)
	{
		doOnce = false;
		// Set up for rendering
		glfwMakeContextCurrent(m_Window);
		glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);
#if VIEW90
		viewport.width /= 2.0;
#endif
		mjv_updateScene(mj_Model, data, &mj_Opt, NULL, &mj_Cam, mjCAT_ALL, &mj_Scn);

		if (m_bTrajInit)
			DrawTrajPoints(data);

		if (m_bPlanInit)
			DrawMPCPlan(data);


		mjr_render(viewport, &mj_Scn, &mj_Con);
		profilerShowXZ(viewport);
		profilerShowMPC(viewport);

#if VIEW90
		glfwSwapBuffers(m_Window);
		viewport.width *= 2.0;
		mjr_readPixels(m_image_rgb90, m_image_depth, viewport, &mj_Con);
		viewport.width /= 2.0;
		viewport.left += viewport.width;
		mj_Cam.azimuth += 90;
		mjv_updateScene(mj_Model, data, &mj_Opt, NULL, &mj_Cam, mjCAT_ALL, &mj_Scn);
		mj_Cam.azimuth -= 90;
		mjr_render(viewport, &mj_Scn, &mj_Con);
#endif
		profilerShowXY(viewport);

#if PROFILER
		profilerShow(viewport);
#endif
		// Show updated scene
		glfwSwapBuffers(m_Window);
		glfwPollEvents();

	}
	bOKtoComplete = false;


//	for (int i = 0; i < count; i++)
//		printf("Axis: %d\t%f\n", i, axes[i]);

	if (m_bSaveVideo)
	{
		glfwMakeContextCurrent(m_Window);
		glfwGetFramebufferSize(m_Window, &viewport.width, &viewport.height);
		viewport.left = 0;

		mjr_readPixels(m_image_rgb, m_image_depth, viewport, &mj_Con);

#if VIEW90
		// insert subsampled depth image in lower-left corner of rgb image
		for( int r=0; r<m_Height; r++ )
		   for( int c=0; c<m_Width/2; c++ )
		   {
			  int adr1 = r*m_Width + c;
			  int adr2 = r*m_Width + m_Width/2 + c;
			  m_image_rgb[3*adr1] = m_image_rgb90[3*adr1];
			  m_image_rgb[3*adr1+1] = m_image_rgb90[3*adr1+1];
			  m_image_rgb[3*adr1+2] = m_image_rgb90[3*adr1+2];
		   }
#endif
		 // write rgb image to file
		 fwrite(m_image_rgb, 3, m_Width*m_Height, fp);
	}

	if (m_bReset)
	{
		m_bReset = false;
		return true;
	}

	return false;
}

// mouse button
void Visualizer::Mouse_Button(int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(m_Window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(m_Window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(m_Window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(m_Window, &cursor_lastx, &cursor_lasty);
}



// mouse move
void Visualizer::Mouse_Move(double xpos, double ypos)
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

void Visualizer::Scroll(double xoffset, double yoffset)
{
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(mj_Model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mj_Scn, &mj_Cam);
//    printf("scroll callback %f, %f\n", xoffset, yoffset);
}

// keyboard
void Visualizer::Keyboard(int key, int scancode, int act, int mods)
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

void Visualizer::Close() {
    // Free mujoco objects
    mjv_freeScene(&mj_Scn);
    mjr_freeContext(&mj_Con);

    // Close window
    glfwDestroyWindow(m_Window);
    m_Window = NULL;
}
