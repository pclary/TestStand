#include <stdio.h>
#include <iostream>

#include "mujoco.h"
#include "SharedRobotDefinitions.h"
#include "CassieToMuJoCo.h"
#include "CassieToRBDL.h"
#include "DynamicModel.h"
#include "Visualizer.h"
#include <fstream>

ofstream groundFile;
ofstream reconFile;

using namespace std;

int main(int argc,char* argv[]) {

	mj_activate("../../ThirdParty/mjpro150/mjkey.txt");
	char error[1000] = "Could not load binary model";
	mjModel* mj_Model = mj_loadXML(xml_model_filename.c_str(), 0, error, 1000);
	if (!mj_Model) {
		mju_error_s("Load model error: %s", error);
		return -1;
	}
	// Initialize mjData
	mjData* mj_Data = mj_makeData(mj_Model);
	mjData* mj_Data_recon = mj_makeData(mj_Model);

	double qpos_init[] = {0.0, 0.0, 0.939, 0.707, 0.0, 0.0, 0.707,
			0.0, 0.0, 0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407,
			0.0, 0.0, 0.68111815, -1.40730353, 1.62972043, -1.77611107, -0.61968402};
	mju_copy(mj_Data->qpos, qpos_init, nX);

	mj_forward(mj_Model, mj_Data);

	Visualizer* vis_sim = new Visualizer(mj_Model, false, "original");
	Visualizer* vis_recon = new Visualizer(mj_Model, false, "reconstruction");

	DynamicModel cassie;
	cassie.LoadModel(xml_model_filename);

	groundFile.open("ground_truth.csv");
	reconFile.open("recon.csv");


	cassie_out_t sensors;

	//initiate comms

	double qpos_rbdl[nX];
	double qvel_rbdl[nQ];
	int nRows = 4000;
	while (nRows--) {

		StateToCassieOutputs(mj_Data->qpos, mj_Data->qvel, &sensors);
//
//		printf("MuJoCo Jacobian:\n");
//		mjtNum jac[nQ*3];
//		mj_jacSite(mj_Model, mj_Data, jac, NULL, 2);
//		mju_printMat(jac, 3, nQ);

		CassieOutputsToState(&cassie, sensors, qpos_rbdl, qvel_rbdl);
//		for (int i = 0; i < 3; i++)
//			qpos_rbdl[i] = mj_Data->qpos[i];
		cassie.setState(qpos_rbdl, qvel_rbdl);

//		Eigen::MatrixXd rbdl_jac = Eigen::MatrixXd::Zero(3, nQ);
//		cassie.GetSiteJacobian(&rbdl_jac, 2);
//		std::cout << "RBDL Jacobian:\n" << rbdl_jac << std::endl;
//
//
//		printf("MuJoCo Mass Matrix:\n");
//		mjtNum mass[nQ*nQ];
//		mju_zero(mass, nQ*nQ);
//		mj_fullM(mj_Model, mass, mj_Data->qM);
//		mju_printMat(mass, nQ, nQ);
//
//		Eigen::MatrixXd rbdl_mass;
//		cassie.GetMassMatrix(&rbdl_mass);
//		std::cout << "RBDL Mass:\n" << rbdl_mass << std::endl;
//
		printf("Site Pos:\n");
		printf("%f\t%f\t%f\n", mj_Data->site_xpos[6], mj_Data->site_xpos[7], mj_Data->site_xpos[8]);

		RigidBodyDynamics::Math::VectorNd x = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);
		RigidBodyDynamics::Math::VectorNd xd = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);

		int targids[] = {1,2,3,4,5};
		cassie.GetTargetPoints(&x, &xd, targids);

//		printf("RBDL Site Pos:\n");
		printf("%f\t%f\t%f\n", x(3), x(4), x(5));
		printf("%f\t%f\t%f\n", qpos_rbdl[3], qpos_rbdl[4], qpos_rbdl[5]);

		for (int i = 0; i < 3; i++)
			mj_Data_recon->qpos[i]=qpos_rbdl[i];
		double quat[4];
		cassie.GetMainBodyQuaternion(quat);
		for (int i = 3; i < 7; i++)
			mj_Data_recon->qpos[i] = quat[i-3];
		for (int i = 7; i < nX; i++)
			mj_Data_recon->qpos[i] = qpos_rbdl[i-1];

		for (int i = 0; i < nQ; i++)
			mj_Data_recon->qvel[i] = qvel_rbdl[i];

		for (int i = 0; i < nU; i++)
			mj_Data->ctrl[i]=0.0;
		mj_Data->ctrl[3] = mj_Data->ctrl[8] = 2.0;

		for (int i = 0; i < nQ+1; i++)
			groundFile << mj_Data->qpos[i] << ",";
		for (int i = 0; i < nQ-1; i++)
			groundFile << mj_Data->qvel[i] << ",";
		groundFile << mj_Data->qvel[nQ-1] << std::endl;

		for (int i = 0; i < nQ+1; i++)
			reconFile << mj_Data_recon->qpos[i] << ",";
		for (int i = 0; i < nQ-1; i++)
			reconFile << mj_Data_recon->qvel[i] << ",";
		reconFile << mj_Data_recon->qvel[nQ-1] << std::endl;


		mj_forward(mj_Model, mj_Data_recon);
		mj_step(mj_Model, mj_Data);

//		while(true)
		vis_sim->Draw(mj_Data);
		vis_recon->Draw(mj_Data_recon);

	}
	reconFile.close();
	groundFile.close();

}
