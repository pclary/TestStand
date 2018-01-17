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

	FILE* stream = fopen("../out/LogFiles/log.csv", "r");

	mjData* mj_Data_recon = mj_makeData(mj_Model);

	double qpos_init[] = {0.0, 0.0, 0.939, 1.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407,
			0.0, 0.0, 0.68111815, -1.40730353, 1.62972043, -1.77611107, -0.61968402};
	mju_copy(mj_Data->qpos, qpos_init, nX);

	mj_forward(mj_Model, mj_Data);

	Visualizer* vis_recon = new Visualizer(mj_Model, false, "reconstruction");

	DynamicModel cassie;
	cassie.LoadModel(xml_model_filename);

	groundFile.open("ground_truth.csv");
	reconFile.open("recon.csv");


	cassie_out_t sensors;

	//initiate comms

	double qpos_rbdl[nQ];
	double qvel_rbdl[nQ];

	char line[5000];
	double line_vals[41];

	while (fgets(line, 5000, stream)) {

		for (int i = 0; i < 40; i++)
			sscanf(line, "%lf,%s", &line_vals[i], line);
		sscanf(line, "%lf", &line_vals[40]);

		for (int i = 0; i < 4; i++)
			sensors.pelvis.vectorNav.orientation[i] = line_vals[i+37];

		for (int i = 0; i < 3; i++)
			sensors.pelvis.vectorNav.angularVelocity[i] = line_vals[i+31];

		sensors.leftLeg.hipRollDrive.position = line_vals[0];
		sensors.leftLeg.hipRollDrive.velocity = line_vals[1];
		sensors.leftLeg.hipYawDrive.position = line_vals[2];
		sensors.leftLeg.hipYawDrive.velocity = line_vals[3];
		sensors.leftLeg.hipPitchDrive.position = line_vals[4];
		sensors.leftLeg.hipPitchDrive.velocity = line_vals[5];
		sensors.leftLeg.kneeDrive.position = line_vals[6];
		sensors.leftLeg.kneeDrive.velocity = line_vals[7];
		sensors.leftLeg.shinJoint.position = line_vals[8];
		sensors.leftLeg.shinJoint.velocity = line_vals[9];
		sensors.leftLeg.tarsusJoint.position = line_vals[10];
		sensors.leftLeg.tarsusJoint.velocity = line_vals[11];
		sensors.leftLeg.footJoint.position = line_vals[12];
		sensors.leftLeg.footJoint.velocity = line_vals[13];

		sensors.rightLeg.hipRollDrive.position = line_vals[14];
		sensors.rightLeg.hipRollDrive.velocity = line_vals[15];
		sensors.rightLeg.hipYawDrive.position = line_vals[16];
		sensors.rightLeg.hipYawDrive.velocity = line_vals[17];
		sensors.rightLeg.hipPitchDrive.position = line_vals[18];
		sensors.rightLeg.hipPitchDrive.velocity = line_vals[19];
		sensors.rightLeg.kneeDrive.position = line_vals[20];
		sensors.rightLeg.kneeDrive.velocity = line_vals[21];
		sensors.rightLeg.shinJoint.position = line_vals[22];
		sensors.rightLeg.shinJoint.velocity = line_vals[23];
		sensors.rightLeg.tarsusJoint.position = line_vals[24];
		sensors.rightLeg.tarsusJoint.velocity = line_vals[25];
		sensors.rightLeg.footJoint.position = line_vals[26];
		sensors.rightLeg.footJoint.velocity = line_vals[27];
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
//		printf("MuJoCo Site Pos:\n");
//		printf("%f\t%f\t%f\n", mj_Data->site_xpos[6], mj_Data->site_xpos[7], mj_Data->site_xpos[8]);
//
//		RigidBodyDynamics::Math::VectorNd x = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);
//		RigidBodyDynamics::Math::VectorNd xd = RigidBodyDynamics::Math::VectorNd::Zero(DOF*XDD_TARGETS);
//
//		int targids[] = {1,2,3,4,5};
//		cassie.GetTargetPoints(&x, &xd, targids);
//
//		printf("RBDL Site Pos:\n");
//		printf("%f\t%f\t%f\n", x(3), x(4), x(5));

		for (int i = 0; i < 3; i++)
			mj_Data_recon->qpos[i]=qpos_rbdl[i];
		for (int i = 6; i < nQ; i++)
			mj_Data_recon->qpos[i+1] = qpos_rbdl[i];
		eulerToQuaternion(&(qpos_rbdl[3]), &(mj_Data_recon->qpos[3]));

		for (int i = 0; i < nQ; i++)
			mj_Data_recon->qvel[i] = qvel_rbdl[i];

		for (int i = 0; i < nU; i++)
			mj_Data->ctrl[i]=0.0;
		mj_Data->ctrl[3] = mj_Data->ctrl[8] = 2.0;


		mj_forward(mj_Model, mj_Data_recon);
		mj_step(mj_Model, mj_Data);

//		while(true)
//		vis_sim->Draw(mj_Data);
		vis_recon->Draw(mj_Data_recon);

	}
	reconFile.close();
	groundFile.close();

}
