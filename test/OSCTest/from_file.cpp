#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include "SharedRobotDefinitions.h"
#include "DynamicModel.h"
#include "DynamicState.h"
#include "OSC_RBDL.h"
#include <fstream>

using namespace std;

ofstream reconFile;

int main(int argc,char* argv[]) {

	int contactIds[] = {2, 3, 4, 5};
	int targetIds[] = {1, 2, 3, 4, 5};

	DynamicState dyn_state;
	dyn_state.Init(contactIds);

	OSC_RBDL* osc = new OSC_RBDL(targetIds);
	osc->AddQDDIdx(3);
	osc->AddQDDIdx(4);
	osc->AddQDDIdx(5);

	FILE* stream = fopen("../out/LogFiles/logfile.csv", "r");

	DynamicModel cassie;
	cassie.LoadModel(xml_model_filename);

	reconFile.open("recon.csv");

	double qpos[nX];
	double qvel[nQ];
	double u_old[nU];
	Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, 1> xdd;

	unsigned int line_size = 10000;
	char line[line_size];
	unsigned int num_vals = 142;
	double line_vals[num_vals];

	bool bFirst = true;

	while (fgets(line, line_size, stream)) {

		for (int i = 0; i < num_vals-1; i++)
			sscanf(line, "%lf,%s", &line_vals[i], line);
		sscanf(line, "%lf", &line_vals[num_vals-1]);

		for (int i = 0; i < nX; i++)
			qpos[i] = line_vals[i];
		for (int i = 0; i < nQ; i++)
			qvel[i] = line_vals[i+nX];
		for (int i = 0; i < nU; i++)
			u_old[i] = line_vals[i+131];
		for (int i = 0; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
			xdd(i,0) = line_vals[i+77];

		cassie.setState(qpos, qvel);

		if (bFirst)
			osc->InitMatrices(&cassie);
		bFirst = false;

		dyn_state.UpdateDynamicState(&cassie);

		Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();

		bool bInContact[] = {true, true, true, true};

		Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
		for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
			bActive(i,0) = true;

		osc->RunPTSC(&cassie, &dyn_state, xdd, bActive, bInContact, &u);

		for (int i = 0; i < nU; i++)
			reconFile << u_old[i] << ",";

		for (int i = 0; i < nU; i++)
			reconFile << u(i,0) << ",";

		reconFile << 0 << std::endl;
	}
	reconFile.close();
}
