/*
 * DynamicModel.h
 *
 *  Created on: Dec 20, 2017
 *      Author: tapgar
 */

#ifndef DYNAMICMODEL_H_
#define DYNAMICMODEL_H_

#include <rbdl/rbdl.h>
#include "xml_parser.h"
#include <Eigen/Dense>
#include "SharedRobotDefinitions.h"

struct RobotState {
	RigidBodyDynamics::Math::VectorNd qpos;
	RigidBodyDynamics::Math::VectorNd qvel;
};

struct Site {
	RigidBodyDynamics::Math::Vector3d pos;
	unsigned int body_id;
	std::string name;
};

struct Constraint {
	RigidBodyDynamics::Math::Vector3d posA;
	unsigned int bodyA_id;

	RigidBodyDynamics::Math::Vector3d posB;
	unsigned int bodyB_id;
};

class DynamicModel {
public:
	DynamicModel();
	virtual ~DynamicModel();

	void LoadModel(std::string xml_file);

	void setState(double* qpos, double* qvel);

	void GetMassMatrix(RigidBodyDynamics::Math::MatrixNd* mat);
	void GetConstraintJacobian(RigidBodyDynamics::Math::MatrixNd* mat);
	void GetConstraintAccel(RigidBodyDynamics::Math::VectorNd* accel);
	void GetConstraintVel(RigidBodyDynamics::Math::VectorNd* vel);
	void GetDrivenVelocityAtConstraint(RigidBodyDynamics::Math::VectorNd* qdot, unsigned int constraint_id);

	void GetBiasForce(RigidBodyDynamics::Math::VectorNd* qfrc_bias);
	void GetPassiveForce(RigidBodyDynamics::Math::VectorNd* qfrc_passive);

	void GetSiteJacobian(RigidBodyDynamics::Math::MatrixNd* mat, int siteId);
	void GetSiteAccel(RigidBodyDynamics::Math::VectorNd* accel, int siteId);

	void GetSelectorMatrix(RigidBodyDynamics::Math::MatrixNd* mat);

	void GetMotorLimits(RigidBodyDynamics::Math::MatrixNd* mlb, RigidBodyDynamics::Math::MatrixNd* mub);

	void GetTargetPoints(RigidBodyDynamics::Math::VectorNd* x, RigidBodyDynamics::Math::VectorNd* xd);

	void GetConstraintPointsDependent(RigidBodyDynamics::Math::VectorNd* x, RigidBodyDynamics::Math::VectorNd* constraint_base_pose);

	double GetMass() { return m_TotalMass_kg; }

	bool IsSpringJoint(int idx) { return jointStiffness(idx,0) > 0.0; }

private:

	RigidBodyDynamics::Model m;
	RobotState m_State;

	RigidBodyDynamics::Math::MatrixNd rotorInertia;
	RigidBodyDynamics::Math::MatrixNd selectorMatrix;
	RigidBodyDynamics::Math::VectorNd jointDamping;
	RigidBodyDynamics::Math::VectorNd jointStiffness;
	RigidBodyDynamics::Math::VectorNd jointReference;

	RigidBodyDynamics::Math::MatrixNd motorLimits;

	std::vector<Site> m_Sites;
	std::vector<Constraint> m_Constraints;

	double m_TotalMass_kg;

};

#endif /* DYNAMICMODEL_H_ */
