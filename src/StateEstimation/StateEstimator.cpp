/*
 * StateEstimator.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#include "StateEstimator.h"

StateEstimator::StateEstimator() {
	// TODO Auto-generated constructor stub

}

StateEstimator::~StateEstimator() {
	// TODO Auto-generated destructor stub
}

void StateEstimator::Init(DynamicModel* dyn, cassie_out_t sensors)
{
	CassieOutToMeasurement(sensors, &m_Measurement);
	MeasurementToState(dyn, m_Measurement, &m_State);
	//have to start on the ground
	for (int i = 0; i < nCON; i++)
	{
		m_State.contact[i] = 1.0;
		m_State.force[i*3+0] = m_State.force[i*3+1] = 0.0;
		m_State.force[i*3+2] = dyn->GetMass()/nCON;
	}

}

void StateEstimator::Update(DynamicModel* dyn, DynamicState* dyn_state, cassie_out_t sensors)
{

	CassieOutToMeasurement(sensors, &m_Measurement);

	m_ContactStateEstimator.Update(&m_State, m_Measurement);
	m_BodyEstimator.Update(dyn, dyn_state, &m_State, m_Measurement);
	m_FloatingBaseEstimator.Update(dyn, &m_State, m_Measurement);
}

void StateEstimator::CassieOutToMeasurement(cassie_out_t sensors, measurement_t* m)
{
	m->zqpos[0] = sensors.leftLeg.hipRollDrive.position;
	m->zqvel[0] = sensors.leftLeg.hipRollDrive.velocity;
	m->ztorque[0] = sensors.leftLeg.hipRollDrive.torque;

	m->zqpos[1] = sensors.leftLeg.hipYawDrive.position;
	m->zqvel[1] = sensors.leftLeg.hipYawDrive.velocity;
	m->ztorque[1] = sensors.leftLeg.hipYawDrive.torque;

	m->zqpos[2] = sensors.leftLeg.hipPitchDrive.position;
	m->zqvel[2] = sensors.leftLeg.hipPitchDrive.velocity;
	m->ztorque[2] = sensors.leftLeg.hipPitchDrive.torque;

	m->zqpos[3] = sensors.leftLeg.kneeDrive.position;
	m->zqvel[3] = sensors.leftLeg.kneeDrive.velocity;
	m->ztorque[3] = sensors.leftLeg.kneeDrive.torque;

	m->zqpos[4] = sensors.leftLeg.shinJoint.position;
	m->zqvel[4] = sensors.leftLeg.shinJoint.velocity;

	m->zqpos[5] = sensors.leftLeg.tarsusJoint.position;
	m->zqvel[5] = sensors.leftLeg.tarsusJoint.velocity;

	m->zqpos[6] = sensors.leftLeg.footJoint.position;
	m->zqvel[6] = sensors.leftLeg.footJoint.velocity;
	m->ztorque[4] = sensors.leftLeg.footDrive.torque;

	m->zqpos[7] = sensors.rightLeg.hipRollDrive.position;
	m->zqvel[7] = sensors.rightLeg.hipRollDrive.velocity;
	m->ztorque[5] = sensors.rightLeg.hipRollDrive.torque;

	m->zqpos[8] = sensors.rightLeg.hipYawDrive.position;
	m->zqvel[8] = sensors.rightLeg.hipYawDrive.velocity;
	m->ztorque[6] = sensors.rightLeg.hipYawDrive.torque;

	m->zqpos[9] = sensors.rightLeg.hipPitchDrive.position;
	m->zqvel[9] = sensors.rightLeg.hipPitchDrive.velocity;
	m->ztorque[7] = sensors.rightLeg.hipPitchDrive.torque;

	m->zqpos[10] = sensors.rightLeg.kneeDrive.position;
	m->zqvel[10] = sensors.rightLeg.kneeDrive.velocity;
	m->ztorque[8] = sensors.rightLeg.kneeDrive.torque;

	m->zqpos[11] = sensors.rightLeg.shinJoint.position;
	m->zqvel[11] = sensors.rightLeg.shinJoint.velocity;

	m->zqpos[12] = sensors.rightLeg.tarsusJoint.position;
	m->zqvel[12] = sensors.rightLeg.tarsusJoint.velocity;

	m->zqpos[13] = sensors.rightLeg.footJoint.position;
	m->zqvel[13] = sensors.rightLeg.footJoint.velocity;
	m->ztorque[9] = sensors.rightLeg.footDrive.torque;

	for (int i = 0; i < 3; i++)
	{
		m->zacc(i) = sensors.pelvis.vectorNav.linearAcceleration[i];
		m->zimu(i) = sensors.pelvis.vectorNav.angularVelocity[i];
		m->zmag(i) = sensors.pelvis.vectorNav.magneticField[i];
		m->zquat(i,0) = sensors.pelvis.vectorNav.orientation[i];
	}
	m->zquat(3,0) = sensors.pelvis.vectorNav.orientation[3];

	Eigen::Matrix3d bodyToIMU;
	bodyToIMU << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;

	m->zacc = bodyToIMU.transpose()*m->zacc;
	m->zimu = bodyToIMU.transpose()*m->zimu;
	m->zmag = bodyToIMU.transpose()*m->zmag;
	//ROTATE THE FUCKING QUATERNION!!!
	//i believe 180 0 180 corresponds to wxyz=0,0,-1,0

}

void StateEstimator::MeasurementToState(DynamicModel* dyn, measurement_t m, state_t* s)
{

	//qpos 0->5... figure this shit out
	for (int i = 0; i < nENC/2; i++)
	{
		s->qpos[i+6] = m.zqpos[i];
		s->qvel[i+6] = m.zqvel[i];
		s->qpos[i+15] = m.zqpos[i+nENC/2];
		s->qvel[i+15] = m.zqvel[i+nENC/2];
	}

	//set q only (dynamic model)

	//law of cosines for finding qpos[13,14,22,23]

}
