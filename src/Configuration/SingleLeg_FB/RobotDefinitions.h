/*
 * RobotDefinitions.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef ROBOTDEFINITIONS_H_
#define ROBOTDEFINITIONS_H_

#define nQ 6 //# joints
#define nX nQ //pos=vel+1 (quaternion)
#define nU 3 //# actuated joints
#define nCON 2 //# contact points
#define nEQ 3 //#of equality constraints 4*3

#define XDD_TARGETS 3 //# of cartesian target accelerations
#define QDD_TARGETS 0 //# of joint target accels

#define DOF 3 // 2D vs 3D

static const double m_dControlTime_s = 0.0005; //how often the controller runs

static const double m_dNominalZTarget_m = 0.95;

#ifdef EMBEDDED
static const std::string xml_model_filename = "/home/robot/DRL/TestStand/src/Configuration/SingleLeg_FB/singleleg.xml";
#else
static const std::string xml_model_filename = "/home/tapgar/cuda-workspace/TestStand/src/Configuration/SingleLeg_FB/singleleg.xml";
#endif



#endif /* ROBOTDEFINITIONS_H_ */
