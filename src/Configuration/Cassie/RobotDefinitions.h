/*
 * RobotDefinitions.h
 *
 *  Created on: Dec 7, 2017
 *      Author: tapgar
 */

#ifndef ROBOTDEFINITIONS_H_
#define ROBOTDEFINITIONS_H_

#define nQ 24 //# joints
#define nX nQ+1 //pos=vel+1 (quaternion)
#define nQstiff 20 //# joints w/o springs
#define nU 10 //# actuated joints
#define nCON 4 //# contact points
#define nEQ 6 //#of equality constraints 2*3

#define XDD_TARGETS 5 //# of cartesian target accelerations
#define QDD_TARGETS 5 //# of joint target accels

#define nENC 14


#define DOF 3 // 2D vs 3D

static const double m_dControlTime_s = 0.0005; //how often the controller runs

static const double m_dNominalZTarget_m = 0.95;

#ifdef EMBEDDED
static const std::string xml_model_filename = "/home/robot/DRL/TestStand/src/Configuration/SingleLeg_FB/singleleg.xml";
#else
static const std::string xml_model_filename = "/home/tapgar/cuda-workspace/TestStand/src/Configuration/SingleLeg_FB/singleleg.xml";
#endif



#endif /* ROBOTDEFINITIONS_H_ */
