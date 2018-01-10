#ifndef CASSIE_OUT_T_TYPES_H
#define CASSIE_OUT_T_TYPES_H
#include "rtwtypes.h"
#ifndef typedef_DiagnosticCodes
#define typedef_DiagnosticCodes

typedef short DiagnosticCodes;

#endif

#ifndef DiagnosticCodes_constants
#define DiagnosticCodes_constants

#define EMPTY                          ((DiagnosticCodes)0)
#define LEFT_HIP_NOT_CALIB             ((DiagnosticCodes)5)
#define LEFT_KNEE_NOT_CALIB            ((DiagnosticCodes)6)
#define RIGHT_HIP_NOT_CALIB            ((DiagnosticCodes)7)
#define RIGHT_KNEE_NOT_CALIB           ((DiagnosticCodes)8)
#define LOW_BATTERY_CHARGE             ((DiagnosticCodes)200)
#define HIGH_CPU_TEMP                  ((DiagnosticCodes)205)
#define HIGH_VTM_TEMP                  ((DiagnosticCodes)210)
#define HIGH_ELMO_DRIVE_TEMP           ((DiagnosticCodes)215)
#define HIGH_STATOR_TEMP               ((DiagnosticCodes)220)
#define LOW_ELMO_LINK_VOLTAGE          ((DiagnosticCodes)221)
#define HIGH_BATTERY_TEMP              ((DiagnosticCodes)225)
#define RADIO_DATA_BAD                 ((DiagnosticCodes)230)
#define RADIO_SIGNAL_BAD               ((DiagnosticCodes)231)
#define BMS_DATA_BAD                   ((DiagnosticCodes)235)
#define VECTORNAV_DATA_BAD             ((DiagnosticCodes)236)
#define VPE_GYRO_SATURATION            ((DiagnosticCodes)240)
#define VPE_MAG_SATURATION             ((DiagnosticCodes)241)
#define VPE_ACC_SATURATION             ((DiagnosticCodes)242)
#define VPE_ATTITUDE_BAD               ((DiagnosticCodes)245)
#define VPE_ATTITUDE_NOT_TRACKING      ((DiagnosticCodes)246)
#define ETHERCAT_DC_ERROR              ((DiagnosticCodes)400)
#define ETHERCAT_ERROR                 ((DiagnosticCodes)410)
#define LOAD_CALIB_DATA_ERROR          ((DiagnosticCodes)590)
#define CRITICAL_BATTERY_CHARGE        ((DiagnosticCodes)600)
#define CRITICAL_CPU_TEMP              ((DiagnosticCodes)605)
#define CRITICAL_VTM_TEMP              ((DiagnosticCodes)610)
#define CRITICAL_ELMO_DRIVE_TEMP       ((DiagnosticCodes)615)
#define CRITICAL_STATOR_TEMP           ((DiagnosticCodes)620)
#define CRITICAL_BATTERY_TEMP          ((DiagnosticCodes)625)
#define TORQUE_LIMIT_REACHED           ((DiagnosticCodes)630)
#define JOINT_LIMIT_REACHED            ((DiagnosticCodes)635)
#define ENCODER_FAILURE                ((DiagnosticCodes)640)
#define SPRING_FAILURE                 ((DiagnosticCodes)645)
#define LEFT_LEG_MEDULLA_HANG          ((DiagnosticCodes)700)
#define RIGHT_LEG_MEDULLA_HANG         ((DiagnosticCodes)701)
#define PELVIS_MEDULLA_HANG            ((DiagnosticCodes)703)
#define CPU_OVERLOAD                   ((DiagnosticCodes)704)
#endif

#ifndef typedef_cassie_battery_out_t
#define typedef_cassie_battery_out_t

typedef struct {
  boolean_T dataGood;
  double stateOfCharge;
  double voltage[12];
  double current;
  double temperature[4];
} cassie_battery_out_t;

#endif

#ifndef typedef_cassie_drive_out_t
#define typedef_cassie_drive_out_t

typedef struct {
  unsigned short statusWord;
  double position;
  double velocity;
  double torque;
  double driveTemperature;
  double dcLinkVoltage;
  double torqueLimit;
  double gearRatio;
} cassie_drive_out_t;

#endif

#ifndef typedef_cassie_joint_out_t
#define typedef_cassie_joint_out_t

typedef struct {
  double position;
  double velocity;
} cassie_joint_out_t;

#endif

#ifndef typedef_cassie_leg_out_t
#define typedef_cassie_leg_out_t

typedef struct {
  cassie_drive_out_t hipRollDrive;
  cassie_drive_out_t hipYawDrive;
  cassie_drive_out_t hipPitchDrive;
  cassie_drive_out_t kneeDrive;
  cassie_drive_out_t footDrive;
  cassie_joint_out_t shinJoint;
  cassie_joint_out_t tarsusJoint;
  cassie_joint_out_t footJoint;
  unsigned char medullaCounter;
  unsigned short medullaCpuLoad;
  boolean_T reedSwitchState;
} cassie_leg_out_t;

#endif

#ifndef typedef_cassie_radio_out_t
#define typedef_cassie_radio_out_t

typedef struct {
  boolean_T radioReceiverSignalGood;
  boolean_T receiverMedullaSignalGood;
  double channel[16];
} cassie_radio_out_t;

#endif

#ifndef typedef_cassie_target_pc_out_t
#define typedef_cassie_target_pc_out_t

typedef struct {
  int etherCatStatus[6];
  int etherCatNotifications[21];
  double taskExecutionTime;
  unsigned int overloadCounter;
  double cpuTemperature;
} cassie_target_pc_out_t;

#endif

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  boolean_T dataGood;
  unsigned short vpeStatus;
  double pressure;
  double temperature;
  double magneticField[3];
  double angularVelocity[3];
  double linearAcceleration[3];
  double orientation[4];
} struct0_T;

#endif

#ifndef typedef_cassie_pelvis_out_t
#define typedef_cassie_pelvis_out_t

typedef struct {
  cassie_target_pc_out_t targetPc;
  cassie_battery_out_t battery;
  cassie_radio_out_t radio;
  struct0_T vectorNav;
  unsigned char medullaCounter;
  unsigned short medullaCpuLoad;
  boolean_T bleederState;
  boolean_T leftReedSwitchState;
  boolean_T rightReedSwitchState;
  double vtmTemperature;
} cassie_pelvis_out_t;

#endif

#ifndef typedef_cassie_out_t
#define typedef_cassie_out_t

typedef struct {
  cassie_pelvis_out_t pelvis;
  cassie_leg_out_t leftLeg;
  cassie_leg_out_t rightLeg;
  boolean_T isCalibrated;
  DiagnosticCodes messages[4];
} cassie_out_t;

#endif
#endif
