#include "cassie_out_t.h"
#include <stddef.h>

static DiagnosticCodes convert_to_enum_DiagnosticCodes(short u);
static DiagnosticCodes convert_to_enum_DiagnosticCodes(short u)
{
  DiagnosticCodes y;
  if (u == 0) {
    y = EMPTY;
  } else if (u == 5) {
    y = LEFT_HIP_NOT_CALIB;
  } else if (u == 6) {
    y = LEFT_KNEE_NOT_CALIB;
  } else if (u == 7) {
    y = RIGHT_HIP_NOT_CALIB;
  } else if (u == 8) {
    y = RIGHT_KNEE_NOT_CALIB;
  } else if (u == 200) {
    y = LOW_BATTERY_CHARGE;
  } else if (u == 205) {
    y = HIGH_CPU_TEMP;
  } else if (u == 210) {
    y = HIGH_VTM_TEMP;
  } else if (u == 215) {
    y = HIGH_ELMO_DRIVE_TEMP;
  } else if (u == 220) {
    y = HIGH_STATOR_TEMP;
  } else if (u == 221) {
    y = LOW_ELMO_LINK_VOLTAGE;
  } else if (u == 225) {
    y = HIGH_BATTERY_TEMP;
  } else if (u == 230) {
    y = RADIO_DATA_BAD;
  } else if (u == 231) {
    y = RADIO_SIGNAL_BAD;
  } else if (u == 235) {
    y = BMS_DATA_BAD;
  } else if (u == 236) {
    y = VECTORNAV_DATA_BAD;
  } else if (u == 240) {
    y = VPE_GYRO_SATURATION;
  } else if (u == 241) {
    y = VPE_MAG_SATURATION;
  } else if (u == 242) {
    y = VPE_ACC_SATURATION;
  } else if (u == 245) {
    y = VPE_ATTITUDE_BAD;
  } else if (u == 246) {
    y = VPE_ATTITUDE_NOT_TRACKING;
  } else if (u == 400) {
    y = ETHERCAT_DC_ERROR;
  } else if (u == 410) {
    y = ETHERCAT_ERROR;
  } else if (u == 590) {
    y = LOAD_CALIB_DATA_ERROR;
  } else if (u == 600) {
    y = CRITICAL_BATTERY_CHARGE;
  } else if (u == 605) {
    y = CRITICAL_CPU_TEMP;
  } else if (u == 610) {
    y = CRITICAL_VTM_TEMP;
  } else if (u == 615) {
    y = CRITICAL_ELMO_DRIVE_TEMP;
  } else if (u == 620) {
    y = CRITICAL_STATOR_TEMP;
  } else if (u == 625) {
    y = CRITICAL_BATTERY_TEMP;
  } else if (u == 630) {
    y = TORQUE_LIMIT_REACHED;
  } else if (u == 635) {
    y = JOINT_LIMIT_REACHED;
  } else if (u == 640) {
    y = ENCODER_FAILURE;
  } else if (u == 645) {
    y = SPRING_FAILURE;
  } else if (u == 700) {
    y = LEFT_LEG_MEDULLA_HANG;
  } else if (u == 701) {
    y = RIGHT_LEG_MEDULLA_HANG;
  } else if (u == 703) {
    y = PELVIS_MEDULLA_HANG;
  } else if (u == 704) {
    y = CPU_OVERLOAD;
  } else {
    y = EMPTY;
  }

  return y;
}

void cassie_out_t_initialize(void)
{
}

void cassie_out_t_terminate(void)
{
}

void pack_cassie_out_t(const cassie_out_t *bus, unsigned char bytes[1233])
{
  int i;
  unsigned char y[24];
  int x[6];
  int b_x[21];
  unsigned char b_y[84];
  double c_x;
  unsigned char c_y[8];
  unsigned int d_x;
  unsigned char d_y[4];
  unsigned char e_x;
  unsigned char e_y;
  double f_x[12];
  unsigned char f_y[96];
  unsigned char g_y[32];
  double g_x[4];
  double h_x[16];
  unsigned char h_y[128];
  unsigned short i_x;
  unsigned char i_y[2];
  double j_x[3];
  short k_x[4];
  for (i = 0; i < 6; i++) {
    x[i] = bus->pelvis.targetPc.etherCatStatus[i];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)24 * sizeof
          (unsigned char)));
  for (i = 0; i < 24; i++) {
    bytes[i] = y[i];
  }

  memcpy(&b_x[0], &bus->pelvis.targetPc.etherCatNotifications[0], 21U * sizeof
         (int));
  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)84 * sizeof
          (unsigned char)));
  memcpy(&bytes[24], &b_y[0], 84U * sizeof(unsigned char));
  c_x = bus->pelvis.targetPc.taskExecutionTime;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[108 + i] = c_y[i];
  }

  d_x = bus->pelvis.targetPc.overloadCounter;
  memcpy((void *)&d_y[0], (void *)&d_x, (unsigned int)((size_t)4 * sizeof
          (unsigned char)));
  for (i = 0; i < 4; i++) {
    bytes[116 + i] = d_y[i];
  }

  c_x = bus->pelvis.targetPc.cpuTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[120 + i] = c_y[i];
  }

  e_x = bus->pelvis.battery.dataGood;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[128] = e_y;
  c_x = bus->pelvis.battery.stateOfCharge;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[129 + i] = c_y[i];
  }

  memcpy(&f_x[0], &bus->pelvis.battery.voltage[0], 12U * sizeof(double));
  memcpy((void *)&f_y[0], (void *)&f_x[0], (unsigned int)((size_t)96 * sizeof
          (unsigned char)));
  memcpy(&bytes[137], &f_y[0], 96U * sizeof(unsigned char));
  c_x = bus->pelvis.battery.current;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[233 + i] = c_y[i];
  }

  for (i = 0; i < 4; i++) {
    g_x[i] = bus->pelvis.battery.temperature[i];
  }

  memcpy((void *)&g_y[0], (void *)&g_x[0], (unsigned int)((size_t)32 * sizeof
          (unsigned char)));
  for (i = 0; i < 32; i++) {
    bytes[i + 241] = g_y[i];
  }

  e_x = bus->pelvis.radio.radioReceiverSignalGood;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[273] = e_y;
  e_x = bus->pelvis.radio.receiverMedullaSignalGood;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[274] = e_y;
  memcpy(&h_x[0], &bus->pelvis.radio.channel[0], sizeof(double) << 4);
  memcpy((void *)&h_y[0], (void *)&h_x[0], (unsigned int)((size_t)128 * sizeof
          (unsigned char)));
  memcpy(&bytes[275], &h_y[0], sizeof(unsigned char) << 7);
  e_x = bus->pelvis.vectorNav.dataGood;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[403] = e_y;
  i_x = bus->pelvis.vectorNav.vpeStatus;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[404 + i] = i_y[i];
  }

  c_x = bus->pelvis.vectorNav.pressure;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[406 + i] = c_y[i];
  }

  c_x = bus->pelvis.vectorNav.temperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[414 + i] = c_y[i];
  }

  for (i = 0; i < 3; i++) {
    j_x[i] = bus->pelvis.vectorNav.magneticField[i];
  }

  memcpy((void *)&y[0], (void *)&j_x[0], (unsigned int)((size_t)24 * sizeof
          (unsigned char)));
  for (i = 0; i < 24; i++) {
    bytes[i + 422] = y[i];
  }

  for (i = 0; i < 3; i++) {
    j_x[i] = bus->pelvis.vectorNav.angularVelocity[i];
  }

  memcpy((void *)&y[0], (void *)&j_x[0], (unsigned int)((size_t)24 * sizeof
          (unsigned char)));
  for (i = 0; i < 24; i++) {
    bytes[i + 446] = y[i];
  }

  for (i = 0; i < 3; i++) {
    j_x[i] = bus->pelvis.vectorNav.linearAcceleration[i];
  }

  memcpy((void *)&y[0], (void *)&j_x[0], (unsigned int)((size_t)24 * sizeof
          (unsigned char)));
  for (i = 0; i < 24; i++) {
    bytes[i + 470] = y[i];
  }

  for (i = 0; i < 4; i++) {
    g_x[i] = bus->pelvis.vectorNav.orientation[i];
  }

  memcpy((void *)&g_y[0], (void *)&g_x[0], (unsigned int)((size_t)32 * sizeof
          (unsigned char)));
  for (i = 0; i < 32; i++) {
    bytes[i + 494] = g_y[i];
  }

  e_x = bus->pelvis.medullaCounter;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[526] = e_y;
  i_x = bus->pelvis.medullaCpuLoad;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[527 + i] = i_y[i];
  }

  e_x = bus->pelvis.bleederState;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[529] = e_y;
  e_x = bus->pelvis.leftReedSwitchState;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[530] = e_y;
  e_x = bus->pelvis.rightReedSwitchState;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[531] = e_y;
  c_x = bus->pelvis.vtmTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[532 + i] = c_y[i];
  }

  i_x = bus->leftLeg.hipRollDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[540 + i] = i_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[542 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[550 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[558 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[566 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[574 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[582 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipRollDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[590 + i] = c_y[i];
  }

  i_x = bus->leftLeg.hipYawDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[598 + i] = i_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[600 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[608 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[616 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[624 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[632 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[640 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipYawDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[648 + i] = c_y[i];
  }

  i_x = bus->leftLeg.hipPitchDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[656 + i] = i_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[658 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[666 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[674 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[682 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[690 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[698 + i] = c_y[i];
  }

  c_x = bus->leftLeg.hipPitchDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[706 + i] = c_y[i];
  }

  i_x = bus->leftLeg.kneeDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[714 + i] = i_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[716 + i] = c_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[724 + i] = c_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[732 + i] = c_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[740 + i] = c_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[748 + i] = c_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[756 + i] = c_y[i];
  }

  c_x = bus->leftLeg.kneeDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[764 + i] = c_y[i];
  }

  i_x = bus->leftLeg.footDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[772 + i] = i_y[i];
  }

  c_x = bus->leftLeg.footDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[774 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[782 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[790 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[798 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[806 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[814 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[822 + i] = c_y[i];
  }

  c_x = bus->leftLeg.shinJoint.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[830 + i] = c_y[i];
  }

  c_x = bus->leftLeg.shinJoint.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[838 + i] = c_y[i];
  }

  c_x = bus->leftLeg.tarsusJoint.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[846 + i] = c_y[i];
  }

  c_x = bus->leftLeg.tarsusJoint.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[854 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footJoint.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[862 + i] = c_y[i];
  }

  c_x = bus->leftLeg.footJoint.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[870 + i] = c_y[i];
  }

  e_x = bus->leftLeg.medullaCounter;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[878] = e_y;
  i_x = bus->leftLeg.medullaCpuLoad;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[879 + i] = i_y[i];
  }

  e_x = bus->leftLeg.reedSwitchState;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[881] = e_y;
  i_x = bus->rightLeg.hipRollDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[882 + i] = i_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[884 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[892 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[900 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[908 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[916 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[924 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipRollDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[932 + i] = c_y[i];
  }

  i_x = bus->rightLeg.hipYawDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[940 + i] = i_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[942 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[950 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[958 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[966 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[974 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[982 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipYawDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[990 + i] = c_y[i];
  }

  i_x = bus->rightLeg.hipPitchDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[998 + i] = i_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1000 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1008 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1016 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1024 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1032 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1040 + i] = c_y[i];
  }

  c_x = bus->rightLeg.hipPitchDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1048 + i] = c_y[i];
  }

  i_x = bus->rightLeg.kneeDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[1056 + i] = i_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1058 + i] = c_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1066 + i] = c_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1074 + i] = c_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1082 + i] = c_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1090 + i] = c_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1098 + i] = c_y[i];
  }

  c_x = bus->rightLeg.kneeDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1106 + i] = c_y[i];
  }

  i_x = bus->rightLeg.footDrive.statusWord;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[1114 + i] = i_y[i];
  }

  c_x = bus->rightLeg.footDrive.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1116 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footDrive.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1124 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footDrive.torque;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1132 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footDrive.driveTemperature;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1140 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footDrive.dcLinkVoltage;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1148 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footDrive.torqueLimit;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1156 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footDrive.gearRatio;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1164 + i] = c_y[i];
  }

  c_x = bus->rightLeg.shinJoint.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1172 + i] = c_y[i];
  }

  c_x = bus->rightLeg.shinJoint.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1180 + i] = c_y[i];
  }

  c_x = bus->rightLeg.tarsusJoint.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1188 + i] = c_y[i];
  }

  c_x = bus->rightLeg.tarsusJoint.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1196 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footJoint.position;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1204 + i] = c_y[i];
  }

  c_x = bus->rightLeg.footJoint.velocity;
  memcpy((void *)&c_y[0], (void *)&c_x, (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1212 + i] = c_y[i];
  }

  e_x = bus->rightLeg.medullaCounter;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[1220] = e_y;
  i_x = bus->rightLeg.medullaCpuLoad;
  memcpy((void *)&i_y[0], (void *)&i_x, (unsigned int)((size_t)2 * sizeof
          (unsigned char)));
  for (i = 0; i < 2; i++) {
    bytes[1221 + i] = i_y[i];
  }

  e_x = bus->rightLeg.reedSwitchState;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[1223] = e_y;
  e_x = bus->isCalibrated;
  memcpy((void *)&e_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bytes[1224] = e_y;
  for (i = 0; i < 4; i++) {
    k_x[i] = bus->messages[i];
  }

  memcpy((void *)&c_y[0], (void *)&k_x[0], (unsigned int)((size_t)8 * sizeof
          (unsigned char)));
  for (i = 0; i < 8; i++) {
    bytes[1225 + i] = c_y[i];
  }
}

void unpack_cassie_out_t(const unsigned char bytes[1233], cassie_out_t *bus)
{
  int i;
  unsigned char x[24];
  unsigned char b_x[84];
  double y;
  unsigned char c_x[8];
  unsigned int b_y;
  unsigned char d_x[4];
  unsigned char e_x;
  unsigned char c_y;
  unsigned char f_x[96];
  unsigned char g_x[32];
  unsigned char h_x[128];
  unsigned short d_y;
  unsigned char i_x[2];
  short e_y[4];
  for (i = 0; i < 24; i++) {
    x[i] = bytes[i];
  }

  memcpy((void *)&bus->pelvis.targetPc.etherCatStatus[0], (void *)&x[0],
         (unsigned int)((size_t)6 * sizeof(int)));
  memcpy(&b_x[0], &bytes[24], 84U * sizeof(unsigned char));
  memcpy((void *)&bus->pelvis.targetPc.etherCatNotifications[0], (void *)&b_x[0],
         (unsigned int)((size_t)21 * sizeof(int)));
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 108];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.targetPc.taskExecutionTime = y;
  for (i = 0; i < 4; i++) {
    d_x[i] = bytes[i + 116];
  }

  memcpy((void *)&b_y, (void *)&d_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned int)));
  bus->pelvis.targetPc.overloadCounter = b_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 120];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.targetPc.cpuTemperature = y;
  e_x = bytes[128];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.battery.dataGood = (c_y != 0);
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 129];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.battery.stateOfCharge = y;
  memcpy(&f_x[0], &bytes[137], 96U * sizeof(unsigned char));
  memcpy((void *)&bus->pelvis.battery.voltage[0], (void *)&f_x[0], (unsigned int)
         ((size_t)12 * sizeof(double)));
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 233];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.battery.current = y;
  for (i = 0; i < 32; i++) {
    g_x[i] = bytes[i + 241];
  }

  memcpy((void *)&bus->pelvis.battery.temperature[0], (void *)&g_x[0], (unsigned
          int)((size_t)4 * sizeof(double)));
  e_x = bytes[273];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.radio.radioReceiverSignalGood = (c_y != 0);
  e_x = bytes[274];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.radio.receiverMedullaSignalGood = (c_y != 0);
  memcpy(&h_x[0], &bytes[275], sizeof(unsigned char) << 7);
  memcpy((void *)&bus->pelvis.radio.channel[0], (void *)&h_x[0], (unsigned int)
         ((size_t)16 * sizeof(double)));
  e_x = bytes[403];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.vectorNav.dataGood = (c_y != 0);
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 404];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->pelvis.vectorNav.vpeStatus = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 406];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.vectorNav.pressure = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 414];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.vectorNav.temperature = y;
  for (i = 0; i < 24; i++) {
    x[i] = bytes[i + 422];
  }

  memcpy((void *)&bus->pelvis.vectorNav.magneticField[0], (void *)&x[0],
         (unsigned int)((size_t)3 * sizeof(double)));
  for (i = 0; i < 24; i++) {
    x[i] = bytes[i + 446];
  }

  memcpy((void *)&bus->pelvis.vectorNav.angularVelocity[0], (void *)&x[0],
         (unsigned int)((size_t)3 * sizeof(double)));
  for (i = 0; i < 24; i++) {
    x[i] = bytes[i + 470];
  }

  memcpy((void *)&bus->pelvis.vectorNav.linearAcceleration[0], (void *)&x[0],
         (unsigned int)((size_t)3 * sizeof(double)));
  for (i = 0; i < 32; i++) {
    g_x[i] = bytes[i + 494];
  }

  memcpy((void *)&bus->pelvis.vectorNav.orientation[0], (void *)&g_x[0],
         (unsigned int)((size_t)4 * sizeof(double)));
  e_x = bytes[526];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.medullaCounter = c_y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 527];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->pelvis.medullaCpuLoad = d_y;
  e_x = bytes[529];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.bleederState = (c_y != 0);
  e_x = bytes[530];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.leftReedSwitchState = (c_y != 0);
  e_x = bytes[531];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->pelvis.rightReedSwitchState = (c_y != 0);
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 532];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->pelvis.vtmTemperature = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 540];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->leftLeg.hipRollDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 542];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 550];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 558];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 566];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 574];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 582];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 590];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipRollDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 598];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->leftLeg.hipYawDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 600];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 608];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 616];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 624];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 632];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 640];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 648];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipYawDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 656];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->leftLeg.hipPitchDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 658];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 666];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 674];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 682];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 690];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 698];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 706];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.hipPitchDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 714];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->leftLeg.kneeDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 716];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 724];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 732];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 740];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 748];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 756];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 764];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.kneeDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 772];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->leftLeg.footDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 774];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 782];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 790];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 798];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 806];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 814];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 822];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footDrive.gearRatio = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 830];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.shinJoint.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 838];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.shinJoint.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 846];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.tarsusJoint.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 854];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.tarsusJoint.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 862];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footJoint.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 870];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->leftLeg.footJoint.velocity = y;
  e_x = bytes[878];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->leftLeg.medullaCounter = c_y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 879];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->leftLeg.medullaCpuLoad = d_y;
  e_x = bytes[881];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->leftLeg.reedSwitchState = (c_y != 0);
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 882];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->rightLeg.hipRollDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 884];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 892];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 900];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 908];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 916];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 924];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 932];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipRollDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 940];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->rightLeg.hipYawDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 942];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 950];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 958];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 966];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 974];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 982];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 990];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipYawDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 998];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->rightLeg.hipPitchDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1000];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1008];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1016];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1024];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1032];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1040];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1048];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.hipPitchDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 1056];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->rightLeg.kneeDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1058];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1066];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1074];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1082];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1090];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1098];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1106];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.kneeDrive.gearRatio = y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 1114];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->rightLeg.footDrive.statusWord = d_y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1116];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1124];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1132];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.torque = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1140];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.driveTemperature = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1148];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.dcLinkVoltage = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1156];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.torqueLimit = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1164];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footDrive.gearRatio = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1172];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.shinJoint.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1180];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.shinJoint.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1188];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.tarsusJoint.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1196];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.tarsusJoint.velocity = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1204];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footJoint.position = y;
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1212];
  }

  memcpy((void *)&y, (void *)&c_x[0], (unsigned int)((size_t)1 * sizeof(double)));
  bus->rightLeg.footJoint.velocity = y;
  e_x = bytes[1220];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->rightLeg.medullaCounter = c_y;
  for (i = 0; i < 2; i++) {
    i_x[i] = bytes[i + 1221];
  }

  memcpy((void *)&d_y, (void *)&i_x[0], (unsigned int)((size_t)1 * sizeof
          (unsigned short)));
  bus->rightLeg.medullaCpuLoad = d_y;
  e_x = bytes[1223];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->rightLeg.reedSwitchState = (c_y != 0);
  e_x = bytes[1224];
  memcpy((void *)&c_y, (void *)&e_x, (unsigned int)((size_t)1 * sizeof(unsigned
           char)));
  bus->isCalibrated = (c_y != 0);
  for (i = 0; i < 8; i++) {
    c_x[i] = bytes[i + 1225];
  }

  memcpy((void *)&e_y[0], (void *)&c_x[0], (unsigned int)((size_t)4 * sizeof
          (short)));
  for (i = 0; i < 4; i++) {
    bus->messages[i] = convert_to_enum_DiagnosticCodes(e_y[i]);
  }
}
