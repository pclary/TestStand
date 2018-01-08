#ifndef COMMAND_STRUCTS
#define COMMAND_STRUCTS

#pragma pack(push, 1)
typedef struct {
  double vectorNavOrientation[4];
  double vectorNavAngularVelocity[3];
  double vectorNavLinearAcceleration[3];
  double vectorNavMagneticField[3];
  double vectorNavPressure;
  double vectorNavTemperature;

  double motorPosition[10];
  double motorVelocity[10];
  double jointPosition[4];
  double jointVelocity[4];

  double radio[16];

  double stateOfCharge;
  double status;
}cassie_outputs_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  double radio[9];
  double torque[10];
}cassie_inputs_t;
#pragma pack(pop)


#endif
