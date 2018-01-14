#include "udp_comms.h"

#include <iostream>
#include <fstream>
#include <sys/time.h>

using namespace std;

ofstream logFile;

void log(cassie_out_t sensors)
{
	
	logFile << sensors.leftLeg.hipRollDrive.position << ",";
	logFile << sensors.leftLeg.hipRollDrive.velocity << ",";

	logFile << sensors.leftLeg.hipYawDrive.position << ",";
	logFile << sensors.leftLeg.hipYawDrive.velocity << ",";

	logFile << sensors.leftLeg.hipPitchDrive.position << ",";
	logFile << sensors.leftLeg.hipPitchDrive.velocity << ",";

	logFile << sensors.leftLeg.kneeDrive.position << ",";
	logFile << sensors.leftLeg.kneeDrive.velocity << ",";

	logFile << sensors.leftLeg.shinJoint.position << ",";
	logFile << sensors.leftLeg.shinJoint.velocity << ",";

	logFile << sensors.leftLeg.tarsusJoint.position << ",";
	logFile << sensors.leftLeg.tarsusJoint.velocity << ",";

	logFile << sensors.leftLeg.footJoint.position << ",";
	logFile << sensors.leftLeg.footJoint.velocity << ",";

	logFile << sensors.rightLeg.hipRollDrive.position << ",";
	logFile << sensors.rightLeg.hipRollDrive.velocity << ",";

	logFile << sensors.rightLeg.hipYawDrive.position << ",";
	logFile << sensors.rightLeg.hipYawDrive.velocity << ",";

	logFile << sensors.rightLeg.hipPitchDrive.position << ",";
	logFile << sensors.rightLeg.hipPitchDrive.velocity << ",";

	logFile << sensors.rightLeg.kneeDrive.position << ",";
	logFile << sensors.rightLeg.kneeDrive.velocity << ",";

	logFile << sensors.rightLeg.shinJoint.position << ",";
	logFile << sensors.rightLeg.shinJoint.velocity << ",";

	logFile << sensors.rightLeg.tarsusJoint.position << ",";
	logFile << sensors.rightLeg.tarsusJoint.velocity << ",";

	logFile << sensors.rightLeg.footJoint.position << ",";
	logFile << sensors.rightLeg.footJoint.velocity << ",";


	for (int i = 0; i < 3; i++)
		logFile << sensors.pelvis.vectorNav.linearAcceleration[i] << ",";
	for (int i = 0; i < 3; i++)
		logFile << sensors.pelvis.vectorNav.angularVelocity[i] << ",";
	for (int i = 0; i < 3; i++)
		logFile << sensors.pelvis.vectorNav.magneticField[i] << ",";
	for (int i = 0; i < 3; i++)
		logFile << sensors.pelvis.vectorNav.orientation[i] << ",";
	logFile << sensors.pelvis.vectorNav.orientation[3] << endl;
}

int main()
{

   udp_comms* tx_comms = new udp_comms(false, 25001, "10.10.10.100");

   if (!tx_comms->conn())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   udp_comms* rx_comms = new udp_comms(false, 25000, "10.10.10.100");

   if (!rx_comms->conn())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   printf("connected\n");

   logFile.open("log.csv");

   cassie_out_t sensors;

   uint8_t byte1, byte2;

   while (true)
   {
      rx_comms->receive_cassie_outputs(&sensors, &byte1, &byte2);
      log(sensors);
   }

   return 0;
}
