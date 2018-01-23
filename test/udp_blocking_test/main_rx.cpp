#include "udp_comms.h"

#include <iostream>
#include <fstream>
#include <sys/time.h>

using namespace std;

int main()
{

   udp_comms* comms = new udp_comms("127.0.0.1", "127.0.0.1", 8880);

   if (!comms->conn())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   printf("connected\n");

   logFile.open("log.csv");

   cassie_out_t sensors;

   comms->send_cassie_outputs(&sensors);

   return 0;
}
