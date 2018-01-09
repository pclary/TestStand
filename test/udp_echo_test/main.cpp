#include "udp_comms.h"

int main()
{

   udp_comms* comms = new udp_comms(true, 8001, "10.10.10.3");

   if (!comms->conn())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   double data = 0.0;
   while (true)
   {
      comms->send_double(data);
      comms->receive_double(&data);
      printf("rx: %f\n", data);
      data += 1.0;
   }

   return 0;
}
