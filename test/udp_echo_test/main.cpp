#include "udp_comms.h"


int main()
{

   udp_comms* comms = new udp_comms(false, 8001, "10.10.10.5");

   if (!comms->conn())
   {
      printf("Failed to connect... returning\n");
      return -1;
   }

   printf("connected\n");


   cassie_out_t sensors;

   while (true)
   {
      comms->receive_cassie_outputs(&sensors);
      printf("rx\n");
   }

   return 0;
}
