#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <stdio.h> //printf
#include <errno.h>
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent
#include "Command_Structs.h"

/**
    TCP Client class
*/
class udp_comms
{
private:
    int sock;
    struct sockaddr_in server;
    struct sockaddr_in remaddr;

    bool server_conn();
    bool client_conn();

    bool receive(char* buff, unsigned int num_bytes);
    bool send(char* buff, unsigned int numBytes);


    bool m_bClient;
    unsigned int PORT;

public:
    udp_comms(bool bClient, unsigned int port);
    bool conn();

    bool receive_cassie_outputs(cassie_outputs_t* data);
    bool receive_cassie_inputs(cassie_inputs_t* data);

    bool send_cassie_inputs(cassie_inputs_t data);
    bool send_cassie_outputs(cassie_outputs_t data);

};

#endif
