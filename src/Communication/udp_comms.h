#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <stdio.h> //printf
#include <errno.h>
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent
#include "cassie_out_t_types.h"
#include "cassie_user_in_t_types.h"
#include <string>
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

    bool receive(unsigned char* buff, unsigned int num_bytes);
    bool send(unsigned char* buff, unsigned int numBytes);


    bool m_bClient;
    unsigned int PORT;
    std::string ip_address;

public:
    udp_comms(bool bClient, unsigned int port, std::string ip_addr);
    bool conn();

//    bool send_double(double tx);
//    bool receive_double(double* rx);

    bool receive_cassie_outputs(cassie_out_t* data);
    bool receive_cassie_inputs(cassie_user_in_t* data);

    bool send_cassie_inputs(cassie_user_in_t data);
    bool send_cassie_outputs(cassie_out_t data);

};

#endif
