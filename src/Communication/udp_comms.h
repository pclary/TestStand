#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <stdio.h> //printf
#include <errno.h>
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <sys/ioctl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> //hostent
#include <unistd.h>
#include <fcntl.h>
#include "Command_Structs.h"
#include "SharedRobotDefinitions.h"
#include "CommandInterface.h"
#include <string>
/**
    TCP Client class
*/
class udp_comms
{
private:
    int sock;
    bool m_bBindFailed;
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;

    bool receive(unsigned char* buff, unsigned int num_bytes);
    bool transmit(unsigned char* buff, unsigned int numBytes);


    sockaddr_in make_sockaddr_in(const char *addr_str, unsigned short port);

    unsigned int PORT;
    std::string local_address_str;
    std::string remote_address_str;

public:
    udp_comms(std::string local_addr, std::string remote_addr, unsigned int port);
    bool conn();

    bool rcv_data_available();

    bool receive_cassie_outputs(cassie_out_t* data);
    bool receive_cassie_inputs(cassie_user_in_t* data);
    bool receive_telemetry(telemetry_t* t);
    bool receive_state_info(CommandInterface::StateInfo_Struct* s);
    bool receive_policy_params(CommandInterface::policy_params_t* s);

    bool send_cassie_inputs(cassie_user_in_t data);
    bool send_cassie_outputs(cassie_out_t data);
    bool send_telemetry(telemetry_t t);
    bool send_state_info(CommandInterface::StateInfo_Struct s);
    bool send_policy_params(CommandInterface::policy_params_t s);

};

#endif
