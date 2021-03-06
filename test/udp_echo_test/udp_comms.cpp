#include "udp_comms.h"
#include <stdint.h>
#include <cstring>
#include "cassie_user_in_t.h"
#include "cassie_out_t.h"

using namespace std;

#define BUFLEN 512

udp_comms::udp_comms(bool bClient, unsigned int port, string ip_addr)
{
	m_bClient = bClient;
	sock = -1;
	PORT = port;
	ip_address = ip_addr;
}

/**
    Connect to a host on a certain port number
 */
bool udp_comms::conn()
{
	if (m_bClient)
		return client_conn();
	else
		return server_conn();
}

bool udp_comms::client_conn()
{
	//create socket if it is not already created
	if(sock == -1)
	{
		//Create socket
		sock = socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP);
		if (sock == -1)
		{
			printf("Could not create socket\n");
		}
	}
	else    {   /* OK , nothing */  }

	server.sin_family = AF_INET;
	server.sin_port = htons( PORT );
	if (inet_aton(ip_address.c_str() , &server.sin_addr) == 0)
	{
		printf("inet_aton() failed\n");
		return false;
	}

	return true;
}

bool udp_comms::server_conn()
{

	//create socket if it is not already created
	if(sock == -1)
	{
		//Create socket
		sock = socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP);
		if (sock == -1)
		{
			printf("Could not create socket\n");
		}
	}
	else    {   /* OK , nothing */  }

	//	server.sin_addr.s_addr = htonl(INADDR_ANY);
	server.sin_family = AF_INET;
	server.sin_port = htons( PORT );
	if (inet_aton(ip_address.c_str() , &server.sin_addr) == 0)
	{
		printf("inet_aton() failed\n");
		return false;
	}

	//pretty much the same as client but bind socket for server
	if( bind(sock, (struct sockaddr*)&server, sizeof(server) ) == -1)
	{
		printf("failed to bind socket... %s\n", strerror(errno));
		return false;
	}


	return true;

}

//bool udp_comms::receive_double(double *rx)
//{
//	unsigned int numBytes = sizeof(double);
//	char buff[numBytes];
//	if (!receive(buff, numBytes))
//		return false;
//
//	memcpy(rx, buff, numBytes);
//
//	return true;
//}
//
//bool udp_comms::send_double(double tx)
//{
//	unsigned int numBytes = sizeof(double);
//	char buff[numBytes];
//
//	memcpy(buff, &tx, numBytes);
//	if (!send(buff, numBytes))
//		return false;
//
//	return true;
//}


/**
    Receive data from the connected host
 */
bool udp_comms::receive_cassie_outputs(cassie_out_t* data, uint8_t* byte1, uint8_t* byte2)
{
	unsigned int numBytes = sizeof(cassie_out_t) + 2;
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	unpack_cassie_out_t(&(buff[2]), data);

	*byte1 = buff[0];
	*byte2 = buff[1];

	return true;
}

bool udp_comms::send_cassie_outputs(cassie_out_t data) {

	unsigned int numBytes = sizeof(cassie_out_t);
	unsigned char buff[numBytes];

	pack_cassie_out_t(&data, buff);

	if (!send(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::receive_cassie_inputs(cassie_user_in_t* data)
{
	unsigned int numBytes = sizeof(cassie_user_in_t);
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	unpack_cassie_user_in_t(buff, data);

	return true;
}

bool udp_comms::send_cassie_inputs(cassie_user_in_t data, uint8_t byte1, uint8_t byte2) {

	unsigned int numBytes = sizeof(cassie_user_in_t) + 2;
	unsigned char buff[numBytes];

	pack_cassie_user_in_t(&data, &(buff[2]));

	buff[0] = byte2;
	buff[1] = byte1;

	if (!send(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::receive(unsigned char* buff, unsigned int num_bytes)
{
	socklen_t rcv_len = num_bytes;
	//Receive a reply from the server
	if (!m_bClient)
	{
		if( recvfrom(sock , buff , num_bytes , 0, (struct sockaddr *) &remaddr, &rcv_len) < 0)
		{
			printf("recv failed: %s\n", strerror(errno));
			return false;
		}
	}
	else
	{
		if( recvfrom(sock , buff , num_bytes , 0, (struct sockaddr *) &server, &rcv_len) < 0)
		{
			printf("recv failed: %s\n", strerror(errno));
			return false;
		}
	}

	return true;
}

/*
    Send data to the connected host
 */
bool udp_comms::send(unsigned char* buff, unsigned int numBytes)
{
	if (!m_bClient)
	{
		//Send some data
		if( sendto(sock , buff ,numBytes , 0, (struct sockaddr *) &remaddr, sizeof(server)) < 0)
		{
			printf("Send failed : %s\n", strerror(errno));
			return false;
		}
	}
	else
	{
		//Send some data
		if( sendto(sock , buff , numBytes , 0, (struct sockaddr *) &server, sizeof(server)) < 0)
		{
			printf("Send failed : %s\n", strerror(errno));
			return false;
		}
	}

	return true;
}
