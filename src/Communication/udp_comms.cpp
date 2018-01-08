#include "udp_comms.h"
#include <stdint.h>
#include <cstring>

using namespace std;

#define SERVER "127.0.0.1"
#define BUFLEN 512

udp_comms::udp_comms(bool bClient, unsigned int port)
{
	m_bClient = bClient;
	sock = -1;
	PORT = port;
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
	if (inet_aton(SERVER , &server.sin_addr) == 0)
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
	if (inet_aton(SERVER , &server.sin_addr) == 0)
	{
		printf("inet_aton() failed\n");
		return false;
	}

	//pretty much the same as client but bind socket for server
	if( bind(sock, (struct sockaddr*)&server, sizeof(server) ) == -1)
	{
		printf("failed to bind socket...\n");
		return false;
	}


	return true;

}




/**
    Receive data from the connected host
 */
bool udp_comms::receive_cassie_outputs(cassie_outputs_t* data)
{
	unsigned int numBytes = sizeof(cassie_outputs_t);
	char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

//	printf("received outputs\n");

	memcpy(data, buff, numBytes);

	return true;
}

bool udp_comms::send_cassie_outputs(cassie_outputs_t data) {

	unsigned int numBytes = sizeof(cassie_outputs_t);
	char buff[numBytes];

	//convert byte order?

	memcpy(buff, &data, numBytes);
	if (!send(buff, numBytes))
		return false;

//	printf("sent outputs\n");

	return true;
}

bool udp_comms::receive_cassie_inputs(cassie_inputs_t* data)
{
	unsigned int numBytes = sizeof(cassie_inputs_t);
	char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

//	printf("received inputs\n");

	memcpy(data, buff, numBytes);

	return true;
}

bool udp_comms::send_cassie_inputs(cassie_inputs_t data) {

	unsigned int numBytes = sizeof(cassie_inputs_t);
	char buff[numBytes];

	//convert byte order?

	memcpy(buff, &data, numBytes);
	if (!send(buff, numBytes))
		return false;

//	printf("sent inputs\n");

	return true;
}

bool udp_comms::receive(char* buff, unsigned int num_bytes)
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
bool udp_comms::send(char* buff, unsigned int numBytes)
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
