#include "udp_comms.h"
#include <stdint.h>
#include <cstring>
#include "cassie_user_in_t.h"
#include "cassie_out_t.h"

using namespace std;

#define BUFLEN 512

udp_comms::udp_comms(std::string local_addr, std::string remote_addr, unsigned int port)
{
	sock = -1;
	PORT = port;
	local_address_str = local_addr;
	remote_address_str = remote_addr;
	m_bBindFailed = false;
}


// Construct an address struct given an address string and port number
sockaddr_in udp_comms::make_sockaddr_in(const char *addr_str, unsigned short port)
{
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	inet_pton(AF_INET, addr_str, &addr.sin_addr);
	addr.sin_port = htons(port);
	memset(&addr.sin_zero, 0, sizeof(addr.sin_zero));
	return addr;
}


/**
    Connect to a host on a certain port number
 */
bool udp_comms::conn()
{

	local_addr = make_sockaddr_in(local_address_str.c_str(), PORT);
	remote_addr = make_sockaddr_in(remote_address_str.c_str(), PORT);

	sock = socket(AF_INET , SOCK_DGRAM , 0);
	if (sock == -1)
	{
		printf("Could not create socket\n");
		return false;
	}

	// Bind to local address
	if (-1 == bind(sock,
			(struct sockaddr *) &local_addr,
			sizeof (struct sockaddr))) {
		perror("Error binding to local address: ");
		m_bBindFailed = true;
#if EMBEDDED
		close(sock);
		return false;
#endif
	}

	// Connect to remote address
	if (-1 == connect(sock,
			(struct sockaddr *) &remote_addr,
			sizeof (struct sockaddr))) {
		perror("Error connecting to remote address: ");
		close(sock);
		return false;
	}

	fcntl(sock, O_NONBLOCK);

	return true;
}

/**
    Receive data from the connected host
 */
bool udp_comms::receive_cassie_outputs(cassie_out_t* data)
{
	unsigned int numBytes = 1235;//sizeof(cassie_out_t) + 2;
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	unpack_cassie_out_t(&(buff[2]), data);

	return true;
}

bool udp_comms::send_cassie_outputs(cassie_out_t data) {

	unsigned int numBytes = 1235;//sizeof(cassie_out_t) + 2;
	unsigned char buff[numBytes];

	pack_cassie_out_t(&data, &(buff[2]));

	if (!transmit(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::receive_cassie_inputs(cassie_user_in_t* data)
{
	unsigned int numBytes = 100;//sizeof(cassie_user_in_t);
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	unpack_cassie_user_in_t(&(buff[2]), data);

	return true;
}

bool udp_comms::send_cassie_inputs(cassie_user_in_t data) {

	unsigned int numBytes = 100;//sizeof(cassie_user_in_t) + 2;
	unsigned char buff[numBytes];

	pack_cassie_user_in_t(&data, &(buff[2]));

	if (!transmit(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::send_telemetry(telemetry_t t)
{
	unsigned int numBytes = sizeof(telemetry_t);
	unsigned char buff[numBytes];

	memcpy(buff, &t, numBytes);

	if (!transmit(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::receive_telemetry(telemetry_t* t)
{
	unsigned int numBytes = sizeof(telemetry_t);
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	memcpy(t, buff, numBytes);

	return true;
}

bool udp_comms::send_state_info(CommandInterface::StateInfo_Struct s)
{
	unsigned int numBytes = sizeof(CommandInterface::StateInfo_Struct);
	unsigned char buff[numBytes];

	memcpy(buff, &s, numBytes);

	if (!transmit(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::receive_state_info(CommandInterface::StateInfo_Struct* s)
{
	unsigned int numBytes = sizeof(CommandInterface::StateInfo_Struct);
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	memcpy(s, buff, numBytes);

	return true;
}

bool udp_comms::send_policy_params(CommandInterface::policy_params_t s)
{
	unsigned int numBytes = sizeof(CommandInterface::policy_params_t);
	unsigned char buff[numBytes];

	memcpy(buff, &s, numBytes);

	if (!transmit(buff, numBytes))
		return false;

	return true;
}

bool udp_comms::receive_policy_params(CommandInterface::policy_params_t* s)
{
	unsigned int numBytes = sizeof(CommandInterface::policy_params_t);
	unsigned char buff[numBytes];

	if (!receive(buff, numBytes))
		return false;

	memcpy(s, buff, numBytes);

	return true;
}

bool udp_comms::rcv_data_available()
{
	struct pollfd fd;
	fd.fd = sock; fd.events = POLLIN; fd.revents = 0;
	return (poll(&fd, 1, 0) > 0);
}

bool udp_comms::receive(unsigned char* buff, unsigned int num_bytes)
{
//	printf("Receiving: %u bytes\n", num_bytes);
//	// Poll for a new packet of the correct length
//
//#ifndef EMBEDDED
//	socklen_t rcv_len = num_bytes;
//	if (m_bBindFailed)
//	{
//		if( recvfrom(sock , buff , num_bytes , 0, (struct sockaddr *) &local_addr, &rcv_len) < 0)
//		{
//			printf("recv failed: %s\n", strerror(errno));
//			return false;
//		}
//	}
//	else
//	{
//		if( recvfrom(sock , buff , num_bytes , 0, (struct sockaddr *) &remote_addr, &rcv_len) < 0)
//		{
//			printf("recv failed: %s\n", strerror(errno));
//			return false;
//		}
//	}
//	return true;
//#endif



	unsigned int nbytes;
	do {
		// Wait if no packets are available
		struct pollfd fd;
		fd.fd = sock; fd.events = POLLIN; fd.revents = 0;
		while (!poll(&fd, 1, 0)) {}

		// Get newest valid packet in RX buffer
		// Does not use sequence number for determining newest packet
		while (poll(&fd, 1, 0)) {
			ioctl(sock, FIONREAD, &nbytes);
			if (num_bytes == nbytes)
				nbytes = recv(sock, buff, num_bytes, 0);
			else
				recv(sock, buff, 0, 0); // Discard packet
		}
	} while (num_bytes != nbytes);

	return true;
}

/*
    Send data to the connected host
 */
bool udp_comms::transmit(unsigned char* buff, unsigned int numBytes)
{
//	printf("Sending: %u bytes\n", numBytes);
//
//#ifndef EMBEDDED
//	if (m_bBindFailed)
//	{
//		if( sendto(sock , buff , numBytes , 0, (struct sockaddr *) &remote_addr, sizeof(local_addr)) < 0)
//		{
//			printf("send failed: %s\n", strerror(errno));
//			return false;
//		}
//	}
//	else
//	{
//		if( sendto(sock , buff , numBytes , 0, (struct sockaddr *) &local_addr, sizeof(remote_addr)) < 0)
//		{
//			printf("send failed: %s\n", strerror(errno));
//			return false;
//		}
//	}
//	return true;
//#endif

	send(sock, buff, numBytes, 0);
	return true;
}
