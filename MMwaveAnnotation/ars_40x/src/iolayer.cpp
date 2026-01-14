#include "iolayer.hpp"
#include <iostream>
// this layer replaces the original socket can

/*
if failed to receive multicast, see
cat /proc/sys/net/ipv4/conf/<nic name>/rp_filter
cat /proc/sys/net/ipv4/conf/all/rp_filter

use
sudo sysctl -w net.ipv4.conf.<nic name>rp_filter=0
to disable filter


*/

namespace iolayer
{
/**
 * @brief Construct a new layer socket::layer socket object
 *
 * @param name
 * @param group_ip
 * @param port
 */
layer_socket::layer_socket(std::string group_ip, int port, std::string remote_ip, std::string local_ip)
{
	int err;
	this->sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (this->sock < 0)
	{
		std::cerr << "failed to create socket" << std::endl;
		return;
	}

	err = 1;
	setsockopt(this->sock, SOL_SOCKET, SO_REUSEADDR, &err, sizeof(err));

	std::cout << "socket initialized" << std::endl;

	struct sockaddr_in local_addr;
	memset(&local_addr, 0x00, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = INADDR_ANY;
	local_addr.sin_port = htons(port);
	err = bind(this->sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
	if (err < 0)
	{
		std::cerr << "failed to bind socket to local address" << std::endl;
		close(this->sock);
		this->sock = -1;
		return;
	}

	std::cout << "socket binded" << std::endl;

	err = 255;
	setsockopt(this->sock, IPPROTO_IP, IP_MULTICAST_TTL, &err, sizeof(err));

	this->mreq.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
	this->mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());

	err =
		setsockopt(this->sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &this->mreq, sizeof(this->mreq));
	if (err < 0)
	{
		std::cerr << "failed to join multicast group" << std::endl;
		close(this->sock);
		this->sock = -1;
		return;
	}

	std::cout << "socket option set" << std::endl;

	// set timeout
	struct timeval timeout = {0, 10000};
	setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
	timeout.tv_usec = 0;
	timeout.tv_sec = 1;
	setsockopt(this->sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

	this->remote_addr.sin_family = AF_INET;
	this->remote_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
	this->remote_addr.sin_port = htons(port);

	std::cout << "layer socket initialized" << std::endl;
}

// add this defination in CMakeLists.txt if radar was connected to CAN2
#ifndef USE_SENCONDARY_PORT

layer_socket::layer_socket(char *name) : layer_socket(MULTI_IP, PORT_1, REMOTE_IP, LOCAL_IP)
{
	std::cout << "can socket replaced with udp multicast at port 1" << std::endl;
}

layer_socket::layer_socket(std::string name) : layer_socket(MULTI_IP, PORT_1, REMOTE_IP, LOCAL_IP)
{
	std::cout << "can socket replaced with udp multicast at port 1" << std::endl;
}

#else

layer_socket::layer_socket(char *name) : layer_socket(MULTI_IP, PORT_2, REMOTE_IP)
{
	std::cout << "can socket replaced with udp multicast at port 2" << std::endl;
}

layer_socket::layer_socket(std::string name) : layer_socket(MULTI_IP, PORT_2, REMOTE_IP)
{
	std::cout << "can socket replaced with udp multicast at port 2" << std::endl;
}

#endif

/**
 * @brief Destroy the layer socket::layer socket object
 *
 */
layer_socket::~layer_socket()
{
	if (this->sock > 0)
	{
		setsockopt(this->sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &this->mreq, sizeof(this->mreq));
		close(this->sock);
	}
}

/**
 * @brief write can frame, ars40x only accept standard frame
 *
 * @param id extended frame id
 * @param dlc data length, from 0 to 8
 * @param data data buffer
 * @return int bytes written, or -1 for errors(sock not initialized, or sendto() failed)
 */
int layer_socket::write(uint32_t id, int dlc, uint8_t *data)
{
	if (this->sock < 0) return -1;

	if (dlc > 8)
	{
		dlc = 8;
	}
	eth_side_frame frame;
	frame.id = htonl(id);
	frame.len = htonl(dlc);
	memcpy(frame.data, data, dlc);
	return sendto(this->sock, &frame, sizeof(frame), 0, (const sockaddr *)&this->remote_addr,
				  sizeof(this->remote_addr));
}

/**
 * @brief read can frame from trdp network
 *
 * @param id buffer for frame id
 * @param dlc data length buffer
 * @param data data buffer
 * @return int 0: no frames were received, >0: received a frame, <0: error
 */
int layer_socket::read(uint32_t *id, int *dlc, uint8_t *data)
{
	if (this->sock < 0) return -1;

	uint8_t buf[128];
	eth_side_frame *frame;
	struct sockaddr_in addr;
	socklen_t addr_len = sizeof(addr);
	int len;
	len = recvfrom(this->sock, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &addr_len);
	if (len != sizeof(eth_side_frame))
	{
		return len < 0 ? len : 0;
	}
	frame = (eth_side_frame *)buf;
	*id = ntohl(frame->id);
	*dlc = ntohl(frame->len);
	if (*dlc > 8)
	{
		// bad thing
		return 0;
	}
	memcpy(data, frame->data, *dlc);
	return *dlc;
}
} // namespace iolayer
