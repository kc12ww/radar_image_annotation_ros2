#include "rclcpp/rclcpp.hpp"
#include <arpa/inet.h>
#include <cstdint>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
namespace iolayer
{
// group ip string
#define MULTI_IP "239.255.0.6"
// default remote ip string
#define REMOTE_IP "10.0.1.6"
// default local ip string
#define LOCAL_IP "10.0.1.20"
// port for can 1
#define PORT_1 6780
// port for can 2
#define PORT_2 (REMOTE_PORT_1 + 1)

typedef struct
{
	uint32_t id;	  // id of can frame, converted to host endian
	uint32_t len;	  // data length, converted to host endian
	uint8_t data[8];  // data bytes, might be network endian
	uint32_t padding; // padding bytes
} eth_side_frame;

class layer_socket
{
  private:
	// multicast ip address
	std::string multi_ip;
	// mulicast port
	int multi_port;
	// remote host ip address
	std::string remote_ip;
	// join or drop multicast request
	struct ip_mreq mreq;
	// remote address struct
	struct sockaddr_in remote_addr;

  public:
	// socket no
	int sock = -1;
	layer_socket(std::string group_ip, int port, std::string remote_ip, std::string local_ip);
	layer_socket(char *name);
	layer_socket(std::string name);
	~layer_socket();
	int write(uint32_t id, int dlc, uint8_t *data);
	int read(uint32_t *id, int *dlc, uint8_t *data);
};

} // namespace iolayer
