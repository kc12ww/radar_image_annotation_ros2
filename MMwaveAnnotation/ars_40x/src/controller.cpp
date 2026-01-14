#include "controller.hpp"

controller::controller(std::string group_ip, int port, std::string remote_ip,
					   std::string local_ip, int id)
	: layer_socket(group_ip, port, remote_ip, local_ip)
{
	this->radar_id = id;
	std::cout << "initialized layer socket" << std::endl;
}
controller::~controller()
{
}

/**
 * @brief read frame
 *
 * @param id can id buffer
 * @param dlc data length buffer
 * @param data data buffer
 * @return int >0: received, <0: timeout
 */
int controller::read_frame(uint32_t *id, int *dlc, uint8_t data[8])
{
	return this->read(id + this->radar_id, dlc, data);
}

/**
 * @brief read frame
 *
 * @param id can id buffer
 * @param dlc data length buffer
 * @param reg data buffer in host endian
 * @return int >0: received, <0: timeout
 */
int controller::read_frame(uint32_t *id, int *dlc, uint64_t *reg)
{
	uint8_t *buf = (uint8_t *)reg;
	if (htonl(0x01) == 0x01)
		// host is big endian
		return this->read(id + this->radar_id, dlc, buf);
	// host is little endian
	int err = this->read(id + this->radar_id, dlc, buf);
	if (err <= 0) return err;
	// to little endian
	*reg = swap_endian(buf);
	return err;
}

/**
 * @brief wait for a frame of given id
 *
 * @param wait_id id waiting for
 * @param data data buffer, if given, recv data will be written to here
 * @param retry how many times do we retry
 * @return int -1: timeout, >0: received
 */
int controller::wait_frame(can_id wait_id, uint8_t data[8], int retry)
{
	uint32_t recv_id = 0;
	int recv_dlc;
	uint8_t recv_data[8];
	do
	{
		this->read(&recv_id, &recv_dlc, recv_data);
		recv_id -= this->radar_id;
		if (recv_id != wait_id)
		{
			retry--;
			continue;
		}
		if (data != nullptr) memcpy(data, recv_data, recv_dlc);
		return recv_dlc;
	} while (retry >= 0);
	return -1;
}

int controller::wait_frame(can_id wait_id, uint64_t *reg, int retry)
{
	uint8_t *buf = (uint8_t *)reg;
	int err = this->wait_frame(wait_id, buf, retry);
	if (err < 0)
	{
		return err;
	}
	*reg = ntoh64(buf);
	return err;
}

/**
 * @brief write a frame to radar, data organized as bytes
 *
 * @param id can id
 * @param dlc data length
 * @param data data buffer
 * @return int >0: how many data bytes written, <0: error
 */
int controller::write_frame(can_id id, int dlc, uint8_t data[8])
{
	return this->write(id + this->radar_id, dlc, data);
}

/**
 * @brief write a frame to radar, data organized as 64 bit uint
 *
 * @param id can id
 * @param dlc data length (bytes)
 * @param reg 64 bit data
 * @return int >0: how many data bytes written (reg msbs first), <0: error
 */
int controller::write_frame(can_id id, int dlc, uint64_t reg)
{
	uint8_t *buf = (uint8_t *)&reg;
	if (htonl(0x01) == 0x01)
		// big endian
		return this->write(id + this->radar_id, dlc, buf);
	// convert from host little endian to network big endian
	reg = swap_endian(buf);
	return this->write(id + this->radar_id, dlc, buf);
}

/**
 * @brief write specified radar config option
 *
 * @param conf config option
 * @param val value
 * @return int >0: success, <0: failed
 */
int controller::write_config(radar_conf conf, int val)
{

	uint64_t reg = 0;
	int err;
	// enable valid bit
	write_bits(reg, conf, 1, 1);
	switch (conf)
	{
		case radar_conf::SendExInfo:
			write_bits(reg, 19, 1, val);
			break;
		case radar_conf::SendQuality:
			write_bits(reg, 18, 1, val);
			break;
		case radar_conf::OutputType:
			write_bits(reg, 27, 2, val);
			break;
		case radar_conf::SensorId:
			write_bits(reg, 24, 3, val);
			break;
		case radar_conf::MaxDistance:
			write_bits(reg, 46, 10, val);
			break;
		case radar_conf::RadarPower:
			write_bits(reg, 29, 3, val);
			break;
		case radar_conf::StoreInNVM:
			write_bits(reg, 23, 1, val);
			break;
		case radar_conf::SortIndex:
			write_bits(reg, 20, 3, val);
			break;
		case radar_conf::CtrlRelay:
			write_bits(reg, 17, 1, val);
			break;
		case radar_conf::InvalidClusters:
			write_bits(reg, 0, 8, val);
			break;
		case radar_conf::RCSThreshold:
			write_bits(reg, 9, 3, val);
			break;
		default:
			return -2;
	}
	return this->write_config_reg(reg);
}

int controller::write_config_reg(uint64_t reg)
{
	return this->write_frame(can_id::RadarCfg, 8, reg);
}

/**
 * @brief write filter
 *
 * @param type 0: cluster mode, 1: object mode
 * @param index filter index
 * @param min min value of the filter
 * @param max max value of the filter
 * @return int >0: success, <0: failed
 */
int controller::write_filter(int type, radar_filter index, int active, int min, int max)
{
	uint64_t reg = 0;
	write_bits(reg, 63, 1, type);				  // type 1
	write_bits(reg, 59, 4, index);				  // index
	write_bits(reg, 57, 2, active ? 0x03 : 0x01); // active and valid
	write_bits(reg, 40, 12, min);				  // min
	write_bits(reg, 24, 12, max);				  // max
	return this->write_frame(can_id::FilterCfg, 5, reg);
}

/**
 * @brief TODO
 *
 * @param enable
 * @param point_1_x
 * @param point_1_y
 * @param point_2_x
 * @param point_2_y
 * @return int
 */
int controller::write_polygon(int enable, int point_1_x, int point_1_y, int point_2_x,
							  int point_2_y)
{
	// TODO
	return 0;
}

/**
 * @brief write radar bias speed
 *
 * @param dir direction of the speed, 0: stationary, 1: moving forward, 2: moving backward
 * @param speed speed, in 0.02 m/s, should always be positive, from 0 to 163.8 m/s
 * @return int
 */
int controller::write_speed(int dir, int speed)
{
	uint64_t reg = 0;
	if (speed < 0 || dir > 2 || dir < 0) return -2;
	write_bits(reg, 48, 13, speed);
	write_bits(reg, 62, 2, dir);
	return this->write_frame(can_id::SpeedInformation, 2, reg);
}
/**
 * @brief write radar bias yaw rate, its like shaking your head, assume the rotate centre to
 * be 1.95 m behind the radar
 *
 * @param yawrate in 0.01 deg/s, from -327.68 to 327.68 deg/s
 * @return int
 */
int controller::write_yawrate(int yawrate)
{
	uint64_t reg = 0;
	if (yawrate < -32768 || yawrate > 32768) return -2;
	write_bits(reg, 48, 16, yawrate);
	return this->write_frame(can_id::YawRateInformation, 2, reg);
}
/**
 * @brief host endian 8 bytes to network endian 64 bit
 *
 * @param data byte pointer
 * @return uint64_t result
 */
uint64_t hton64(uint8_t data[8])
{
	if (htonl(0x01) == 0x01)
	{
		return *(uint64_t *)data;
	}
	else
	{
		return swap_endian(data);
	}
}

/**
 * @brief network endian 8 bytes to host endian 64 bit
 *
 * @param data byte pointer
 * @return uint64_t result
 */
uint64_t ntoh64(uint8_t data[8])
{
	return hton64(data);
}
