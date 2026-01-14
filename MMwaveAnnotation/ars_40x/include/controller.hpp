#include "iolayer.hpp"

typedef enum
{
	RadarCfg = 0x200,
	RadarState = 0x201,
	FilterCfg = 0x202,
	FilterState_Header = 0x203,
	FilterState_Cfg = 0x204,
	PolygonFilter_Cfg = 0x205,
	PolygonFilter_State = 0x206,
	CollDetCfg = 0x400,
	CollDetRegionCfg = 0x401,
	CollDetState = 0x408,
	CollDetRegionState = 0x402,
	SpeedInformation = 0x300,
	YawRateInformation = 0x301,
	Cluster_0_Status = 0x600,
	Cluster_1_General = 0x701,
	Cluster_2_Quality = 0x702,
	Object_0_Status = 0x60A,
	Object_1_General = 0x60B,
	Object_2_Quality = 0x60C,
	Object_3_Extended = 0x60D,
	Object_4_Warning = 0x60E,
	VersionID = 0x700,
	CollDetRelayCtrl = 0x8,
} can_id;

typedef enum
{
	SensorId = 57,		 // 0 ~ 7
	MaxDistance = 56,	 // 2m, 0 ~ 1023
	RadarPower = 58,	 // 0: standard, 1: -3dB, 2: -6dB, 3: -9dB
	OutputType = 59,	 // 0: do not output, 1: objects, 2: clusters
	SendQuality = 60,	 // 0: inactive, 1: active
	SendExInfo = 61,	 // 0: inactive, 1: active
	SortIndex = 62,		 // 0: no sort, 1: sort by distance, 2: sort by rcs
	CtrlRelay = 16,		 // 0: inactive, 1: active
	RCSThreshold = 8,	 // 0: standard, 1: high sensitivity
	StoreInNVM = 63,	 // 0: do not store, 1: store configurations in rom
	InvalidClusters = 12 // not supported
} radar_conf;

typedef enum
{
	NofObj = 0x00,
	Distance = 0x01,
	Azimuth = 0x02,
	VrelOncome = 0x03,
	VrelDepart = 0x04,
	RCS = 0x05,
	Lifetime = 0x06,
	Size = 0x07,
	ProbExists = 0x08,
	Y = 0x09,
	X = 0x0A,
	VYRightLeft = 0x0B,
	VXOncome = 0x0C,
	VYLeftRight = 0xD,
	VXDepart = 0x0E
} radar_filter;

// set given bit to 1
#define set_bit(reg, bit) ((reg) |= ((uint64_t)0x01 << (bit)))
// set given bit to 0
#define reset_bit(reg, bit) ((reg) &= ~((uint64_t)0x01 << (bit)))
// write given bit fields
#define write_bits(reg, start, size, val)                                                      \
	do                                                                                         \
	{                                                                                          \
		(reg) &= ~((0xFFFFFFFFFFFFFFFF >> (64 - (size))) << (start));                          \
		(reg) |= (((val) & (0xFFFFFFFFFFFFFFFF >> (64 - (size)))) << (start));                 \
	} while (0)
// get given bit fields
#define get_bits(reg, start, size) (((reg) >> (start)) & (0xFFFFFFFFFFFFFFFF >> (64 - (size))))

/**
 * @brief swap byte endian, data should be a uint8_t* pointing at the start address of a
 * uint64_t this conversion wont modify the data
 *
 */
#define swap_endian(data)                                                                      \
	((uint64_t)(data)[0] << 56 | (uint64_t)(data)[1] << 48 | (uint64_t)(data)[2] << 40 |       \
	 (uint64_t)(data)[3] << 32 | (uint64_t)(data)[4] << 24 | (uint64_t)(data)[5] << 16 |       \
	 (uint64_t)(data)[6] << 8 | (uint64_t)(data)[7])

class controller : public iolayer::layer_socket
{
  private:
  public:
	int radar_id;
	controller(std::string group_ip, int port, std::string remote_ip, std::string local_ip,
			   int id = 0);
	~controller();
	int read_data();
	int read_frame(uint32_t *id, int *dlc, uint8_t data[8]);
	int read_frame(uint32_t *id, int *dlc, uint64_t *reg);
	int wait_frame(can_id wait_id, uint8_t data[8], int retry = 10);
	int wait_frame(can_id wait_id, uint64_t *reg, int retry = 10);
	int write_frame(can_id id, int dlc, uint8_t data[8]);
	int write_frame(can_id id, int dlc, uint64_t reg);
	int write_config(radar_conf conf, int val);
	int write_config_reg(uint64_t reg);
	int write_filter(int type, radar_filter index, int active, int min, int max);
	int write_polygon(int enable, int point_1_x, int point_1_y, int point_2_x, int point_2_y);
	int write_speed(int dir, int speed);
	int write_yawrate(int yawrate);
};

uint64_t hton64(uint8_t data[8]);

uint64_t ntoh64(uint8_t data[8]);
