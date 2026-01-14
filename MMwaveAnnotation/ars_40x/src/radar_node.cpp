#include "ars_40x_msgs/msg/cluster_general_array.hpp"
#include "ars_40x_msgs/msg/object_extended_array.hpp"
#include "ars_40x_msgs/msg/object_general_array.hpp"
#include "ars_40x_msgs/msg/radar_state.hpp"
#include "ars_40x_msgs/msg/speed_information.hpp"
#include "ars_40x_msgs/msg/yaw_rate_information.hpp"
#include "ars_40x_msgs/srv/radar_config.hpp"
#include "ars_40x_msgs/srv/radar_filter.hpp"
#include "controller.hpp"
#include "hb_msgs/msg/heart_beat.hpp"
#include "rclcpp/rclcpp.hpp"

class radar_node : public rclcpp::Node
{
  private:
	// radar communication controller
	std::shared_ptr<controller> radar;

	// network parameters
	std::string group_ip;
	int port;
	std::string remote_ip;
	std::string local_ip;
	std::string output_type;
	int radar_id;

	// mutex to avoid communicaion interruption
	std::timed_mutex mutex;

	// message handlers
	rclcpp::Publisher<ars_40x_msgs::msg::ClusterGeneralArray>::SharedPtr cluster_pub;
	rclcpp::Publisher<ars_40x_msgs::msg::ObjectExtendedArray>::SharedPtr object_pub;
	rclcpp::Publisher<ars_40x_msgs::msg::RadarState>::SharedPtr state_pub;
	rclcpp::Subscription<ars_40x_msgs::msg::SpeedInformation>::SharedPtr speed_sub;
	rclcpp::Subscription<ars_40x_msgs::msg::YawRateInformation>::SharedPtr yawrate_sub;
	rclcpp::Service<ars_40x_msgs::srv::RadarConfig>::SharedPtr conf_srv;
	rclcpp::Service<ars_40x_msgs::srv::RadarFilter>::SharedPtr filt_srv;

	std::thread recvthread;

	rclcpp::TimerBase::SharedPtr hb_timer;
	rclcpp::Publisher<hb_msgs::msg::HeartBeat>::SharedPtr hb_pub;
	int object_array_cnt = 0;
	int cluster_array_cnt = 0;
	int object_drop = 0;
	int cluster_drop = 0;

	bool bypassed = false;

  public:
	int enable_bypass(bool enable)
	{
		this->bypassed = enable;
		return 0;
	}

	int radar_init()
	{
		uint64_t reg;
		auto rate = rclcpp::Rate(std::chrono::milliseconds(1000));
		if (this->bypassed)
		{
			RCLCPP_INFO(this->get_logger(), "host bypassed, skip init");
			return 0;
		}

		if (this->output_type == "object")
		{

			// output type
			RCLCPP_INFO(this->get_logger(), "config output type: %d",
						this->radar->write_config(radar_conf::OutputType, 1));
			rate.sleep();
			RCLCPP_INFO(this->get_logger(), "config extended info: %d",
						this->radar->write_config(radar_conf::SendExInfo, 1));
			rate.sleep();
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "config output type: %d",
						this->radar->write_config(radar_conf::OutputType, 2));
			rate.sleep();
		}
		return 0;
	}

	void hb_callback(void)
	{
		hb_msgs::msg::HeartBeat hb;
		hb.name = this->get_fully_qualified_name();
		hb.stamp = rclcpp::Clock().now();
		if (this->output_type == "object")
		{
			if (this->object_array_cnt == 0)
			{
				hb.code = -1;
				hb.reason = "no response from radar";
			}
			else if (this->object_drop > 0)
			{
				hb.code = -2;
				hb.reason = "frame drop";
				this->object_drop = 0;
			}
			else
			{
				this->object_array_cnt = 0;
				hb.code = 0;
				hb.reason = "ok";
			}
		}
		else
		{
			if (this->cluster_array_cnt == 0)
			{
				hb.code = -1;
				hb.reason = "no response from radar";
			}
			else if (this->cluster_drop > 0)
			{
				hb.code = -2;
				hb.reason = "frame drop";
				this->cluster_drop = 0;
			}
			else
			{
				this->cluster_array_cnt = 0;
				hb.code = 0;
				hb.reason = "ok";
			}
		}
		this->hb_pub->publish(hb);
	}

	radar_node(std::string name) : Node(name)
	{
		// parameters
		this->declare_parameter<std::string>("group_ip", "239.255.0.6");
		this->declare_parameter<int>("port", 6780);
		this->declare_parameter<std::string>("remote_ip", "10.0.1.6");
		this->declare_parameter<std::string>("local_ip", "10.0.1.20");
		this->declare_parameter<std::string>("output_type", "object");
		this->declare_parameter<int>("radar_id", 0);
		this->declare_parameter<bool>("bypassed", false);
		this->get_parameter("group_ip", this->group_ip);
		this->get_parameter("port", this->port);
		this->get_parameter("remote_ip", this->remote_ip);
		this->get_parameter("local_ip", this->local_ip);
		this->get_parameter("output_type", this->output_type);
		this->get_parameter("radar_id", this->radar_id);
		this->get_parameter("bypassed", this->bypassed);
		RCLCPP_INFO(this->get_logger(),
					"in multicast group %s, port %d, remote host %s, local host %s, output "
					"type: %s, radar id: %d",
					this->group_ip.c_str(), this->port, this->remote_ip.c_str(),
					this->local_ip.c_str(), this->output_type.c_str(), this->radar_id);
		if (this->bypassed)
		{
			RCLCPP_INFO(this->get_logger(), "NOTE: this host is bypassed");
		}
		// create radar controller
		this->radar = std::make_shared<controller>(this->group_ip, this->port, this->remote_ip,
												   this->local_ip, this->radar_id);
		RCLCPP_INFO(this->get_logger(), "controller initialized");
		// initial config
		this->radar_init();

		// create publishers
		this->cluster_pub =
			this->create_publisher<ars_40x_msgs::msg::ClusterGeneralArray>("radar_clusters", 1);
		this->object_pub =
			this->create_publisher<ars_40x_msgs::msg::ObjectExtendedArray>("radar_objects", 1);
		this->state_pub =
			this->create_publisher<ars_40x_msgs::msg::RadarState>("radar_state", 1);
		// subscribe
		this->speed_sub = this->create_subscription<ars_40x_msgs::msg::SpeedInformation>(
			"radar_speed", 1,
			std::bind(&radar_node::speed_callback, this, std::placeholders::_1));
		this->yawrate_sub = this->create_subscription<ars_40x_msgs::msg::YawRateInformation>(
			"radar_yawrate", 1,
			std::bind(&radar_node::yawrate_callback, this, std::placeholders::_1));
		// config service
		this->conf_srv = this->create_service<ars_40x_msgs::srv::RadarConfig>(
			"radar_config", std::bind(&radar_node::config_callback, this, std::placeholders::_1,
									  std::placeholders::_2));
		this->filt_srv = this->create_service<ars_40x_msgs::srv::RadarFilter>(
			"radar_filter", std::bind(&radar_node::filter_callback, this, std::placeholders::_1,
									  std::placeholders::_2));
		this->hb_pub = this->create_publisher<hb_msgs::msg::HeartBeat>("/heartbeat", 1);
		this->hb_timer = this->create_wall_timer(std::chrono::milliseconds(200),
												 std::bind(&radar_node::hb_callback, this));

		RCLCPP_INFO(this->get_logger(), "initialize done");
	}

	void radar_state_to_msg(uint64_t reg, ars_40x_msgs::msg::RadarState &msg)
	{
		msg.rcs_threshold = get_bits(reg, 2, 3);
		msg.invalid_clusters = get_bits(reg, 8, 8);
		msg.ctrl_relay = get_bits(reg, 17, 1);
		msg.output_type = get_bits(reg, 18, 2);
		msg.send_quality = get_bits(reg, 20, 1);
		msg.send_ex_info = get_bits(reg, 21, 1);
		msg.motion_rx_state = get_bits(reg, 22, 2);
		msg.sensor_id = get_bits(reg, 24, 3);
		msg.sort_index = get_bits(reg, 28, 3);
		msg.radar_power = get_bits(reg, 31, 3);
		msg.voltage_error = get_bits(reg, 41, 1);
		msg.temporary_error = get_bits(reg, 42, 1);
		msg.temperature_error = get_bits(reg, 43, 1);
		msg.interference = get_bits(reg, 44, 1);
		msg.persistent_error = get_bits(reg, 45, 1);
		msg.max_distance = get_bits(reg, 46, 10) / 2.0;
		msg.nvm_read_status = get_bits(reg, 62, 1);
		msg.nvm_write_status = get_bits(reg, 63, 1);
	}

	void filter_callback(const ars_40x_msgs::srv::RadarFilter_Request::SharedPtr req,
						 ars_40x_msgs::srv::RadarFilter_Response::SharedPtr res)
	{
		int min, max;
		float percents[] = {-1, 12.5, 37.5, 82.5, 94.5, 99.45, 99.95, 101};
		if (this->bypassed)
		{
			RCLCPP_INFO(this->get_logger(), "host bypassed, ignore filter config");
			res->retval = -1;
			res->reason = "cannot config filter when host is bypassed";
			return;
		}

		switch (req->index)
		{
			case 0: // NofObj
				min = req->min;
				max = req->max;
				if (min < 0) min = 0;
				if (max > 4095) max = 4095;
				break;
			case 1: // Distance
			case 6: // Lifetime
				min = req->min / 0.1;
				max = req->max / 0.1;
				if (min < 0) min = 0;
				if (max > 4095) max = 4095;
				break;
			case 2: // Azimuth
			case 7: // Size
				min = (req->min + 50) / 0.025;
				max = (req->max + 50) / 0.025;
				if (min < 0) min = 0;
				if (max > 4095) max = 4095;
				break;
			case 3:	 // VrelOncome
			case 4:	 // VrelDepart
			case 11: // VYRightLeft
			case 12: // VXOncome
			case 13: // VYLeftRight
			case 14: // VXDepart
				min = req->min / 0.0315;
				max = req->max / 0.0315;
				if (min < 0) min = 0;
				if (max > 4095) max = 4095;
				break;
			case 5: // RCS
				min = (req->min + 50) / 0.025;
				max = (req->max + 50) / 0.025;
				if (min < 0) min = 0;
				if (max > 4095) max = 4095;
				break;
			case 8: // ProbExists
				min = 0;
				max = 7;
				for (int i = 0; i < 7; i++)
				{
					if (req->min > percents[i] && req->min < percents[i + 1])
					{
						min = i;
					}
					if (req->max > percents[i] && req->max < percents[i + 1])
					{
						max = i;
					}
				}
				break;
			case 9: // Y
				min = (req->min + 409.5) / 0.2;
				max = (req->max + 409.5) / 0.2;
				if (min < 0) min = 0;
				if (max > 4095) max = 4095;
				break;
			case 10: // X
				min = (req->min + 500) / 0.2;
				max = (req->max + 500) / 0.2;
				if (min < 0) min = 0;
				if (max > 8191) max = 8191;
				break;
			default:
				res->retval = -1;
				res->reason = "unsupported filter index";
				break;
		}
		if (!this->mutex.try_lock_for(std::chrono::milliseconds(1000)))
		{
			res->retval = -1;
			res->reason = "failed to get lock";
			return;
		}
		res->retval = this->radar->write_filter(req->type, (radar_filter)req->index,
												req->active, min, max);
		if (res->retval > 0)
			res->reason = "successful";
		else
			res->reason = "failed to write filter config frame";
		this->mutex.unlock();
	}

	void config_callback(const ars_40x_msgs::srv::RadarConfig_Request::SharedPtr req,
						 ars_40x_msgs::srv::RadarConfig_Response::SharedPtr res)
	{
		auto s = req->option;
		int err = 0;
		uint64_t reg;
		// prepare config reg
		if (!this->mutex.try_lock_for(std::chrono::milliseconds(100)))
		{
			res->result = 0;
			res->explan = "failed to get mutex lock";
			return;
		}

		do
		{
			// check options
			if (this->bypassed)
			{
				if (s == "bypassed")
				{
					this->enable_bypass(req->value > 0 ? true : false);
					res->result = 20;
					res->explan = "successful";
				}
				else
				{
					res->result = 0;
					res->explan = "cannot config when host is bypassed";
				}
				break;
			}
			if (s == "raw")
			{
				// raw config
				err = this->radar->write_config_reg(req->value);
				if (err < 0)
				{
					res->result = 0;
					res->explan = "failed to write config frame";
					break;
				}
				err = this->radar->wait_frame(can_id::RadarState, &res->result, 100);
				if (err < 0)
				{
					res->result = 0;
					res->explan = "config written but no response";
					break;
				}
				res->explan = "successful";
			}
			else if (s == "output_type")
			{
				err = this->radar->write_config(radar_conf::OutputType, req->value);
				if (err < 0)
				{
					res->result = 0;
					res->explan = "failed to write config frame";
					break;
				}
				err = this->radar->wait_frame(can_id::RadarState, &res->result, 100);
				if (err < 0)
				{
					res->result = 0;
					res->explan = "config written but no response";
					break;
				}
				ars_40x_msgs::msg::RadarState state;
				this->radar_state_to_msg(res->result, state);
				if (state.output_type != req->value)
				{
					res->explan = "the radar refused to change";
					// the radar always refuse to change
					break;
				}
			}
			else
			{
				// unsupported for now
				res->result = 0;
				res->explan = "unsupported config option";
				break;
			}
			res->explan = "successful";
		} while (0);
		this->mutex.unlock();
	}

	void speed_callback(ars_40x_msgs::msg::SpeedInformation msg)
	{
		if (this->bypassed)
		{
			return;
		}

		if (this->mutex.try_lock_for(std::chrono::milliseconds(10)))
		{
			this->radar->write_speed(abs(msg.speed) < 0.001 ? 0 : (msg.speed > 0 ? 1 : 2),
									 msg.speed * 50);
			this->mutex.unlock();
		}
	}

	void yawrate_callback(ars_40x_msgs::msg::YawRateInformation msg)
	{
		if (this->bypassed)
		{
			return;
		}

		if (this->mutex.try_lock_for(std::chrono::milliseconds(10)))
		{
			this->radar->write_yawrate(msg.yawrate * 100);
			this->mutex.unlock();
		}
	}

	void start_recv()
	{
		this->recvthread = std::thread(std::bind(&radar_node::recv_loop, this));
		this->recvthread.detach();
	}

	/**
	 * @brief in this thread read data and publish
	 *
	 */
	void recv_loop()
	{
		uint32_t id;
		int dlc;
		uint8_t data[8];
		uint64_t reg;
		int incoming_clusters = 0;
		int incoming_objects = 0;
		ars_40x_msgs::msg::ClusterGeneralArray::SharedPtr clusters = nullptr;
		ars_40x_msgs::msg::ObjectExtendedArray::SharedPtr objects = nullptr;
		ars_40x_msgs::msg::ClusterGeneral cluster;
		ars_40x_msgs::msg::ObjectExtended object;
		ars_40x_msgs::msg::RadarState state;

		RCLCPP_INFO(this->get_logger(), "waiting for rclcpp to be ready");
		while (!rclcpp::ok())
			;
		RCLCPP_INFO(this->get_logger(), "start receiving data from radar");
		while (rclcpp::ok())
		{
			// wait for lock
			if (!this->mutex.try_lock_for(std::chrono::milliseconds(10)))
			{
				RCLCPP_DEBUG(this->get_logger(), "failed to get lock, pass this");
				continue;
			}
			if (this->radar->read_frame(&id, &dlc, data) <= 0)
			{
				this->mutex.unlock();
				// sleep so other threads could get the mutex
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				continue;
			}
			reg = ntoh64(data);
			switch (id)
			{
				// radar state, got this every second
				case can_id::RadarState:
					this->radar_state_to_msg(reg, state);
					this->state_pub->publish(state);
					RCLCPP_DEBUG(this->get_logger(), "radar state: %016lx", reg);
					break;

				case can_id::Cluster_0_Status:
					// if there are remaining clusters, publish them
					if (clusters != nullptr && clusters->clusters.size() > 0)
					{
						this->cluster_pub->publish(*clusters);
						this->cluster_drop += 1;
					}
					// prepare a new cluster array
					incoming_clusters = data[0] + data[1];
					clusters = std::make_shared<ars_40x_msgs::msg::ClusterGeneralArray>();
					clusters->header.frame_id =
						rclcpp::extend_name_with_sub_namespace("radar", this->get_namespace());
					clusters->header.stamp = rclcpp::Clock().now();
					this->cluster_array_cnt += 1;
					RCLCPP_DEBUG(this->get_logger(), "incoming clusters: %d",
								 incoming_clusters);
					break;

				case can_id::Cluster_1_General:
					// if not expected, ignore
					if (incoming_clusters <= 0 || clusters == nullptr) break;
					// we convert raw big endian bytes to host endian 64 bit, then shift bits to
					// get all data parts the manual is misleading and sucks
					cluster.id = get_bits(reg, 56, 8);
					cluster.dist_long = get_bits(reg, 43, 13) * 0.2 - 500;
					cluster.dist_lat = get_bits(reg, 32, 10) * 0.2 - 102.3;
					cluster.vrel_long = (int)get_bits(reg, 22, 10) * 0.25 - 128;
					cluster.vrel_lat = (int)get_bits(reg, 13, 9) * 0.25 - 64;
					cluster.rcs = (int)get_bits(reg, 0, 8) * 0.5 - 64;
					cluster.dyn_prop = get_bits(reg, 48, 3);
					RCLCPP_DEBUG(this->get_logger(),
								 "got cluster %d: distlong: %f, dist_lat: %f, vrellong: %f, "
								 "vrellat: %f, rcs: %f",
								 cluster.id, cluster.dist_long, cluster.dist_lat,
								 cluster.vrel_long, cluster.vrel_lat, cluster.rcs);
					// put one cluster
					clusters->clusters.push_back(cluster);
					incoming_clusters--;

					// the array will be published once all clusters were received from radar
					if (incoming_clusters == 0 && clusters->clusters.size() > 0)
					{
						// object will be destroied once its published
						this->cluster_pub->publish(*clusters);
						clusters = nullptr;
						RCLCPP_DEBUG(this->get_logger(), "published cluster array");
					}
					break;

				case can_id::Object_0_Status:
					if (objects != nullptr && objects->objects.size() > 0)
					{
						this->object_pub->publish(*objects);
						this->object_drop += 1;
					}
					incoming_objects = data[0];
					objects = std::make_shared<ars_40x_msgs::msg::ObjectExtendedArray>();
					objects->header.frame_id =
						rclcpp::extend_name_with_sub_namespace("radar", this->get_namespace());
					objects->header.stamp = rclcpp::Clock().now();
					this->object_array_cnt += 1;
					break;

				case can_id::Object_1_General:
					if (incoming_objects <= 0 || objects == nullptr) break;
					object.general.id = get_bits(reg, 56, 8);
					object.general.dist_long = (int)get_bits(reg, 43, 13) * 0.2 - 500;
					object.general.dist_lat = (int)get_bits(reg, 32, 11) * 0.2 - 204.6;
					object.general.vrel_long = (int)get_bits(reg, 22, 10) * 0.25 - 128;
					object.general.vrel_lat = (int)get_bits(reg, 13, 9) * 0.25 - 64;
					object.general.dyn_prop = get_bits(reg, 8, 3);
					object.general.rcs = (int)get_bits(reg, 0, 8) * 0.5 - 64;
					objects->objects.push_back(object);
					// save address so we can add extended information later

					break;
				case can_id::Object_3_Extended:
					if (incoming_objects <= 0 || objects == nullptr) break;
					for (auto &p : objects->objects)
					{
						if (p.general.id != data[0]) continue;
						p.arel_long = get_bits(reg, 45, 11) * 0.01 - 10;
						p.arel_lat = get_bits(reg, 36, 9) * 0.01 - 2.5;
						switch (get_bits(reg, 32, 3))
						{
							case 0:
								p.tag = "point";
								break;
							case 1:
								p.tag = "car";
								break;
							case 2:
								p.tag = "truck";
								break;
							case 3:
								p.tag = "pedestrian";
								break;
							case 4:
								p.tag = "motorcycle";
								break;
							case 5:
								p.tag = "bicycle";
								break;
							case 6:
								p.tag = "wide";
								break;
							case 7:
								p.tag = "unknown";
								break;
							default:
								p.tag = "error";
								break;
						}
						p.orientation_angle = get_bits(reg, 22, 10) * 0.4 - 180;
						p.length = get_bits(reg, 8, 8) * 0.2;
						p.width = get_bits(reg, 0, 8) * 0.2;
						incoming_objects--;
						break;
					}

					if (incoming_objects == 0 && objects->objects.size() > 0)
					{
						this->object_pub->publish(*objects);
						objects = nullptr;
						RCLCPP_DEBUG(this->get_logger(), "published object array");
					}
					break;

				// other frames
				default:
					break;
			}
			// release lock after read
			this->mutex.unlock();
			// sleep?
			// TODO
			std::this_thread::yield();
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<radar_node>("radar_node");
	node->start_recv();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
