#include "ars_40x_msgs/msg/cluster_general.hpp"
#include "ars_40x_msgs/msg/cluster_general_array.hpp"
#include "ars_40x_msgs/msg/object_general.hpp"
#include "ars_40x_msgs/msg/object_general_array.hpp"
#include "ars_40x_msgs/msg/radar_state.hpp"
#include "ars_40x_msgs/msg/speed_information.hpp"
#include "ars_40x_msgs/msg/yaw_rate_information.hpp"
#include "ars_40x_msgs/msg/object_extended_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace ars_40x_msgs::msg;
using namespace visualization_msgs::msg;
using namespace geometry_msgs::msg;

class rviz_node : public rclcpp::Node
{
  private:
	rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub;
	rclcpp::Subscription<ClusterGeneralArray>::SharedPtr clusters_sub;
	rclcpp::Subscription<ObjectExtendedArray>::SharedPtr objects_sub;
	rclcpp::TimerBase::SharedPtr timer;

	void clusters_callback(ClusterGeneralArray array)
	{
		RCLCPP_INFO(this->get_logger(), "  << got %ld clusters", array.clusters.size());
		MarkerArray markers;
		int cnt = 0;
		static auto start = std::chrono::steady_clock::now();
		Marker m;
		Point p;
		builtin_interfaces::msg::Duration life;

		for (auto i : array.clusters)
		{
			m.points.clear();
			if (i.rcs > -50)
			{
				m.id = i.id;
				m.type = Marker::POINTS;
				m.header.frame_id = "map";
				m.action = Marker::ADD;
				m.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count();
				m.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
				m.ns =  std::to_string(i.rcs);
				p.x = i.dist_long;
				p.y = i.dist_lat;
				p.z = 0;
				m.points.push_back(p);
				m.scale.x = (i.rcs + 50) / 100 * 0.5;
				m.scale.y = m.scale.x;
				m.color.r = 1;
				m.color.g = 0;
				m.color.b = 0;
				m.color.a = 1;
				life.sec = 0;
				life.nanosec = 100000000;
				m.lifetime = life;
				markers.markers.push_back(m);
				cnt ++;
			}	
		}

		this->markers_pub->publish(markers);
	}

	void objects_callback(ObjectExtendedArray array)
	{
		RCLCPP_INFO(this->get_logger(), "     << got %ld objects", array.objects.size());
				MarkerArray markers;
		int cnt = 0;
		static auto start = std::chrono::steady_clock::now();
		Marker m;
		Point p;
		builtin_interfaces::msg::Duration life;

		for (auto i : array.objects)
		{
			m.points.clear();
			if (i.general.rcs > -50)
			{
				m.id = i.general.id;
				m.type = Marker::POINTS;
				m.header.frame_id = "map";
				m.action = Marker::ADD;
				m.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count();
				m.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
				m.ns =  i.tag;
				p.x = i.general.dist_long;
				p.y = i.general.dist_lat;
				p.z = 0;
				m.points.push_back(p);
				m.scale.x = i.length;
				m.scale.y = i.width;
				if (i.tag == "point")
				{
					m.color.r = 0;
					m.color.g = 1;
					m.color.b = 0;
				}
				else if (i.tag == "wide")
				{
					m.color.r = 1;
					m.color.g = 1;
					m.color.b = 1;
				}
				else if (i.tag == "unknown")
				{
					m.color.r = 0;
					m.color.g = 0;
					m.color.b = 1;
				}
				else
				{
					m.color.r = 1;
					m.color.g = 0;
					m.color.b = 0;
				}
				m.color.a = 1;
				life.sec = 0;
				life.nanosec = 100000000;
				m.lifetime = life;
				markers.markers.push_back(m);
				cnt ++;
			}	
		}
		this->markers_pub->publish(markers);
	}

  public:
	rviz_node(std::string name) : Node(name)
	{
		this->markers_pub = this->create_publisher<MarkerArray>("radar_cluster_cloud", 5);

		this->clusters_sub = this->create_subscription<ClusterGeneralArray>(
			"radar_clusters", 1,
			std::bind(&rviz_node::clusters_callback, this, std::placeholders::_1));
		this->objects_sub = this->create_subscription<ObjectExtendedArray>(
			"radar_objects", 1,
			std::bind(&rviz_node::objects_callback, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(), "client initialized");
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rviz_node>("rviz_node");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
