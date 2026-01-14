#include <iostream>
#include <deque>
#include <iostream>
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <boost/circular_buffer.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "ars_40x_msgs/msg/cluster_general.hpp"
#include "ars_40x_msgs/msg/cluster_general_array.hpp"
#include "ars_40x_msgs/msg/object_extended.hpp"
#include "ars_40x_msgs/msg/object_extended_array.hpp"
#include "ars_40x_msgs/msg/object_general.hpp"
#include "ars_40x_msgs/msg/object_general_array.hpp"
#include "ars_40x_msgs/msg/radar_state.hpp"
#include "ars_40x_msgs/msg/speed_information.hpp"
#include "ars_40x_msgs/msg/yaw_rate_information.hpp"

#include "SensorDataWriter.hpp"

using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;

namespace fs = std::filesystem;

struct DataObject {
    int frame;
    int id;
    float dist_long;
    float dist_lat;
    float vrel_long;
    float vrel_lat;
    float rcs;
    int dyn_prop;
    float arel_long;
    std::string tag;
    float arel_lat;
    float orientation_angle;
    float length;
    float width;
    int class_type;
};

class DataPreprocessing {
    public:
    DataPreprocessing(const fs::path rosbag_dir, const fs::path output_dir){

        SensorDataWriter data_wtriter(0);

        // topics_of_interest_ = {"/front/radar_objects","/iv_points_1","/hkcam1/image"};  // // PanZhiHua
        // topics_of_interest_ = {"/back/radar_objects","/iv_points_2","/hkcam2/image"};  // PanZhiHua
        topics_of_interest_ = {"/front/radar_objects","/ls128_left/lslidar_point_cloud","/image/oak_front"};  // LvShun

        // Storage Options
        storage_options_.uri = rosbag_dir;
        storage_options_.storage_id = "sqlite3";
        // Converter Options
        converter_options_.input_serialization_format = "cdr"; 
        converter_options_.output_serialization_format = "cdr";
        std::cout << "serialization_format:" << rmw_get_serialization_format() << std::endl;
        // Open Bag and Store Metadata
        rosbag2_cpp::readers::SequentialReader reader;
        
        try {
            reader.open(storage_options_, converter_options_);
        } catch (std::exception e) {
            exit(1);
        }
        std::vector<rosbag2_storage::TopicMetadata> topic_types = reader.get_all_topics_and_types();
        bag_data_ = reader.get_metadata();
        for (std::vector<rosbag2_storage::TopicMetadata>::iterator itr  = topic_types.begin(); itr != topic_types.end(); itr++) {
            topic_to_type_.insert({itr->name, itr->type});
        }
        
        rosbag2_storage::StorageFilter storage_filter{};
        storage_filter.topics = std::vector<std::string> {topics_of_interest_};
        reader.set_filter(storage_filter);

        sensor_msgs::msg::PointCloud2 cur_lidar,last_lidar;
        sensor_msgs::msg::Image cur_image, last_image;

        int frame_num = 0;
        while(reader.has_next()){
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
            if (msg->topic_name == topics_of_interest_[0]){
                if(lidar_buf.empty() || image_buf.empty()) continue;
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                std::cout << "Processing:" << msg->topic_name << std::endl;
                rclcpp::Serialization<ars_40x_msgs::msg::ObjectExtendedArray> serialization_;
                auto ros_msg_ = std::make_shared<ars_40x_msgs::msg::ObjectExtendedArray>();
                serialization_.deserialize_message(&serialized_msg, ros_msg_.get());
                std::stringstream time_ss;
                time_ss << ros_msg_->header.stamp.sec << "." << ros_msg_->header.stamp.nanosec;
                double radar_time = stod(time_ss.str());
                std::vector<DataObject> data;
                for(auto item : ros_msg_->objects){
                    DataObject point;
                    point.dist_lat = (float)item.general.dist_lat;
                    point.dist_long = (float)item.general.dist_long;
                    point.arel_lat = item.arel_lat;
                    point.arel_long = item.arel_long;
                    point.class_type = 0;
                    point.dyn_prop = (int8_t) item.general.dyn_prop;
                    point.frame = frame_num;
                    point.id = (int16_t) item.general.id;
                    point.length = item.length;
                    point.orientation_angle = item.orientation_angle;
                    point.rcs = item.general.rcs;
                    point.tag = item.tag;
                    point.width = item.width;
                    point.vrel_lat = item.general.vrel_lat;
                    point.vrel_long = item.general.vrel_long;
                    data.push_back(point);
                }
                writeCSV(frame_num,output_dir,".csv",data);
                frame_num++;
                if(!lidar_buf.empty()){
                    cur_lidar = lidar_buf.front();
                    std::stringstream ss;
                    ss << cur_lidar.header.stamp.sec << "." << cur_lidar.header.stamp.nanosec;
                    double lidar_time = stod(ss.str());
                    // if lidar frame earlier than radar frame, then pop out
                    if(lidar_time < radar_time){
                        last_lidar = cur_lidar;
                        lidar_buf.pop_front();
                    }else{
                        std::stringstream ss;
                        ss << last_lidar.header.stamp.sec << "." << last_lidar.header.stamp.nanosec;
                        double last_lidar_time = stod(ss.str());
                        // if last lidar frame closer to radar frame than current lidar frame, then use last lidar frame
                        if(fabs(radar_time-last_lidar_time) <= fabs(radar_time-lidar_time)){
                            // save last lidar frame 
                        }else{
                            // save current lidar frame
                            last_lidar = cur_lidar;
                        }
                        lidar_buf.pop_front();
                    }
                    pcl::PointCloud<pcl::PointXYZ> cloud;
                    pcl::fromROSMsg(last_lidar, cloud);
                    std::stringstream pathss;
                    pathss << output_dir.c_str() << to_string(frame_num) << ".pcd";
                    std::cout << pathss.str() << std::endl;
                    pcl::io::savePCDFileASCII(pathss.str(), cloud);
                }
                if(!image_buf.empty()){
                    cur_image = image_buf.front();
                    std::stringstream ss;
                    ss << cur_image.header.stamp.sec << "." << cur_image.header.stamp.nanosec;
                    double image_time = stod(ss.str());
                    // if lidar frame earlier than radar frame, then pop out
                    if(image_time < radar_time){
                        last_image = cur_image;
                        image_buf.pop_front();
                    }else{
                        std::stringstream ss;
                        ss << last_image.header.stamp.sec << "." << last_image.header.stamp.nanosec;
                        double last_image_time = stod(ss.str());
                        // if last image frame closer to radar frame than current lidar frame, then use last image frame
                        if(fabs(radar_time-last_image_time) <= fabs(radar_time-image_time)){
                            // save last image frame 
                        }else{
                            // save current image frame
                            last_image = cur_image;
                        }
                        image_buf.pop_front();
                    }
                    cv_bridge::CvImagePtr in_image_ptr;
                    try {
                        in_image_ptr = cv_bridge::toCvCopy(last_image, sensor_msgs::image_encodings::BGR8);
                    } catch (cv_bridge::Exception & e) {
                        std::cout << "cv_bridge exception: " << e.what() << std::endl;
                        return;
                    }
                    std::stringstream pathss;
                    pathss << output_dir.c_str() << to_string(frame_num) << ".jpg";
                    cv::imwrite(pathss.str(), in_image_ptr->image);
                }
            }else if (msg->topic_name == topics_of_interest_[1]){
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                std::cout << "Processing:" << msg->topic_name << std::endl;
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
                auto ros_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
                serialization_.deserialize_message(&serialized_msg, ros_msg_.get());
                lidar_buf.push_back(*ros_msg_);
            }else if (msg->topic_name == topics_of_interest_[2]){
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                std::cout << "Processing:" << msg->topic_name << std::endl;
                rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
                auto ros_msg_ = std::make_shared<sensor_msgs::msg::Image>();
                serialization_.deserialize_message(&serialized_msg, ros_msg_.get());
                image_buf.push_back(*ros_msg_);
            }
        }
        data_wtriter.close();
        reader.close();
    }

    std::string generate_filename(int frame_number, string file_name, string file_format) {
        std::ostringstream filename;
        // filename << file_name << std::setfill('0') << std::setw(5) << frame_number << file_format;
        filename << file_name << frame_number << file_format;
        return filename.str();
    }

    void writeCSV(int frame_number, string radar_file_name, string radar_file_format_csv, std::vector<DataObject> &data) {

        std::lock_guard<std::mutex> guard(write_mutex_);

        std::string directory_radar = radar_file_name;
        // 检查目录是否存在，如果不存在则创建目录
        if(!fs::exists(directory_radar)) {
            fs::create_directory(directory_radar);
        }

        std::string radar_filename_csv = generate_filename(frame_number, radar_file_name, radar_file_format_csv);
        std::ofstream file_csv(radar_filename_csv);
        if (!file_csv.is_open()) {
            std::cerr << "Error opening file_csv: " << radar_filename_csv << std::endl;
            return;
        }

        file_csv << "frame id dist_long dist_lat vrel_long vrel_lat rcs dyn_prop arel_long tag arel_lat orientation_angle length width class" << std::endl;
      
        // 逐行写入每个对象的属性
        for (const auto &obj : data) {
            file_csv << obj.frame << " " << obj.id << " " << obj.dist_long << " " << obj.dist_lat << " "
                     << obj.vrel_long << " " << obj.vrel_lat << " " << obj.rcs << " "
                     << obj.dyn_prop << " " << obj.arel_long << " " << obj.tag << " "
                     << obj.arel_lat << " " << obj.orientation_angle << " " << obj.length << " "
                     << obj.width << " " << obj.class_type << std::endl;
        }
        file_csv.close();
        std::cout << "Frame data saved successfully. ---" << radar_filename_csv << std::endl;
    }

    private:
    deque<sensor_msgs::msg::PointCloud2> lidar_buf;
    deque<sensor_msgs::msg::Image> image_buf;
    ars_40x_msgs::msg::ObjectExtendedArray radar_array;
    sensor_msgs::msg::PointCloud2 lidar_ros_msg_;
    sensor_msgs::msg::Image camera_ros_msg_;
    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;
    rosbag2_storage::BagMetadata bag_data_;
    std::map<std::string, std::string> topic_to_type_;
    std::vector<std::string> topics_of_interest_;
    std::mutex write_mutex_;
};

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
        return 1;
    }
    DataPreprocessing *dd_ = new DataPreprocessing(argv[1],argv[2]);

    return 0;
}
