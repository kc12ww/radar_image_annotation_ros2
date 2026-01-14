#ifndef CLOUD_ANNOTATION_RADAR_TRAN_LIDAR_HPP
#define CLOUD_ANNOTATION_RADAR_TRAN_LIDAR_HPP

#include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <array>
#include <string>
#include <fstream>

#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <mutex>

// #include <visualization_msgs/msg/marker_array.hpp>


using namespace std;
using std::placeholders::_1;

#define MAX_RADAR_TIME_GAP 15 * 1e6
typedef pcl::PointXYZI PointType;

class RadarLidarCalib {
public:
    RadarLidarCalib(){
        radar_points = std::make_shared<pcl::PointCloud<PointType>>();
        loadExtInt();
    }
    void loadExtInt();
    pcl::PointXYZ radar2lidar(const double dist_long, const double dist_lat);
    pcl::PointXYZ lidar2radar(const double x, const double y);
private:
    vector<double> extRotV;
    vector<double> extTransV;

    Eigen::Matrix3d extRot;
    Eigen::Vector3d extTrans;
    Eigen::Matrix4d extRT;

    pcl::PointCloud<PointType>::Ptr radar_points;
};

pcl::PointXYZ RadarLidarCalib::radar2lidar(const double dist_long, const double dist_lat) {

    pcl::PointXYZ point_tmp;

    if (std::abs(dist_long) > 1e-6 || std::abs(dist_lat) > 1e-6){
       
        point_tmp.x = dist_long;
        point_tmp.y = dist_lat;
        // if(point_tmp.y > 0) point_tmp.z = 0.0;
        point_tmp.z = 0.0;

    }

    Eigen::MatrixXd X(4, 1);
    Eigen::MatrixXd Y(4, 1);

    X(0) = point_tmp.x;
    X(1) = point_tmp.y;
    X(2) = point_tmp.z;
    X(3) = 1.0;

    Y = extRT * X;

    point_tmp.x = Y(0);
    point_tmp.y = Y(1);
    point_tmp.z = Y(2);

    return point_tmp;
}

pcl::PointXYZ RadarLidarCalib::lidar2radar(const double x, const double y) {

    pcl::PointXYZ point_tmp;

    if (std::abs(x) > 1e-6 || std::abs(y) > 1e-6){
       
        point_tmp.x = x;
        point_tmp.y = y;
        point_tmp.z = 0.0;

    }

    Eigen::MatrixXd X(4, 1);
    Eigen::MatrixXd Y(4, 1);

    X(0) = point_tmp.x;
    X(1) = point_tmp.y;
    X(2) = point_tmp.z;
    X(3) = 1.0;

    Y = extRT.inverse() * X;

    point_tmp.x = Y(0);
    point_tmp.y = Y(1);
    point_tmp.z = Y(2);

    return point_tmp;
}

void RadarLidarCalib::loadExtInt() {

    // double ida[] = { 1.0,  0.0,  0.0,
    //                  0.0,  1.0,  0.0,
    //                  0.0,  0.0,  1.0};



    std::array<double, 9> ida = {1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0};

    // 修改内容
    ida = {.999954, -0.0070342, -0.00481069,
            -0.00516621, 0.995131, -0.00063,
            0.00677553, 0.0000851, 0.99514776};


    std::cout << "Loading radar2lidar params." << std::endl;

    std::vector<double> tmp_rot(ida.begin(), ida.end());
    extRotV = tmp_rot;

    std::cout<< "radar2lidar_rot: " << "\n";
    for(int i = 0; i < extRotV.size(); ++i) {
        std::cout << extRotV[i] << " ";
        if((i+1) % 3 == 0) std::cout << std::endl;
    }


    std::array<double, 3> zea = {0.0, 0.0, 0.0};
    zea = {0.0, 0.0, -0.04};

    std::vector<double> ze(zea.begin(), zea.end());
    extTransV = ze;

    std::cout<< "radar2lidar_trans: " << "\n" ;
    for(int i = 0; i < extTransV.size(); ++i) {
        std::cout << extTransV[i] << " ";
        if((i+1) % 3 == 0) std::cout << std::endl;
    }

    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
 
    extRT.block<3, 3>(0, 0) = extRot;
    extRT.block<3, 1>(0, 3) = extTrans;
    extRT.row(3) << 0, 0, 0, 1;

    std::cout<< "--->loadExtInt end!!! "  << std::endl;

    usleep(100);
}

#endif 
