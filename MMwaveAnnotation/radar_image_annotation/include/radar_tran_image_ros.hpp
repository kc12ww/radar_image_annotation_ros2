#ifndef CLOUD_ANNOTATION_RADAR_TRAN_IMAGE_HPP
#define CLOUD_ANNOTATION_RADAR_TRAN_IMAGE_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
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

#include <visualization_msgs/msg/marker_array.hpp>


using namespace std;
using std::placeholders::_1;

#define MAX_RADAR_TIME_GAP 15 * 1e6
typedef pcl::PointXYZI PointType;

class RadarCameraCalib : public rclcpp::Node {
public:
    RadarCameraCalib():Node("radar_auto_annotation"){
        radar_points = std::make_shared<pcl::PointCloud<PointType>>();
        loadExtInt();
    }
    void loadExtInt();
    std::vector<cv::Point> radar2camera(const double dist_lat, const double dist_long);



private:
    vector<double> extRotV;
    vector<double> extTransV;
    vector<double> intRotV;

    Eigen::Matrix3d extRot;
    Eigen::Vector3d extTrans;
    Eigen::Matrix4d extRT;
    Eigen::Matrix3d intRot;

    std::string radarDataFile;
    pcl::PointCloud<PointType>::Ptr radar_points;
    std::vector<cv::Point> all_points;
    Eigen::Matrix4d calibration_extRT;
    Eigen::Matrix4d orign_calibration_extRT;
    Eigen::Matrix3d calibration_intRot;
    Eigen::Matrix3d orign_calibration_intRot;
    Eigen::Matrix<double, 3, 4> orign_calibration_intRot_eigen;
};




std::vector<cv::Point> RadarCameraCalib::radar2camera(const double dist_lat, const double dist_long) {

    pcl::PointCloud<PointType> radar_pc;

    radar_points->clear();

    //初始化 雷达点云 图像矩阵 标定外参内参，旋转矩阵

    radar_points = radar_points;
    calibration_extRT = extRT;
    orign_calibration_extRT = extRT;
    calibration_intRot = intRot;
    orign_calibration_intRot = intRot;
    //初始化内部旋转矩阵
    orign_calibration_intRot_eigen.block<3, 3>(0, 0) = orign_calibration_intRot;
    orign_calibration_intRot_eigen.col(3).setZero();

    if (std::abs(dist_long) > 1e-6 || std::abs(dist_lat) > 1e-6){

        pcl::PointXYZI radar_point;
        radar_point.x = dist_long;
        radar_point.y = dist_lat;
        radar_points->points.push_back(radar_point);
        PointType point_tmp;
        point_tmp.x = dist_long;
        point_tmp.y = dist_lat;
        if(point_tmp.y > 0) point_tmp.z = 0.0;
        radar_pc.push_back(point_tmp);

    }

    const int point_nums = radar_pc.points.size();
    Eigen::MatrixXd X(4, point_nums);
    Eigen::MatrixXd Y(4, point_nums);

    for(int i = 0; i < radar_pc.points.size(); ++i) {
        X(0, i) = radar_pc.points[i].x;
        X(1, i) = radar_pc.points[i].y;
        X(2, i) = radar_pc.points[i].z;
        X(3, i) = 1.0;
    }


    Eigen::Matrix4d lidar_extRT;
    lidar_extRT <<
                0, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, 1, -0.05,
            0, 0, 0, 1;

    lidar_extRT =  lidar_extRT.inverse();

    Y = orign_calibration_intRot_eigen * calibration_extRT * lidar_extRT * X;

    all_points.clear();
    for(int i = 0; i < radar_pc.points.size(); ++i) {
        cv::Point poi;
        poi.x = Y(0, i) / Y(2, i);
        poi.y = Y(1, i) / Y(2, i);
        all_points.push_back(poi);
        //std::cout << "===转换后的雷达点坐标===" << poi.x << " 和 " << poi.y << std::endl;

    }
    return all_points;
}


void RadarCameraCalib::loadExtInt() {
    double ida[] = { 1.0,  0.0,  0.0,
                     0.0,  1.0,  0.0,
                     0.0,  0.0,  1.0};

    std::vector < double > tmp_rot(ida, std::end(ida));

    std::cout << "Loading radar2camera params." << std::endl;

    declare_parameter("radar2camera_rot", tmp_rot);
    get_parameter("radar2camera_rot", extRotV);
    std::cout<< "radar2camera_rot: " << "\n";
    for(int i = 0; i < extRotV.size(); ++i) {
        std::cout << extRotV[i] << " ";
        if((i+1) % 3 == 0) std::cout << std::endl;
    }

    double zea[] = {0.0, 0.0, 0.0};
    std::vector < double > ze(zea, std::end(zea));
    declare_parameter("radar2camera_trans", ze);
    get_parameter("radar2camera_trans", extTransV);
    std::cout<< "radar2camera_trans: " << "\n" ;
    for(int i = 0; i < extTransV.size(); ++i) {
        std::cout << extTransV[i] << " ";
        if((i+1) % 3 == 0) std::cout << std::endl;
    }

    declare_parameter("camera_intrinsic", tmp_rot);
    get_parameter("camera_intrinsic", intRotV);
    std::cout<< "camera_intrinsic: " << "\n";
    for(int i = 0; i < intRotV.size(); ++i) {
        std::cout << intRotV[i] << " ";
        if((i+1) % 3 == 0) std::cout << std::endl;
    }

    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    intRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(intRotV.data(), 3, 3);

    extRT.block<3, 3>(0, 0) = extRot;
    extRT.block<3, 1>(0, 3) = extTrans;
    extRT.row(3) << 0, 0, 0, 1;

    std::cout<< "--->loadExtInt end!!! "  << std::endl;

    usleep(100);
}





#endif //CLOUD_ANNOTATION_RADAR_TRAN_IMAGE_HPP
