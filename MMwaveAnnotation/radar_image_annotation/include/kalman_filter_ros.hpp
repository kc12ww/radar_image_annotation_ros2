//
// Created by ww on 8/21/24.
//

#ifndef CLOUD_ANNOTATION_KALMAN_FILTER_HPP
#define CLOUD_ANNOTATION_KALMAN_FILTER_HPP


#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


#define NUM_HEURISTIC_MAX_PTS 3000



class RadarKalmanFilter : public rclcpp::Node {
public:
    RadarKalmanFilter() : Node("RadarAnnotationKalmanFilter") {

        RCLCPP_INFO_STREAM(get_logger(), "\033[5;32m" << "RadarAnnotationKalmanFilter starting" << "\033[0m");
        pub_interest_one_pc = create_publisher<sensor_msgs::msg::PointCloud2>("kalman_filter/test", 1);
        RCLCPP_INFO_STREAM(get_logger(), "\033[5;32m" << "RadarAnnotationKalmanFilter ending" << "\033[0m");

    }

    void addOrUpdateTarget(int target_id, const VectorXd& measurement) {
        if (kalman_filters_.find(target_id) == kalman_filters_.end()) {
            // New target, initialize state
            KalmanFilterState new_target;
            new_target.x_ = measurement;
            new_target.P_ = MatrixXd::Identity(4, 4);
            kalman_filters_[target_id] = new_target;
        } else {
            // Existing target, update state
            kalman_filters_[target_id].x_ = stateUpdate(kalman_filters_[target_id], measurement);
        }
    }

    void filterMain(const std::unordered_map<int, VectorXd>& measurements) {
        RCLCPP_INFO_STREAM(get_logger(), "\033[5;32m" << "filterMain starting" << "\033[0m");

        pcl::PointCloud<PointT> estimated_points_cloud;

        for (const auto& [target_id, measurement] : measurements) {
            addOrUpdateTarget(target_id, measurement);

            // Retrieve the updated state for publishing
            const auto& state_estimate = kalman_filters_[target_id].x_;
            PointT estimated_point;
            estimated_point.x = state_estimate(0);
            estimated_point.y = state_estimate(1);
            estimated_point.z = 0; // Assuming 2D tracking in XY plane
            estimated_points_cloud.push_back(estimated_point);
        }

        sensor_msgs::msg::PointCloud2 cloud_ROS;
        pcl::toROSMsg(estimated_points_cloud, cloud_ROS);
        cloud_ROS.header.stamp = clock.now();
        cloud_ROS.header.frame_id = "laser_link";
        pub_interest_one_pc->publish(cloud_ROS);

        RCLCPP_INFO_STREAM(get_logger(), "\033[5;32m" << "filterMain ending" << "\033[0m");
    }

private:
    struct KalmanFilterState {
        VectorXd x_;  // State vector
        MatrixXd P_;  // Covariance matrix
    };

    VectorXd stateUpdate(KalmanFilterState& kf_state, const VectorXd& z) {
        VectorXd x_pred = F_ * kf_state.x_;
        MatrixXd P_pred = F_ * kf_state.P_ * F_.transpose() + Q_;
        
        VectorXd y = z - H_ * x_pred;
        MatrixXd S = H_ * P_pred * H_.transpose() + R_;
        MatrixXd K = P_pred * H_.transpose() * S.inverse();
        
        kf_state.x_ = x_pred + K * y;
        kf_state.P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_pred;
        
        return kf_state.x_;
    }

    double dt_ = 0.2;
    MatrixXd F_ = (MatrixXd(4, 4) << 1, 0, dt_, 0, 0, 1, 0, dt_, 0, 0, 1, 0, 0, 0, 0, 1).finished();
    MatrixXd H_ = MatrixXd::Identity(4, 4);
    MatrixXd Q_ = MatrixXd::Identity(4, 4) * 0.01;
    MatrixXd R_ = MatrixXd::Identity(4, 4) * 0.1;

    std::unordered_map<int, KalmanFilterState> kalman_filters_;  // Maps target IDs to Kalman filter states
    rclcpp::Clock clock;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_interest_one_pc;
};





#endif //CLOUD_ANNOTATION_KALMAN_FILTER_HPP
