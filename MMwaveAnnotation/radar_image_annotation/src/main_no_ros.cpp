//
// Created by ww on 7/11/24.
// Modified by DPL 10/18/2024
//

/**
Topic: /back/radar_cluster_cloud | Type: visualization_msgs/msg/MarkerArray | Count: 1920 | Serialization Format: cdr
Topic: /back/radar_objects | Type: ars_40x_msgs/msg/ObjectExtendedArray | Count: 1887 | Serialization Format: cdr
Topic: /back/radar_state | Type: ars_40x_msgs/msg/RadarState | Count: 136 | Serialization Format: cdr
*/

// #include <rclcpp/rclcpp.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vtkRenderingFreeTypeModule.h>
#include <algorithm>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include "radar_tran_image.hpp"
#include "radar_tran_lidar.hpp"
#include "kalman_filter.hpp"
#include "mutil_object_id_tracker.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace fs = std::filesystem;
using namespace std;
using std::placeholders::_1;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer Visualizer;

#define CHECK_SQ(x, y, a, b) sqrt(pow(x - a, 2) + pow(y - b, 2))

struct DataObject
{
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

class PointCloudSelector
{
public:
    PointCloudSelector() : running(true), file_index_(1), selected_points_count_(0), frame_num_(0)
    {

        loadParams();
        updateImage();

        viewer_->registerPointPickingCallback(&PointCloudSelector::pointPick_callback, *this, (void *)&cb_args_);
        viewer_->registerKeyboardCallback(&PointCloudSelector::keyboardEventOccurred, *this, (void *)&cb_args_);
        cv::setMouseCallback(windowName, onMouse, this);

        updatePointCloud();
    }

private:
    struct callback_args
    {
        PointCloudT::Ptr clicked_points_3d;
        std::shared_ptr<Visualizer> viewerPtr;
    };

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *cookie)
    {

        updateImage();
        // 检查事件是否为按键按下
        if (event.keyDown())
        {
            // 获取按下的键的符号
            std::string keySym = event.getKeySym();
            std::cout << "keySym " << keySym << std::endl;

            // 根据不同的键符号执行不同的操作
            if (keySym == "d" || keySym == "D")
            {
                // std::cout << "keySym " << keySym << std::endl;
                dynamic_d_flag_ = true;
                dynamic_s_flag_ = false;

                std::cout << "Update Forward...\n";
                if (save_annotation_radar_flag_ == true)
                {
                    writeCSV(file_index_, file_name_annotationed_, ".csv", loaded_data_);
                }
                clearParams();

                file_index_++;

                updatePointCloud();
                updateImage();
            }
            else if (keySym == "s" || keySym == "S")
            {
                std::cout << "Update Backwards...\n";
                if (save_annotation_radar_flag_ == true)
                {
                    writeCSV(file_index_, file_name_annotationed_, ".csv", loaded_data_);
                }
                dynamic_s_flag_ = true;
                dynamic_d_flag_ = false;

                clearParams();

                if (--file_index_ < 0)
                    file_index_ = 0;

                updatePointCloud();
                updateImage();
            }
            else if (keySym == "1")
            {
                // std::cout << "keySym " << keySym << std::endl;
                dynamic_d_flag_ = true;
                dynamic_s_flag_ = false;

                std::cout << "Update Forward...\n";
                if (save_annotation_radar_flag_ == true)
                {
                    writeCSV(file_index_, file_name_annotationed_, ".csv", loaded_data_);
                }
                clearParams();

                file_index_ += 10;

                updatePointCloud();
                updateImage();
            }
            else if (keySym == "2")
            {
                std::cout << "Update Backwards...\n";
                if (save_annotation_radar_flag_ == true)
                {
                    writeCSV(file_index_, file_name_annotationed_, ".csv", loaded_data_);
                }
                dynamic_s_flag_ = true;
                dynamic_d_flag_ = false;

                clearParams();

                file_index_ -= 10;

                if (file_index_ < 0)
                    file_index_ = 0;

                updatePointCloud();
                updateImage();
            }
        }
    }

    void clearParams()
    {
        loaded_data_.clear();
        loaded_annoed_data_.clear();
        callback_data_->clicked_points_3d->points.clear();
        all_clicked_points_.clear();
        filter_points_.clear();
    }

    void pointPick_callback(const pcl::visualization::PointPickingEvent &event, void *args)
    {

        std::cout << "pointPick start" << std::endl;

        callback_data_ = (callback_args *)args;
        args_flag = true;
        if (event.getPointIndex() == -1)
        {
            return;
        }

        callback_data_->clicked_points_3d->points.clear();

        PointT current_point;
        event.getPoint(current_point.x, current_point.y, current_point.z);

        checkPickPoint(current_point);

        pre_clicked_points_.clear();

        for (int tmp = 0; tmp < all_clicked_points_.size(); ++tmp)
        {
            pre_clicked_points_.push_back(all_clicked_points_[tmp].second);
            std::cout << "all_clicked_points_" << tmp << ": " << all_clicked_points_[tmp].second << std::endl;
            callback_data_->clicked_points_3d->points.push_back(all_clicked_points_[tmp].second);
        }

        pcl::visualization::PointCloudColorHandlerCustom<PointT> red(callback_data_->clicked_points_3d, 255, 0, 0);
        callback_data_->viewerPtr->removePointCloud("clicked_points");
        callback_data_->viewerPtr->addPointCloud(callback_data_->clicked_points_3d, red, "clicked_points");
        callback_data_->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 45, "clicked_points");

        viewer2_->removePointCloud("clicked_points");
        viewer2_->addPointCloud(callback_data_->clicked_points_3d, red, "clicked_points");
        viewer2_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 45, "clicked_points");
        viewer2_->getRenderWindow()->Render();

        std::cout << "event.getPointIndex(): " << event.getPointIndex() << std::endl;
        std::cout << "clicked_points_3d->points size: " << callback_data_->clicked_points_3d->points.size() << std::endl;
        std::cout << "current_point: \n"
                  << current_point << std::endl;

        std::cout << "pointPick end" << std::endl;
    }

    void checkPickPoint(const PointT &current_point)
    {
        auto it = std::find_if(all_clicked_points_.begin(), all_clicked_points_.end(),
                               [&current_point](const pair<int, PointT> &point)
                               {
                                   return CHECK_SQ(point.second.x, point.second.y, current_point.x, current_point.y) < 1.0f;
                               });
        if (it != all_clicked_points_.end())
        {
            // 如果找到 current_point，则移除
            all_clicked_points_.erase(it);
        }
        else
        {
            // 如果未找到 current_point，则添加
            for (auto &obj : loaded_data_)
            {
                pcl::PointXYZ p = radar_lidar_trans.radar2lidar(obj.dist_long, obj.dist_lat);
                if (CHECK_SQ(p.x, p.y, current_point.x, current_point.y) < 1.0f)
                {
                    all_clicked_points_.push_back({obj.id, current_point});
                    break;
                }
            }
        }
    }

    
    // 打印DataObject信息的函数
    void printData(const std::vector<DataObject> &data)
    {
        for (const auto &obj : data)
        {
            // 使用 std::setw 和 std::setprecision 设置输出格式
            std::cout << std::setw(5) << obj.frame << " "
                      << std::setw(5) << obj.id << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.dist_long << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.dist_lat << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.vrel_long << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.vrel_lat << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.rcs << " "
                      << std::setw(5) << obj.dyn_prop << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.arel_long << " "
                      << std::setw(10) << obj.tag << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.arel_lat << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.orientation_angle << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.length << " "
                      << std::setw(10) << std::fixed << std::setprecision(2) << obj.width << " "
                      << std::setw(5) << obj.class_type
                      << std::endl;
        }
    }

    void checkPickPointWrite(std::vector<DataObject> &data)
    {

        PointT current_point;

        // printData(data);

        for (auto &obj : data)
        {
            pcl::PointXYZ p = radar_lidar_trans.radar2lidar(obj.dist_long, obj.dist_lat);
            current_point.x = p.x;
            current_point.y = p.y;
            current_point.z = 0.0;

            auto it = std::find_if(all_clicked_points_.begin(), all_clicked_points_.end(),
                                   [&current_point](const pair<int, PointT> &point)
                                   {
                                       return CHECK_SQ(point.second.x, point.second.y, current_point.x, current_point.y) < 0.5f;
                                   });
            if (it != all_clicked_points_.end())
            {
                // 如果找到 current_point，则标记为1
                obj.class_type = 1;
            }
            else
            {
                obj.class_type = 0;
            }
        }
        // printData(data);
    }

    void updateImage()
    {
        std::string next_image_file = generateNextFileName(".jpg");
        cv::Mat current_image = cv::imread(next_image_file);

        if (current_image.empty())
        {
            std::cerr << "Error: Could not open or find the image " << next_image_file << std::endl;
            return;
        }

        cv::putText(current_image, "frame:" + std::to_string(file_index_), cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);

        showImagePoint(current_image);

        cv::imshow(windowName, current_image);

        // 检查是否按下了'ESC'键
        if (cv::waitKey(10) == 27)
        {
            cv::destroyWindow(windowName);
            return;
        }
    }

    /**
     * 点云投影到图像上
     */
    void showImagePoint(cv::Mat &current_image)
    {

        int test_tmp = 1;
        for (const auto &result : all_clicked_points_)
        {
            // 坐标转换为像素坐标系
            std::vector<cv::Point> xy_point = radar_trans.radar2camera(result.second.x, result.second.y);
            double x = xy_point[0].x;
            double y = xy_point[0].y;
            
            // x = 10 + test_tmp * 10;
            // y = 20 + test_tmp * 20;
            // test_tmp++;
            std::cout << "--------------->" << result.second.x << " ," << result.second.y << std::endl;
            std::cout << "--------------->" << x << ", " << y << std::endl;


            // 绘制矩形框
            int box_size = 20; // 矩形框大小
            cv::Rect rect(x - box_size / 2, y - box_size / 2, box_size, box_size + 80);
            cv::Scalar color(0, 255, 0);

            cv::rectangle(current_image, rect, color, 1.4); // 红色矩形框
            cv::circle(current_image, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);

            PointT current_point;
            current_point.x = result.second.x;
            current_point.y = result.second.y;
            current_point.z = 0.0;

            filter_points_.push_back({result.first, current_point});
        }
    }

    static void onMouse(int event, int x, int y, int flags, void *userdata)
    {

        if (event == cv::EVENT_LBUTTONDOWN)
        {
            PointCloudSelector *self = static_cast<PointCloudSelector *>(userdata);
            self->mouse_x = x;
            self->mouse_y = y;
            self->mouse_clicked = true;
        }
    }

    void pointcloud_remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
    {
        if (cloud->size() > 0)
        {
            // 对点云进行下采样
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<PointT> voxel_grid;
            voxel_grid.setInputCloud(cloud);
            voxel_grid.setLeafSize(0.1, 0.1, 0.1); // 设置体素格子大小
            voxel_grid.filter(*cloud_downsampled);

            //------------------------------------------点云分割--------------------------------------------------------
            // 创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // 创建分割对象
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // 可选择配置，设置模型系数需要优化
            seg.setOptimizeCoefficients(true);
            // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
            seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型
            seg.setMethodType(pcl::SAC_RANSAC);    // 设置随机采样一致性方法类型
            seg.setMaxIterations(1000);            // 最大迭代次数
            seg.setDistanceThreshold(0.5);         // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
            // seg.setInputNormals(normals);
            seg.setInputCloud(cloud_downsampled); // 输入所需要分割的点云对象
            // 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
            seg.segment(*inliers, *coefficients);
            //---------------------------------------------------------------------------------------------------------
            if (inliers->indices.size() == 0)
            {
                cout << "error! Could not found any inliers!" << endl;
            }
            // extract ground
            // 从点云中抽取分割的处在平面上的点集
            pcl::ExtractIndices<pcl::PointXYZ> extractor; // 点提取对象
            extractor.setInputCloud(cloud_downsampled);
            extractor.setIndices(inliers);
            extractor.setNegative(true);
            extractor.filter(*cloud_filtered);
            // vise-versa, remove the ground not just extract the ground
            // just setNegative to be true
            cout << "PointCloud filter done." << endl;
        }
        else
        {
            cout << "no PointCloud data!" << endl;
        }
    }

    std::vector<Eigen::Vector4f> pointcloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_clustered)
    {
        // 桌子平面上的点云团使用欧式聚类的算法　kd树搜索　对点云聚类分割
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);                        // 　桌子平面上其他的点云
        std::vector<pcl::PointIndices> cluster_indices;    // 点云团索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // 欧式聚类对象
        ec.setClusterTolerance(0.2);                       // 设置近邻搜索的搜索半径为20cm
        ec.setMinClusterSize(50);                          // 设置一个聚类需要的最少的点数目为100
        ec.setMaxClusterSize(25000);                       // 设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod(tree);                          // 设置点云的搜索机制
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中

        std::vector<Eigen::Vector4f> ret;

        // 迭代访问点云索引cluster_indices,直到分割处所有聚类
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
        {
            double min_x = 100, max_x = -100, min_y = 100, max_y = -100, min_z = 100, max_z = -100;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cloud_cluster->points.push_back(cloud->points[*pit]); // 获取每一个点云团的点
                min_x = fmin(min_x, cloud->points[*pit].x);
                max_x = fmax(max_x, cloud->points[*pit].x);
                min_y = fmin(min_y, cloud->points[*pit].y);
                max_y = fmax(max_y, cloud->points[*pit].y);
                min_z = fmin(min_z, cloud->points[*pit].z);
                max_z = fmax(max_z, cloud->points[*pit].z);
            }

            std::cout << min_x << " " << max_x << " ," << min_y << " " << max_y << " ," << min_z << " " << max_z << std::endl;
            double rel_z = fabs(min_z - max_z);
            double v = fabs((min_x - max_x) * (min_y - max_y) * rel_z);
            std::cout << "cluster volume is:" << v << std::endl;
            if (v < 0.1f || v > 1.0f || rel_z > 2.0f)
                continue;

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            std::cout << "Cluster " << j << " : " << cloud_cluster->points.size() << " data points. " << std::endl;

            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud_cluster, pcaCentroid); // 计算点云质心
            std::cout << "Cluster center: (" << pcaCentroid[0] << "," << pcaCentroid[1] << "," << pcaCentroid[2] << ")" << std::endl;
            // if(cloud_cluster->width > 250 ) continue;
            // if(pcaCentroid[2] > 1.0f) continue;
            ret.push_back(pcaCentroid);
            *cloud_clustered += *cloud_cluster;
            std::cout << "done." << std::endl;
        }
        std::cout << "PointCloud Cluster done. " << std::endl;
        return ret;
    }

    void updatePointCloud()
    {

        std::string next_lidar_file = generateNextFileName(".pcd");

        PointCloudT::Ptr lidar_cloud(new PointCloudT);
        if (pcl::io::loadPCDFile(next_lidar_file, *lidar_cloud) < 0)
        {
            std::cerr << "lidar cloud " << next_lidar_file.c_str() << " 不存在！" << std::endl;

            return;
        }

        std::string next_file = generateNextFileName(".csv");

        *curr_frame_pc_ = *lidar_cloud;
        if (args_flag == false)
        {
            pre_frame_pc_ = curr_frame_pc_;
            std::cout << "pre_frame_pc = curr_frame_pc init..." << std::endl;
        }

        if (radar_track_flag_ == true)
        {
            std::cout << "-------dynamicTrackingRadar------>" << std::endl;
            dynamicTrackingRadar();
        }

        PointCloudT::Ptr new_cloud(new PointCloudT);
        if (lidar_cluster_flag_ == true)
        {

            // lidar pointcloud filter
            PointCloudT::Ptr lidar_cloud_filtered(new PointCloudT);
            pointcloud_remove_plane(lidar_cloud, lidar_cloud_filtered);
            PointCloudT::Ptr lidar_cloud_clustered(new PointCloudT);

            // lidar pointcloud cluster
            std::vector<Eigen::Vector4f> cluster_centers;
            cluster_centers = pointcloud_cluster(lidar_cloud_filtered, lidar_cloud_clustered);

            std::cout << "Shift + 鼠标左键选点，按 ‘Q’结束选点" << std::endl;

            std::cout << " file_index_:" << file_index_ << std::endl;

            std::string cur_csv_file = generateNextFileName(".csv");

            loaded_data_ = readCSV(cur_csv_file);
            before_loaded_data_ = readCSVTracking(cur_csv_file);

            // 判断是否为：加载之前的标定数据
            // cur_csv_file = generateNextFileNameBack(".csv");

            std::string annoed_csv_file = generateNextFileNameBack(".csv");
            loaded_annoed_data_ = readCSV(annoed_csv_file);

            std::vector<DataObject> data_tmp;
            data_tmp = readCSV(cur_csv_file);
            if (data_tmp.size() > 0)
            { // if read from annotation, store into loaded_data_
                std::cout << "last annotation size: " << data_tmp.size() << std::endl;

                loaded_data_ = readCSV(cur_csv_file);
            }

            // std::cout << " loaded_data_ size:" << loaded_data_.size() << std::endl;
            std::cout << "重新加载点的个数为：%zu" << loaded_data_.size() << std::endl;

            all_clicked_points_.clear();

            PointT current_point;

            for (const auto &obj : loaded_data_)
            {
                pcl::PointXYZ p = radar_lidar_trans.radar2lidar(obj.dist_long, obj.dist_lat);

                new_cloud->push_back(p);

                current_point.x = p.x;
                current_point.y = p.y;
                current_point.z = 0.0;

                if (!(data_tmp.size() > 0)) // if read from raw csv
                {
                    if (CHECK_SQ(0, 0, current_point.x, current_point.y) > 100.0f)
                        continue; // distance : x threshold
                    if (fabs(current_point.y) > 5)
                        continue; // width : y threshold
                    if (obj.class_type == 0)
                    {
                        // uncomment if you want to use auto annotation, please set the threshold by yourself
                        for (auto it = cluster_centers.begin(); it != cluster_centers.end(); ++it)
                        {
                            if (CHECK_SQ(it->data()[0], it->data()[1], current_point.x, current_point.y) < 0.8f)
                            {
                                all_clicked_points_.push_back({obj.id, current_point});
                            }
                        }
                    }
                    else if (obj.class_type == 1)
                    {
                        all_clicked_points_.push_back({obj.id, current_point});
                        std::cout << "obj.class_type == 1 : " << current_point << std::endl;
                    }
                }
                else
                { // if read from annotated csv
                    if (obj.class_type == 1)
                    {
                        all_clicked_points_.push_back({obj.id, current_point});
                        std::cout << "obj.class_type == 1 : " << current_point << std::endl;
                    }
                }
            }

            *radar_cloud_ = *new_cloud;
        }

        {
            boost::lock_guard<boost::mutex> guard(cloud_mutex_);

            viewer_->removeAllPointClouds();
            viewer_->addPointCloud(radar_cloud_, "pick");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "pick"); // 设置点云大小

            viewer2_->removeAllPointClouds();
            pcl::visualization::PointCloudColorHandlerCustom<PointT> white(radar_cloud_, 255, 255, 255);
            viewer2_->addPointCloud(radar_cloud_, white, "radar");
            viewer2_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "radar");

            pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow(lidar_cloud, 255, 255, 0);
            viewer2_->addPointCloud(lidar_cloud, yellow, "lidar"); // options: lidar_cloud_filtered, lidar_cloud_clustered
            viewer2_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "lidar");

            std::cout << "args_flag: " << args_flag << std::endl;
            if (args_flag == true)
            {

                std::cout << "all_clicked_points_.size(): -------" << all_clicked_points_.size() << std::endl;
                for (int tmp = 0; tmp < all_clicked_points_.size(); ++tmp)
                {
                    std::cout << "all_clicked_points_" << tmp << ": " << all_clicked_points_[tmp].second << std::endl;
                    callback_data_->clicked_points_3d->points.push_back(all_clicked_points_[tmp].second);
                }

                pcl::visualization::PointCloudColorHandlerCustom<PointT> red(callback_data_->clicked_points_3d, 255, 0, 0);
                viewer_->addPointCloud(callback_data_->clicked_points_3d, red, "clicked_points");
                viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 25, "clicked_points");
                viewer2_->addPointCloud(callback_data_->clicked_points_3d, red, "clicked_points");
                viewer2_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 45, "clicked_points");
                showTrackingPoints();
                showFilterPoints();
            }
            viewer2_->getRenderWindow()->Render();
        }

        // Ensure the viewer is refreshed
        if (!viewer_->wasStopped() && !viewer2_->wasStopped())
        {
            std::cout << "file_index_: " << file_index_ << std::endl;
            viewer_->spin();
            viewer2_->spin();
        }
    }

    void showPickedPoints()
    {

        std::cout << "all_clicked_points_.size(): -------" << all_clicked_points_.size() << std::endl;
        for (int tmp = 0; tmp < all_clicked_points_.size(); ++tmp)
        {
            std::cout << "all_clicked_points_" << tmp << ": " << all_clicked_points_[tmp].second << std::endl;
            callback_data_->clicked_points_3d->points.push_back(all_clicked_points_[tmp].second);
        }

        pcl::visualization::PointCloudColorHandlerCustom<PointT> red(callback_data_->clicked_points_3d, 255, 0, 0);
        viewer_->addPointCloud(callback_data_->clicked_points_3d, red, "Picked_Points");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 45, "Picked_Points");
        viewer2_->addPointCloud(callback_data_->clicked_points_3d, red, "Picked_Points");
        viewer2_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 45, "Picked_Points");
    }

    void showFilterPoints()
    {

        std::cout << "filter_points.size(): -------" << filter_points_.size() << std::endl;
        for (int tmp = 0; tmp < filter_points_.size(); ++tmp)
        {
            callback_data_->clicked_points_3d->points.push_back(filter_points_[tmp].second);
        }
        pcl::visualization::PointCloudColorHandlerCustom<PointT> green(callback_data_->clicked_points_3d, 0, 255, 0);
        viewer_->removePointCloud("filter_points");
        viewer_->addPointCloud(callback_data_->clicked_points_3d, green, "filter_points");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "filter_points");
        viewer2_->removePointCloud("filter_points");
        viewer2_->addPointCloud(callback_data_->clicked_points_3d, green, "filter_points");
        viewer2_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "filter_points");
    }

    void showTrackingPoints()
    {

        std::cout << "showTrackingPoints: -------" << tracking_point_ << std::endl;

        callback_data_->clicked_points_3d->points.push_back(tracking_point_);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(callback_data_->clicked_points_3d, 0, 0, 255);
        viewer_->removePointCloud("tracking_points");
        viewer_->addPointCloud(callback_data_->clicked_points_3d, blue, "tracking_points");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "tracking_points");
    }

    std::vector<DataObject> readCSV(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Error opening file for reading!---" << filename << std::endl;
            annotation_csv_flag = false;
            return {};
        }

        std::vector<DataObject> data;
        std::string line;

        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            DataObject obj;
            ss >> obj.frame >> obj.id >> obj.dist_long >> obj.dist_lat >> obj.vrel_long >> obj.vrel_lat >> obj.rcs >> obj.dyn_prop >> obj.arel_long >> obj.tag >> obj.arel_lat >> obj.orientation_angle >> obj.length >> obj.width >> obj.class_type;

            data.push_back(obj);
        }

        if (args_flag == true)
        {
            PointT current_point;
            for (const auto &point_tmp : data)
            {
                pcl::PointXYZ p = radar_lidar_trans.radar2lidar(point_tmp.dist_long, point_tmp.dist_lat);
                current_point.x = p.x;
                current_point.y = p.y;
                current_point.z = 0.0;
                checkPickPoint(current_point);
            }
        }

        file.close();
        return data;
    }

    std::shared_ptr<PointCloudT> readCSVToPcd(const std::string &filename)
    {
        auto new_cloud = std::make_shared<PointCloudT>(); // 创建共享指针

        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Error opening file for reading!---" << filename << std::endl;
            return new_cloud; // Return an empty cloud if there's an error
        }

        std::vector<DataObject> data;
        std::string line;

        // Skip the header line if it exists
        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            DataObject obj;
            ss >> obj.frame >> obj.id >> obj.dist_long >> obj.dist_lat >> obj.vrel_long >> obj.vrel_lat >> obj.rcs >> obj.dyn_prop >> obj.arel_long >> obj.tag >> obj.arel_lat >> obj.orientation_angle >> obj.length >> obj.width >> obj.class_type;

            data.push_back(obj);
        }

        PointT current_point;
        for (const auto &point_tmp : data)
        {
            // Convert radar to LiDAR coordinates (dummy example here)
            // pcl::PointXYZ p = pcl::PointXYZ(point_tmp.dist_long, point_tmp.dist_lat, 0.0); // Simplified conversion
            pcl::PointXYZ p = radar_lidar_trans.radar2lidar(point_tmp.dist_long, point_tmp.dist_lat);

            // Set the x and y values from dist_long and dist_lat
            current_point.x = p.x;
            current_point.y = p.y;
            current_point.z = 0.0; // Set z to 0 as specified

            // Add the point to the new point cloud
            new_cloud->points.push_back(current_point);
        }

        file.close();
        return new_cloud; // Return the populated point cloud
    }

    void writeCSV(int frame_number, string radar_file_name, string radar_file_format_csv, std::vector<DataObject> &data)
    {

        std::lock_guard<std::mutex> guard(write_mutex_);

        std::string directory_radar = radar_file_name;
        // 检查目录是否存在，如果不存在则创建目录
        if (!fs::exists(directory_radar))
        {
            fs::create_directory(directory_radar);
            std::cout << "directory_radar %s created." << directory_radar.c_str() << std::endl;
        }

        std::string radar_filename_csv = generate_filename(frame_number, radar_file_name, radar_file_format_csv);
        std::ofstream file_csv(radar_filename_csv);
        if (!file_csv.is_open())
        {
            std::cerr << "Error opening file_csv: " << radar_filename_csv.c_str() << std::endl;
            return;
        }

        checkPickPointWrite(data);

        file_csv << "frame id dist_long dist_lat vrel_long vrel_lat rcs dyn_prop arel_long tag arel_lat orientation_angle length width class" << std::endl;

        // 逐行写入每个对象的属性
        for (const auto &obj : data)
        {
            file_csv << obj.frame << " " << obj.id << " " << obj.dist_long << " " << obj.dist_lat << " "
                     << obj.vrel_long << " " << obj.vrel_lat << " " << obj.rcs << " "
                     << obj.dyn_prop << " " << obj.arel_long << " " << obj.tag << " "
                     << obj.arel_lat << " " << obj.orientation_angle << " " << obj.length << " "
                     << obj.width << " " << obj.class_type << std::endl;
        }
        // for (const auto &obj : data) {
        //     file_csv << obj.frame << "," << obj.id << "," << obj.dist_long << "," << obj.dist_lat << ","
        //              << obj.vrel_long << "," << obj.vrel_lat << "," << obj.rcs << ","
        //              << obj.dyn_prop << "," << obj.arel_long << "," << obj.tag << ","
        //              << obj.arel_lat << "," << obj.orientation_angle << "," << obj.length << ","
        //              << obj.width << "," << obj.class_type << std::endl;
        // }
        file_csv.close();
        std::cout << "Frame data saved successfully. ---" << radar_filename_csv << std::endl;
    }

    // 生成文件名的函数，根据帧数生成文件名
    std::string generate_filename(int frame_number, string file_name, string file_format)
    {
        std::ostringstream filename;
        // filename << file_name << std::setfill('0') << std::setw(5) << frame_number << file_format;
        filename << file_name << frame_number << file_format;
        return filename.str();
    }

    void loadParams()
    {
        std::cout << " loadParams start" << std::endl;

        viewer_ = std::make_shared<Visualizer>("viewer");

        cb_args_.clicked_points_3d = clicked_points_3d_;
        cb_args_.viewerPtr = viewer_;

        viewer2_ = std::make_shared<Visualizer>("viewer2");

        file_index_ = 10;
        std::cout << "file_index_: " << file_index_ << std::endl;

        // pcd_folder_ = "/home/ww/datasets/PanZhiHua/9.bag/alldata/";
        pcd_folder_ = "/home/ww/datasets/LvShun_ros2/perception/1011obs-4_2/all/";

       
        std::cout << " pcd_folder_:" << pcd_folder_ << std::endl;

        save_annotation_radar_flag_ = true;
        std::cout << " save_annotation_radar_flag:" << save_annotation_radar_flag_ << std::endl;

        lidar_cluster_flag_ = false;
        std::cout << " lidar_cluster_flag_:" << lidar_cluster_flag_ << std::endl;

        radar_track_flag_ = true;
        std::cout << " radar_track_flag_:" << radar_track_flag_ << std::endl;

        // file_name_annotationed_ = "/home/ww/datasets/PanZhiHua/9.bag/ann/";
        file_name_annotationed_ = "/home/ww/datasets/LvShun_ros2/perception/1011obs-4_2/ann/";

        std::cout << " file_name_annotationed_:" << file_name_annotationed_ << std::endl;

        before_state_estimate_ = VectorXd::Zero(4);
        before_input_radar_data_ = VectorXd::Zero(4);

        mouse_x = 0;
        mouse_y = 0;
        mouse_clicked = false;
        before_velocity_rep_ = 0.0;
        args_flag = false;
        annotation_csv_flag = true;

        dynamic_s_flag_ = false;
        dynamic_d_flag_ = false;

        windowName = "Dynamic Image Display";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(windowName, 640, 480);

        all_clicked_points_.clear();
        std::cout << " loadParams end" << std::endl;
    }

    std::string generateNextFileName(std::string file_format)
    {
        std::stringstream ss;
        // ss << pcd_folder_ << std::setw(5) << std::setfill('0') << file_index_ << file_format;
        ss << pcd_folder_ << file_index_ << file_format;
        return ss.str();
    }

    std::string generateNextFileNameBack(std::string file_format)
    {
        std::stringstream ss;
        // ss << file_name_annotationed_ << std::setw(5) << std::setfill('0') << file_index_ << file_format;
        ss << file_name_annotationed_ << file_index_ << file_format;
        return ss.str();
    }

    /**加载上一帧的标注后数据 */
    std::string generateCurrentAnnoFileName(std::string file_format)
    {
        std::stringstream ss;
        int current_anno_file_index = file_index_ - 1;
        ss << file_name_annotationed_ << current_anno_file_index << file_format;
        return ss.str();
    }

    bool alignPointCloudsICP(const PointCloudT::Ptr &last_radar_cloud,
                             PointCloudT::Ptr &current_radar_cloud,
                             float threshold,
                             Eigen::Matrix4f &transformation_matrix)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        icp.setInputSource(current_radar_cloud);
        icp.setInputTarget(last_radar_cloud);

        icp.setMaxCorrespondenceDistance(0.05); // 最大对应距离
        icp.setMaximumIterations(100);          // 最大迭代次数
        icp.setTransformationEpsilon(2.0);      // 转换平移量的收敛判定
        icp.setEuclideanFitnessEpsilon(0.1);    // 欧几里得适应度误差的收敛判定

        PointCloudT Final;
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

        initial_guess(0, 3) = 0.1;
        icp.align(Final, initial_guess);

        if (icp.hasConverged() && icp.getFitnessScore() < threshold)
        {
            transformation_matrix = icp.getFinalTransformation();

            std::cout << "ICP has converged." << std::endl;
            std::cout << "Score: " << icp.getFitnessScore() << std::endl;
            std::cout << "Transformation Matrix:" << std::endl
                      << transformation_matrix << std::endl;

            return true; // 成功匹配，返回 true
        }
        else
        {
            std::cout << "ICP did not converge or the fitness score is above the threshold." << std::endl;
            return false; // 匹配失败，返回 false
        }
    }

    bool alignPointCloudsGICP(const PointCloudT::Ptr &last_radar_cloud,
                              PointCloudT::Ptr &current_radar_cloud,
                              float threshold,
                              Eigen::Matrix4f &transformation_matrix)
    {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

        gicp.setInputSource(current_radar_cloud); // 源点云
        gicp.setInputTarget(last_radar_cloud);    // 目标点云

        PointCloudT aligned_cloud;
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        initial_guess(0, 3) = 0.1;
        gicp.align(aligned_cloud, initial_guess);

        if (gicp.hasConverged() && gicp.getFitnessScore() < threshold)
        {
            std::cout << "GICP converged. Fitness score: " << gicp.getFitnessScore() << std::endl;
            transformation_matrix = gicp.getFinalTransformation();
            return true;
        }
        else
        {
            std::cerr << "GICP did not converge." << std::endl;
            transformation_matrix = Eigen::Matrix4f::Identity();
            return false;
        }
    }

    void dynamicTrackingRadar()
    {
        std::string cur_csv_file = generateNextFileName(".csv");
        loaded_data_ = readCSV(cur_csv_file);

        std::shared_ptr<PointCloudT> current_radar_cloud = readCSVToPcd(cur_csv_file);

        std::vector<DataObject> data_tmp; // 上一帧标注后的数据
        cur_csv_file = generateCurrentAnnoFileName(".csv");
        data_tmp = readCSV(cur_csv_file);

        std::shared_ptr<PointCloudT> last_radar_cloud = readCSVToPcd(cur_csv_file);

        float threshold = 0.1f;
        Eigen::Matrix4f transformation_matrix;

        std::string next_lidar_file = generateNextFileName(".pcd");
        std::shared_ptr<PointCloudT> lidar_cloud(new PointCloudT);
        if (pcl::io::loadPCDFile(next_lidar_file, *lidar_cloud) < 0)
        {
            std::cout << "lidar cloud %s 不存在！" << next_lidar_file.c_str() << std::endl;

            return;
        }

        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(current_radar_cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*current_radar_cloud);
        voxel_grid.setInputCloud(last_radar_cloud);
        voxel_grid.filter(*last_radar_cloud);

        if (last_radar_cloud->size() > 0)
        {

            std::vector<int> indices; // 用于存储有效点的索引
            pcl::removeNaNFromPointCloud(*current_radar_cloud, *current_radar_cloud, indices);
            pcl::removeNaNFromPointCloud(*last_radar_cloud, *last_radar_cloud, indices);

            // std::cout << "last_radar_cloud.size():" << last_radar_cloud->size() << std::endl;
            // std::cout << "current_radar_cloud.size():" << current_radar_cloud->size() << std::endl;

            // bool success = alignPointCloudsICP(last_radar_cloud, current_radar_cloud, threshold, transformation_matrix);
            bool success = alignPointCloudsGICP(last_radar_cloud, current_radar_cloud, threshold, transformation_matrix);

            if (success)
            {
                std::cout << "Transformation applied to current_radar_cloud." << std::endl;
                pcl::transformPointCloud(*current_radar_cloud, *current_radar_cloud, transformation_matrix);
            }
            else
            {
                std::cout << "Matching failed or error exceeded threshold." << std::endl;
            }
        }

        all_clicked_points_.clear();

        if (data_tmp.size() > 0)
        {
            std::cout << "last annotation size: " << data_tmp.size() << std::endl;
            for (const auto &obj : data_tmp)
            {
                // 创建 KDTree 对象
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(current_radar_cloud);

                std::vector<int> nearest_indices(1);     // 最近点的索引
                std::vector<float> nearest_distances(1); // 最近点的距离

                if (obj.class_type == 1)
                {

                    PointT current_point_tmp;
                    pcl::PointXYZ p = radar_lidar_trans.radar2lidar(obj.dist_long, obj.dist_lat);

                    // Set the x and y values from dist_long and dist_lat
                    current_point_tmp.x = p.x;
                    current_point_tmp.y = p.y;
                    current_point_tmp.z = 0.0; // Set z to 0 as specified

                    // 查找 tmp 点在 target 点云中的最近点
                    if (kdtree.nearestKSearch(current_point_tmp, 1, nearest_indices, nearest_distances) > 0)
                    {
                        PointT current_nearest_point = (*current_radar_cloud)[nearest_indices[0]];

                        double min_distance = nearest_distances[0];

                        if (min_distance > 2.0)
                            break;

                        // for (auto obj_tmp : loaded_data_)
                        // {
                        //     PointT loaded_current_point_tmp;
                        //     pcl::PointXYZ loaded_p = radar_lidar_trans.radar2lidar(obj_tmp.dist_long, obj_tmp.dist_lat);
                        //     float diff = abs(current_nearest_point.x - loaded_p.x) + abs(current_nearest_point.y - loaded_p.y);
                        //     if (diff < 0.01){

                        //     }
                        // }

                        all_clicked_points_.push_back({obj.id, current_nearest_point});

                        // RCLCPP_INFO_STREAM(get_logger(), "\033[5;32m" << "current_nearest_point_: " << current_nearest_point << "\033[0m");

                        std::cout << "\033[5;32m" << "current_nearest_point_: " << current_nearest_point << "\033[0m" << std::endl;
                    }
                }
            }
        }

        *radar_cloud_ = *current_radar_cloud;

        Eigen::VectorXd cur_radar_data = before_input_radar_data_;

        std::cout << "pre_clicked_points_.size(): " << pre_clicked_points_.size() << std::endl;

        int tmp_i_1 = 0;

        for (auto tmp : pre_clicked_points_)
        {
            // 创建 KDTree 对象
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(curr_frame_pc_);

            std::vector<int> nearest_indices(1);     // 最近点的索引
            std::vector<float> nearest_distances(1); // 最近点的距离

            // 查找 tmp 点在 target 点云中的最近点
            if (kdtree.nearestKSearch(tmp, 1, nearest_indices, nearest_distances) > 0)
            {
                PointT nearest_point = (*curr_frame_pc_)[nearest_indices[0]];

                double min_distance = nearest_distances[0];

                if (min_distance > 2.0)
                    break;

                std::cout << "before_nearest_point_: " << before_nearest_point_ << std::endl;

                for (auto csv_tmp : before_loaded_data_)
                {
                    std::cout << "long ,lat: " << csv_tmp.dist_lat << "," << csv_tmp.dist_long << std::endl;

                    if (before_nearest_point_.x == csv_tmp.dist_lat && before_nearest_point_.y == csv_tmp.dist_long)
                    {
                        cur_radar_data[0] = csv_tmp.dist_long;
                        cur_radar_data[1] = csv_tmp.dist_lat;
                        cur_radar_data[2] = csv_tmp.vrel_long;
                        cur_radar_data[3] = csv_tmp.vrel_lat;
                        break;
                    }
                }

                before_nearest_point_ = nearest_point;
                before_input_radar_data_ = cur_radar_data;
                tmp_i_1++;

                // std::cout << "cur_radar_data: " << cur_radar_data << std::endl;
                // std::cout << "nearest_point: " << nearest_point << std::endl;

                // // if(min_distance < 1.0)
                // // all_clicked_points_.push_back({tmp_i_1, nearest_point});

                // std::cout << "min_distance: " << min_distance << std::endl;
            }
            else
            {
                std::cerr << "No nearest point found." << std::endl;
            }
        }

        pre_frame_pc_ = curr_frame_pc_;
        // mutObjIdTra();

        // kalmanTracking(cur_radar_data);
    }

    void mutObjIdTra()
    {
        // 初始化跟踪的目标（上一帧）
        std::vector<TrackedObject> tracked_objects = {
            {{0.5, 0.5, 0.0}, 1, 0},
            {{2.0, 2.0, 0.0}, 2, 0}};

        // 当前帧检测到的目标
        std::vector<pcl::PointXYZ> detected_objects = {
            {0.6, 0.5, 0.0},
            {2.1, 2.0, 0.0},
            {4.0, 4.0, 0.0}};

        // 最大匹配距离
        double max_distance = 1.0;

        // 调用关联与更新函数
        mut_obj_id_tra_.associateAndUpdate(tracked_objects, detected_objects, max_distance);

        // 输出更新后的目标列表
        std::cout << "Updated Tracked Objects:\n";
        for (const auto &obj : tracked_objects)
        {
            std::cout << "ID: " << obj.id
                      << ", Position: (" << obj.position.x << ", "
                      << obj.position.y << ", " << obj.position.z << ")\n";
        }
    }

    void kalmanTracking(const Eigen::VectorXd &cur_radar_data)
    {

        std::unordered_map<int, VectorXd> measurements = {
            {1, VectorXd::Random(4)},
            {2, VectorXd::Random(4)},
            // Add more targets as needed
        };

        radar_kalman_filter_.filterMain(measurements);

        // VectorXd cur_state_estimate = radar_kalman_filter_.filterMain(cur_radar_data);

        // tracking_point_.x =  cur_state_estimate(1);
        // tracking_point_.y =  cur_state_estimate(0);
        // tracking_point_.z = 0.0;

        // std::cout << "tracking_point_: " << tracking_point_ << std::endl;
        // before_state_estimate_ = cur_state_estimate;
    }

    std::vector<DataObject> readCSVTracking(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Error opening file for reading!---" << filename << std::endl;
            annotation_csv_flag = false;
            return {};
        }

        std::vector<DataObject> data;
        std::string line;

        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            DataObject obj;
            ss >> obj.frame >> obj.id >> obj.dist_long >> obj.dist_lat >> obj.vrel_long >> obj.vrel_lat >> obj.rcs >> obj.dyn_prop >> obj.arel_long >> obj.tag >> obj.arel_lat >> obj.orientation_angle >> obj.length >> obj.width >> obj.class_type;

            data.push_back(obj);
        }

        file.close();
        return data;
    }

public:
    // 模拟处理函数
    void process()
    {
        while (running)
        {
            // 模拟某种处理
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "Processing point cloud data..." << std::endl;
        }
    }

    // 停止处理
    void stop()
    {
        running = false;
    }

    std::atomic<bool> running; // 用于控制线程停止

    // std::string pcd_name_;
    std::string pcd_folder_;
    int file_index_;
    callback_args *callback_data_;

    int selected_points_count_;
    PointCloudT::Ptr radar_cloud_ = std::make_shared<PointCloudT>();

    std::shared_ptr<Visualizer> viewer_;
    std::shared_ptr<Visualizer> viewer2_;
    boost::mutex cloud_mutex_;
    std::mutex write_mutex_;

    callback_args cb_args_;
    PointCloudT::Ptr clicked_points_3d_ = std::make_shared<PointCloudT>();
    int frame_num_;

    std::string windowName;
    std::vector<DataObject> loaded_data_;
    std::vector<DataObject> loaded_annoed_data_;

    bool save_annotation_radar_flag_;
    bool lidar_cluster_flag_;
    bool radar_track_flag_;

    std::thread pick_point_thread_;
    std::mutex mutex_;
    // std::condition_variable cv_;
    bool stop_thread_;
    int before_key_;

    // 点云投影到图像上，并画框
    RadarCameraCalib radar_trans;
    RadarLidarCalib radar_lidar_trans;
    std::vector<DataObject> cloud2image_data_;
    PointCloudT::Ptr cloud2image_ = std::make_shared<PointCloudT>();
    int mouse_x;
    int mouse_y;
    int mouse_x_before, mouse_y_before;
    bool mouse_clicked;
    double before_velocity_rep_;

    bool args_flag;

    // 向后播放数据时，更新标定数据，而不是重新标定
    string file_name_annotationed_;

    // 增加取消已选点的功能，回溯时可视化已选点功能
    // std::vector<PointT> all_clicked_points_;
    std::vector<pair<int, PointT>> all_clicked_points_;
    bool annotation_csv_flag;
    bool dynamic_s_flag_, dynamic_d_flag_;

    // pcl中显示已过滤的点
    // std::vector<PointT> filter_points_;
    std::vector<pair<int, PointT>> filter_points_;

    // 动态跟踪功能
    PointCloudT::Ptr pre_frame_pc_ = std::make_shared<PointCloudT>();
    PointCloudT::Ptr curr_frame_pc_ = std::make_shared<PointCloudT>();

    std::vector<PointT> pre_clicked_points_;
    RadarKalmanFilter radar_kalman_filter_;
    std::vector<DataObject> before_loaded_data_;
    PointT before_nearest_point_;
    PointT tracking_point_;               //
    std::vector<PointT> tracking_points_; //
    Eigen::VectorXd before_state_estimate_;
    Eigen::VectorXd before_input_radar_data_;

    // add objects id track
    MultiTargetIdTracker mut_obj_id_tra_;
    std::map<int, std::vector<PointT>> id_to_points_map;
};

int main(int argc, char **argv)
{

    auto selector = std::make_shared<PointCloudSelector>();

    std::thread processing_thread(&PointCloudSelector::process, selector);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    selector->stop();

    processing_thread.join();

    std::cout << "Program finished." << std::endl;
    return 0;
}