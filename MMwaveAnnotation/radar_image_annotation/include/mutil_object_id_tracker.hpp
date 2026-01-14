#ifndef MUTIL_OBJECT_ID_TRACKER_HPP
#define MUTIL_OBJECT_ID_TRACKER_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 结构体：表示一个被跟踪的目标
struct TrackedObject
{
    pcl::PointXYZ position; // 目标位置
    int id;                 // 唯一 ID
    int age;                // 存在时间（连续未匹配的帧数）
};

// ID 管理器：管理 ID 的分配和回收
class IDManager
{
private:
    int next_id;                   // 下一个分配的 ID
    std::vector<int> recycled_ids; // 回收的 ID

public:
    IDManager() : next_id(0) {}

    // 分配新 ID
    int getNewID()
    {
        if (!recycled_ids.empty())
        {
            int id = recycled_ids.back();
            recycled_ids.pop_back();
            return id;
        }
        return next_id++;
    }

    // 回收 ID
    void recycleID(int id)
    {
        recycled_ids.push_back(id);
    }
};

// 多目标跟踪类
class MultiTargetIdTracker
{
private:
    std::vector<TrackedObject> tracked_objects; // 跟踪的目标列表
    IDManager id_manager;                       // ID 管理器
    double max_distance;                        // 数据关联的最大距离

    // 计算欧几里得距离
    double euclideanDistance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    }

    // 数据关联
    int associateObjects(const pcl::PointXYZ &current_point)
    {
        double min_distance = max_distance;
        int nearest_index = -1;

        for (size_t i = 0; i < tracked_objects.size(); ++i)
        {
            double distance = euclideanDistance(current_point, tracked_objects[i].position);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_index = i;
            }
        }
        return nearest_index;
    }

public:
    MultiTargetIdTracker(double max_distance = 1.0) : max_distance(max_distance) {}

    // 关联上一帧的跟踪目标与当前帧的检测目标
    void associateAndUpdate(std::vector<TrackedObject> &tracked_objects,
                            const std::vector<pcl::PointXYZ> &detected_objects,
                            double max_distance)
    {
        // 标记当前帧中每个检测目标是否已被匹配
        std::vector<bool> matched(detected_objects.size(), false);

        // 遍历所有跟踪的目标
        for (auto &tracked_object : tracked_objects)
        {
            double min_distance = max_distance;
            int nearest_index = -1;

            // 在当前帧的检测目标中找到最近的目标
            for (size_t i = 0; i < detected_objects.size(); ++i)
            {
                if (!matched[i])
                {
                    double distance = euclideanDistance(tracked_object.position, detected_objects[i]);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        nearest_index = i;
                    }
                }
            }

            // 如果找到了最近的目标，更新跟踪目标的位置
            if (nearest_index != -1)
            {
                tracked_object.position = detected_objects[nearest_index];
                tracked_object.age = 0;        // 重置未匹配计数
                matched[nearest_index] = true; // 标记该检测目标已被匹配
            }
            else
            {
                tracked_object.age++; // 如果未匹配到，增加未匹配计数
            }
        }

        // 对未被匹配的检测目标，创建新的跟踪目标
        for (size_t i = 0; i < detected_objects.size(); ++i)
        {
            if (!matched[i])
            {
                TrackedObject new_object;
                new_object.position = detected_objects[i];
                new_object.id = tracked_objects.size() + 1; // 为新目标分配新 ID
                new_object.age = 0;
                tracked_objects.push_back(new_object);
            }
        }

        // 移除长时间未匹配的目标
        tracked_objects.erase(
            std::remove_if(tracked_objects.begin(), tracked_objects.end(),
                           [](const TrackedObject &obj)
                           { return obj.age > 5; }),
            tracked_objects.end());
    }

    // 更新跟踪器状态：检测输入的新目标
    void update(const std::vector<pcl::PointXYZ> &detected_objects)
    {
        std::vector<bool> matched(detected_objects.size(), false);

        // 更新已有目标
        for (auto &tracked_object : tracked_objects)
        {
            int nearest_index = associateObjects(tracked_object.position);

            if (nearest_index != -1)
            {
                tracked_object.position = detected_objects[nearest_index]; // 更新位置
                tracked_object.age = 0;                                    // 重置未匹配的年龄
                matched[nearest_index] = true;
            }
            else
            {
                tracked_object.age++; // 未匹配目标的年龄增加
            }
        }

        // 添加新的目标
        for (size_t i = 0; i < detected_objects.size(); ++i)
        {
            if (!matched[i])
            {
                TrackedObject new_object;
                new_object.position = detected_objects[i];
                new_object.id = id_manager.getNewID(); // 分配新的 ID
                new_object.age = 0;
                tracked_objects.push_back(new_object);
            }
        }

        // 移除长时间未匹配的目标
        tracked_objects.erase(
            std::remove_if(tracked_objects.begin(), tracked_objects.end(),
                           [this](const TrackedObject &obj)
                           {
                               if (obj.age > 5)
                               {                                 // 超过5帧未匹配的目标删除
                                   id_manager.recycleID(obj.id); // 回收 ID
                                   return true;
                               }
                               return false;
                           }),
            tracked_objects.end());
    }

    // 获取当前跟踪的对象
    std::vector<TrackedObject> getTrackedObjects() const
    {
        return tracked_objects;
    }

    // 显示跟踪结果
    void displayTracking(cv::Mat &img)
    {
        for (const auto &obj : tracked_objects)
        {
            cv::Point point(int(obj.position.x * 100), int(obj.position.y * 100));
            cv::circle(img, point, 5, cv::Scalar(0, 255, 0), -1);

            std::string id_text = "ID: " + std::to_string(obj.id);
            cv::putText(img, id_text, point, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
        cv::imshow("Tracking with IDs", img);
        cv::waitKey(1);
    }
};

#endif // MUTIL_OBJECT_ID_TRACKER_HPP
