# old version
    config中以pointcloud结尾的路径是输入数据的路径，现在是需要csv、pcd、图片数据；
    以annotationed结尾的是输出路径，保存标注后的csv文件
    file_index表示从第一帧开始标注
    图片是作为辅助，激光雷达点云和毫米波点云为主

# version 20241012
1. 修改pcd和注释存储路径`config/params.yaml`
    - 数据集：图像后缀`.jpg`，激光雷达点云后缀`.bin`，毫米波点云后缀`.pcd`，所有文件编号从`0`开始
2. 运行radar标注工具
    ```bash
    ros2 launch radar_image_annotation run.launch.py 
    ```
3. 出现三个窗口，若图像窗口没出现，选中毫米波窗口按Shift
4. 【Attention】先在毫米波窗口按住`Shift+左键`随便点击一个毫米波点，然后再通过`d`键或`s`键切换
    - `d`键下一帧，`s`键上一帧

# version 20241015
- 根据激光雷达点云实现自动标注
    + 先转换到统一坐标系下，通过距离阈值判断激光雷达聚类质心和毫米波点位置，实现自动标注
    + 对于没有标注数据的帧，默认开启自动标注；对于已经标注过的帧关闭自动标注，防止重复标注

# version 20241107
- 测试发现自动标注功能在攀钢场景下误检较为严重，因此注释了自动标注功能
- 改进了选点方式，减少了误选
- 将毫米波坐标系旋转到激光雷达下，方便对比

# version 20241108
- 改进自动标注算法
    - 通过点云聚类体积筛选人
    - 通过点云聚类高度滤除其他元素

# version 20241111
- 解决bug
    - 改进窗口刷新，实现了两个窗口同步
    - 在all_clicked_points_和filter_points_添加了目标id
    - 添加CHECK_SQ(x,y,a,b)宏
    - 修改checkPickPoint，补充添加点筛选规则
    - 添加lidar2radar函数
    - 解决了dist_lat,dist_long位置翻转的问题
    <!-- - 修改了csv输入输出格式，`" "-> ","` -->

# version 20241113
- 添加数据预处理节点，当前可以直接读取rosbag，生成数据集
- 修改节点名称
- 更新运行步骤：
    1. 运行数据预处理工具
        ```bash
        ros2 run radar_annotation radar_annotation_data_preprocess <input_file_dir> <output_dir>
        ```
    2. 运行radar标注工具
        ```bash
        ros2 launch radar_annotation run.launch.py 
        ```

# version 20241120
- 增加多目标跟踪功能

# version 20241226
- 更新为解耦ROS的版本
- 在radar_image_annotation/目录下新建build文件夹

- mkdir build 
- cd build 
- cmake ..
- make 

- 运行程序：  ./radar_annotation_annotation

# version 20250425
- 解决标注前和标注后数据信息不一致的bug

# version 20260114
- 上传ROS2版本的标注代码
- 处理ros的bag包，转换为单帧数据文件：修改/src/data_preprocess.cpp文件中74行左右的数据话题名称：
-           攀枝花的bag包：topics_of_interest_ = {"/back/radar_objects","/iv_points_2","/hkcam2/image"};  // PanZhiHua
-           旅顺的bag包：topics_of_interest_ = {"/front/radar_objects","/ls128_left/lslidar_point_cloud","/image/oak_front"};  // LvShun
- 执行ros2 run radar_annotation radar_annotation_data_preprocess <input_file_dir> <output_dir><br/>
- 使用旅顺数据集进行毫米波雷达点云投影到图像（ROS2版本）：修改config/params.yaml文件中的pcd_folder、file_name_annotationed变量为本地文件夹路径；修改/include/radar_tran_image.hpp文件中95行左右的代码，交换long和lat；