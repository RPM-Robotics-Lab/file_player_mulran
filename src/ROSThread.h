#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>

#include <QObject>
#include <QThread>
#include <QMutex>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "file_player/color.h"
#include "file_player/datathread.h"

using namespace std;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();
    void ros_initialize(rclcpp::Node::SharedPtr node);
    void run();
    void Shutdown();
    QMutex *mutex_;
    rclcpp::Node::SharedPtr node_;

    int64_t initial_data_stamp_;
    int64_t last_data_stamp_;

    bool auto_start_flag_;
    int stamp_show_count_;

    bool play_flag_;
    bool pause_flag_;
    bool loop_flag_;
    bool stop_skip_flag_;
    double play_rate_;
    string data_folder_path_;

    int imu_data_version_;

    void SaveRosbag();
    void Ready();
    void ResetProcessStamp(int position);

signals:
    void StampShow(quint64 stamp);
    void StartSignal();

private:

    int search_bound_;

    bool radarpolar_active_;
    bool imu_active_;

    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnet_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ouster_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr radarpolar_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    int64_t prev_clock_stamp_;

    multimap<int64_t, string>                         data_stamp_;
    map<int64_t, sensor_msgs::msg::NavSatFix>         gps_data_;
    map<int64_t, sensor_msgs::msg::Imu>               imu_data_;
    map<int64_t, sensor_msgs::msg::MagneticField>     mag_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> gps_thread_;
    DataThread<int64_t> imu_thread_;
    DataThread<int64_t> radarpolar_thread_;
    DataThread<int64_t> ouster_thread_;

    map<int64_t, int64_t> stop_period_; //start and stop stamp

    void DataStampThread();
    void GpsThread();
    void ImuThread();
    void OusterThread(); // giseop
    void RadarpolarThread();

    void FilePlayerStart(const std_msgs::msg::Bool::SharedPtr msg);
    void FilePlayerStop(const std_msgs::msg::Bool::SharedPtr msg);

    std::vector<string> ouster_file_list_;
    std::vector<string> radarpolar_file_list_;

    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback();
    int64_t processed_stamp_;
    int64_t pre_timer_stamp_;

    bool reset_process_stamp_flag_;

    pair<string,sensor_msgs::msg::PointCloud2> ouster_next_;
    pair<string,cv::Mat> radarpolar_next_; // giseop

    int GetDirList(string dir, vector<string> &files);

public slots:

};

#endif // VIEWER_ROS_H
