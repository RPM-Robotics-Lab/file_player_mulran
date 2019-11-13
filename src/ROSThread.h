#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>


#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
// #include <irp_sen_msgs/imu.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
// #include <file_player/dynamic_file_playerConfig.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "file_player/color.h"
#include "rosbag/bag.h"
#include <ros/transport_hints.h>
#include "file_player/datathread.h"
#include <sys/types.h>

#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();
    void ros_initialize(ros::NodeHandle &n);
    void run();
    QMutex *mutex_;
    ros::NodeHandle nh_;
    ros::NodeHandle left_camera_nh_;
    ros::NodeHandle right_camera_nh_;

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

    void Ready();
    void ResetProcessStamp(int position);

signals:
    void StampShow(quint64 stamp);
    void StartSignal();

private:

    int search_bound_;

    bool radarpolar_active_;
    bool imu_active_;

    ros::Subscriber start_sub_;
    ros::Subscriber stop_sub_;

    // ros::Publisher imu_origin_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher magnet_pub_;
    ros::Publisher ouster_pub_;
    ros::Publisher radarpolar_pub_;

    multimap<int64_t, string>                    data_stamp_;
    // map<int64_t, irp_sen_msgs::imu>         imu_data_origin_;
    map<int64_t, sensor_msgs::Imu>         imu_data_;
    map<int64_t, sensor_msgs::MagneticField>         mag_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> imu_thread_;
    DataThread<int64_t> radarpolar_thread_; 
    DataThread<int64_t> ouster_thread_;

    map<int64_t, int64_t> stop_period_; //start and stop stamp

    void DataStampThread();
    void ImuThread();
    void OusterThread(); // giseop
    void RadarpolarThread(); 

    void FilePlayerStart(const std_msgs::BoolConstPtr& msg);
    void FilePlayerStop(const std_msgs::BoolConstPtr& msg);

    vector<string> ouster_file_list_;
    vector<string> radarpolar_file_list_;

    ros::Timer timer_;
    void TimerCallback(const ros::TimerEvent&);
    int64_t processed_stamp_;
    int64_t pre_timer_stamp_;

    bool reset_process_stamp_flag_;

    pair<string,sensor_msgs::PointCloud2> ouster_next_;
    pair<string,cv::Mat> radarpolar_next_; // giseop     

    int GetDirList(string dir, vector<string> &files);

public slots:

};

#endif // VIEWER_ROS_H
