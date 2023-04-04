#include <QMutexLocker>

#include "ROSThread.h"

#include <regex>

using namespace std;

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  int ring;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (uint32_t, t, t) (int, ring, ring)
                                   )


ROSThread::ROSThread(QObject *parent, QMutex *th_mutex)
  :QThread(parent), mutex_(th_mutex)
{
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  stop_skip_flag_ = true;

  radarpolar_active_ = true;
  imu_active_ = true ;// OFF in v1 (11/13/2019 released), giseop

  search_bound_ = 10;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  prev_clock_stamp_ = 0;
}


ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;
  gps_thread_.active_ = false;
  imu_thread_.active_ = false;
  ouster_thread_.active_ = false;
  radarpolar_thread_.active_ = false;


  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  ouster_thread_.cv_.notify_all(); // giseop
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();
  radarpolar_thread_.cv_.notify_all(); // giseop
  if(radarpolar_thread_.thread_.joinable()) radarpolar_thread_.thread_.join();

}


void 
ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_   = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
  ouster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/os1_points", 1000); // giseop
  radarpolar_pub_ = nh_.advertise<sensor_msgs::Image>("/radar/polar", 10); // giseop
}


void 
ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}


void 
ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();

  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  ouster_thread_.active_ = false; // giseop
  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();

  radarpolar_thread_.active_ = false; // giseop
  radarpolar_thread_.cv_.notify_all();
  if(radarpolar_thread_.thread_.joinable()) radarpolar_thread_.thread_.join();

  //check path is right or not
  ifstream f((data_folder_path_+"/data_stamp.csv").c_str());
  if(!f.good()){
    cout << "Please check the file path. The input path is wrong (data_stamp.csv not exist)" << endl;
    return;
  }
  f.close();



  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;

  //data stamp data load
  fp = fopen((data_folder_path_+"/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;


  //Read gps data
  fp = fopen((data_folder_path_+"/gps.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double cov[9];
  sensor_msgs::NavSatFix gps_data;
  gps_data_.clear();
  while( fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8])
         == 13
         )
  {
    gps_data.header.stamp.fromNSec(stamp);
    gps_data.header.frame_id = "gps";
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;
    for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
    gps_data_[stamp] = gps_data;
  }
  cout << "Gps data are loaded" << endl;

  fclose(fp);

  //Read IMU data
  if(imu_active_)
  {
    fp = fopen((data_folder_path_+"/xsens_imu.csv").c_str(),"r");
    double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
    // irp_sen_msgs::imu imu_data_origin;
    sensor_msgs::Imu imu_data;
    sensor_msgs::MagneticField mag_data;
    imu_data_.clear();
    mag_data_.clear();
    // cout << imu_data << endl ;
    while(1)
    {
      int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", \
                          &stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
      if(length != 8 && length != 17)
        break;
      if(length == 8)
      {
        imu_data.header.stamp.fromNSec(stamp);
        imu_data.header.frame_id = "imu";
        imu_data.orientation.x = q_x;
        imu_data.orientation.y = q_y;
        imu_data.orientation.z = q_z;
        imu_data.orientation.w = q_w;

        imu_data_[stamp] = imu_data;
        imu_data_version_ = 1;

        // imu_data_origin.header.stamp.fromNSec(stamp);
        // imu_data_origin.header.frame_id = "imu";
        // imu_data_origin.quaternion_data.x = q_x;
        // imu_data_origin.quaternion_data.y = q_y;
        // imu_data_origin.quaternion_data.z = q_z;
        // imu_data_origin.quaternion_data.w = q_w;
        // imu_data_origin.eular_data.x = x;
        // imu_data_origin.eular_data.y = y;
        // imu_data_origin.eular_data.z = z;
        // imu_data_origin_[stamp] = imu_data_origin;
      }
      else if(length == 17)
      {
        imu_data.header.stamp.fromNSec(stamp);
        imu_data.header.frame_id = "imu";
        imu_data.orientation.x = q_x;
        imu_data.orientation.y = q_y;
        imu_data.orientation.z = q_z;
        imu_data.orientation.w = q_w;
        imu_data.angular_velocity.x = g_x;
        imu_data.angular_velocity.y = g_y;
        imu_data.angular_velocity.z = g_z;
        imu_data.linear_acceleration.x = a_x;
        imu_data.linear_acceleration.y = a_y;
        imu_data.linear_acceleration.z = a_z;

        imu_data.orientation_covariance[0] = 3;
        imu_data.orientation_covariance[4] = 3;
        imu_data.orientation_covariance[8] = 3;
        imu_data.angular_velocity_covariance[0] = 3;
        imu_data.angular_velocity_covariance[4] = 3;
        imu_data.angular_velocity_covariance[8] = 3;
        imu_data.linear_acceleration_covariance[0] = 3;
        imu_data.linear_acceleration_covariance[4] = 3;
        imu_data.linear_acceleration_covariance[8] = 3;

        imu_data_[stamp] = imu_data;

        mag_data.magnetic_field.x = m_x;
        mag_data.magnetic_field.y = m_y;
        mag_data.magnetic_field.z = m_z;
        mag_data_[stamp] = mag_data;
        imu_data_version_ = 2;

        // imu_data_origin.header.stamp.fromNSec(stamp);
        // imu_data_origin.header.frame_id = "imu";
        // imu_data_origin.quaternion_data.x = q_x;
        // imu_data_origin.quaternion_data.y = q_y;
        // imu_data_origin.quaternion_data.z = q_z;
        // imu_data_origin.quaternion_data.w = q_w;
        // imu_data_origin.eular_data.x = x;
        // imu_data_origin.eular_data.y = y;
        // imu_data_origin.eular_data.z = z;
        // imu_data_origin.gyro_data.x = g_x;
        // imu_data_origin.gyro_data.y = g_y;
        // imu_data_origin.gyro_data.z = g_z;
        // imu_data_origin.acceleration_data.x = a_x;
        // imu_data_origin.acceleration_data.y = a_y;
        // imu_data_origin.acceleration_data.z = a_z;
        // imu_data_origin.magneticfield_data.x = m_x;
        // imu_data_origin.magneticfield_data.y = m_y;
        // imu_data_origin.magneticfield_data.z = m_z;
        // imu_data_origin_[stamp] = imu_data_origin;
      }
    }
    cout << "IMU data are loaded" << endl;
    fclose(fp);
  } // read IMU

  ouster_file_list_.clear();
  radarpolar_file_list_.clear();

  GetDirList(data_folder_path_ + "/Ouster", ouster_file_list_);
  GetDirList(data_folder_path_ + "/radar/polar", radarpolar_file_list_);

  data_stamp_thread_.active_ = true;
  gps_thread_.active_ = true;
  imu_thread_.active_ = true;
  ouster_thread_.active_ = true;
  radarpolar_thread_.active_ = true;

  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  gps_thread_.thread_ = std::thread(&ROSThread::GpsThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
  ouster_thread_.thread_ = std::thread(&ROSThread::OusterThread,this);
  radarpolar_thread_.thread_ = std::thread(&ROSThread::RadarpolarThread,this);
}


void 
ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++)
  {
    auto stamp = iter->first;
    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true))
    {
      if(processed_stamp_ == 0)
      {
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true)
    {
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }

    //check whether stop region or not
    if(stamp == stop_region_iter->first)
    {
      if(stop_skip_flag_ == true)
      {
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true)
      {
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false)
      return;

    if(iter->second.compare("imu") == 0 && imu_active_ == true)
    {
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("gps") == 0)
    {
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("ouster") == 0)
    {
      ouster_thread_.push(stamp);
      ouster_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("radar") == 0 && radarpolar_active_ == true)
    {
      radarpolar_thread_.push(stamp);
      radarpolar_thread_.cv_.notify_all();
    }
    stamp_show_count_++;
    if(stamp_show_count_ > 100)
    {
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
      rosgraph_msgs::Clock clock;


      clock.clock.fromNSec(stamp);
      clock_pub_.publish(clock);
      prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1))
    {
      iter = data_stamp_.begin();
      stop_region_iter = stop_period_.begin();
      processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1))
    {
      play_flag_ = false;
      while(!play_flag_)
      {
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
        usleep(10000);
      }
    }


  }
  cout << "Data publish complete" << endl;
}


void ROSThread::GpsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(gps_thread_.mutex_);
    gps_thread_.cv_.wait(ul);
    if(gps_thread_.active_ == false) return;
    ul.unlock();

    while(!gps_thread_.data_queue_.empty()){
      auto data = gps_thread_.pop();
      //process
      if(gps_data_.find(data) != gps_data_.end()){
        gps_pub_.publish(gps_data_[data]);
      }

    }
    if(gps_thread_.active_ == false) return;
  }
}




void 
ROSThread::ImuThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
    imu_thread_.cv_.wait(ul);
    if(imu_thread_.active_ == false) return;
    ul.unlock();

    while(!imu_thread_.data_queue_.empty())
    {
      auto data = imu_thread_.pop();
      //process
      if(imu_data_.find(data) != imu_data_.end())
      {
        imu_pub_.publish(imu_data_[data]);

        // imu_origin_pub_.publish(imu_data_origin_[data]);
        if(imu_data_version_ == 2)
        {
          magnet_pub_.publish(mag_data_[data]); // Warning publisher has not been initialized
        }
      }
    }
    if(imu_thread_.active_ == false) return;
  }
}


void 
ROSThread::TimerCallback(const ros::TimerEvent&)
{
  int64_t current_stamp = ros::Time::now().toNSec();
  if(play_flag_ == true && pause_flag_ == false){
    processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
  }
  pre_timer_stamp_ = current_stamp;

  if(play_flag_ == false){
    processed_stamp_ = 0; //reset
    prev_clock_stamp_ = 0;
  }
}


void 
ROSThread::OusterThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1)
  {
    std::unique_lock<std::mutex> ul(ouster_thread_.mutex_);
    ouster_thread_.cv_.wait(ul);
    if(ouster_thread_.active_ == false)
      return;
    ul.unlock();

    while(!ouster_thread_.data_queue_.empty())
    {
      auto data = ouster_thread_.pop();

      //publish data
      if(to_string(data) + ".bin" == ouster_next_.first)
      {
        //publish
        ouster_next_.second.header.stamp.fromNSec(data);
        ouster_next_.second.header.frame_id = "ouster"; // frame ID
        ouster_pub_.publish(ouster_next_.second);
      }
      else
      {
        //load current data
        pcl::PointCloud<PointXYZIRT> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/Ouster" +"/"+ to_string(data) + ".bin";

        if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") != ouster_file_list_.end())
        {
          ifstream file;
          file.open(current_file_name, ios::in|ios::binary);
          int k = 0;
          while(!file.eof())
          {
            PointXYZIRT point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            point.ring = (k%64) + 1 ;
            k = k+1 ;
            cloud.points.push_back (point);
          }
          file.close();

          pcl::toROSMsg(cloud, publish_cloud);
          publish_cloud.header.stamp.fromNSec(data);
          publish_cloud.header.frame_id = "ouster";
          ouster_pub_.publish(publish_cloud);
        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<PointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") - ouster_file_list_.begin();
      if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),ouster_file_list_[current_file_index+1]) != ouster_file_list_.end()){
        string next_file_name = data_folder_path_ + "/Ouster" +"/"+ ouster_file_list_[current_file_index+1];

        ifstream file;
        file.open(next_file_name, ios::in|ios::binary);
        int k = 0;
        while(!file.eof()){
          PointXYZIRT point;
          file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
          point.ring = (k%64) + 1 ;
          k = k+1 ;
          cloud.points.push_back (point);
        }

        file.close();
        pcl::toROSMsg(cloud, publish_cloud);
        ouster_next_ = make_pair(ouster_file_list_[current_file_index+1], publish_cloud);
      }
      previous_file_index = current_file_index;
    }
    if(ouster_thread_.active_ == false) return;
  }
}


void 
ROSThread::RadarpolarThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(radarpolar_thread_.mutex_);
    radarpolar_thread_.cv_.wait(ul);
    if(radarpolar_thread_.active_ == false)
      return;
    ul.unlock();

    while(!radarpolar_thread_.data_queue_.empty())
    {
      auto data = radarpolar_thread_.pop();
      //process
      if(radarpolar_file_list_.size() == 0) continue;

      //publish
      if( to_string(data)+".png" == radarpolar_next_.first && !radarpolar_next_.second.empty() )
      {
        cv_bridge::CvImage radarpolar_out_msg;
        radarpolar_out_msg.header.stamp.fromNSec(data);
        radarpolar_out_msg.header.frame_id = "radar_polar";
        radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        radarpolar_out_msg.image    = radarpolar_next_.second;
        radarpolar_pub_.publish(radarpolar_out_msg.toImageMsg());
      }
      else
      {
        string current_radarpolar_name = data_folder_path_ + "/radar/polar" + "/" + to_string(data) + ".png";

        cv::Mat radarpolar_image;
        radarpolar_image = imread(current_radarpolar_name, CV_LOAD_IMAGE_GRAYSCALE);
        if(!radarpolar_image.empty())
        {

          cv_bridge::CvImage radarpolar_out_msg;
          radarpolar_out_msg.header.stamp.fromNSec(data);
          radarpolar_out_msg.header.frame_id = "radar_polar";
          radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
          radarpolar_out_msg.image    = radarpolar_image;
          radarpolar_pub_.publish(radarpolar_out_msg.toImageMsg());

        }
        previous_img_index = 0;
      }

      //load next image
      current_img_index = find( next(radarpolar_file_list_.begin(),max(0,previous_img_index - search_bound_)), radarpolar_file_list_.end(), to_string(data)+".png" ) - radarpolar_file_list_.begin();
      if(current_img_index < radarpolar_file_list_.size()-2)
      {
        string next_radarpolar_name = data_folder_path_ + "/radar/polar" +"/"+ radarpolar_file_list_[current_img_index+1];

        cv::Mat radarpolar_image;
        radarpolar_image = imread(next_radarpolar_name, CV_LOAD_IMAGE_COLOR);

        if(!radarpolar_image.empty())
        {
          cv::cvtColor(radarpolar_image, radarpolar_image, cv::COLOR_RGB2BGR);
          radarpolar_next_ = make_pair(radarpolar_file_list_[current_img_index+1], radarpolar_image);
        }
      }
      previous_img_index = current_img_index;
    }
    
    if(radarpolar_thread_.active_ == false) return;
  }
}


int 
ROSThread::GetDirList(string dir, vector<string> &files)
{

  files.clear();
  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
  {
    string errmsg{(string{"No directory ("} + dir + string{")"})};
    const char * ptr_errmsg = errmsg.c_str();
    perror(ptr_errmsg);
    // perror("No directory");
  }
  else
  {
    while (n--)
    {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != "..")
      {
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
    }
    free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++)
  {
    files.push_back(*iter);
  }
  return 0;
}


void 
ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}


void 
ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;

  emit StartSignal();
}


void 
ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }
}



void ROSThread::SaveRosbag()
{
  rosbag::Bag bag;
  const std::string bag_path = data_folder_path_+"/output.bag";
  bag.open(data_folder_path_+"/output.bag", rosbag::bagmode::Write);
  cout<<"Storing bag to: "<<bag_path<<endl;


  // cout<<"Found: "<<radarpolar_file_list_.size()<<" radar sweeps"<<endl;
  // int count = 1;
  // for(auto && file_name : radarpolar_file_list_){

  //   cv::Mat radarpolar_image;
  //   const std::string file_path = data_folder_path_ + "/radar/polar/" + file_name;
  //   cout<<"radar: "<<count++<<"/"<<radarpolar_file_list_.size()<<endl;
  //   //cout<<"load ("<<count++<<"/"<<radarpolar_file_list_.size()<<") from: "<<file_path<<endl;
  //   radarpolar_image = imread(file_path, 0);

  //   size_t lastindex = file_name.find_last_of(".");
  //   std::string stamp_str = file_name.substr(0, lastindex);
  //   int64_t  stamp_int;
  //   std::istringstream ( stamp_str ) >> stamp_int;

  //   cv_bridge::CvImage radarpolar_out_msg;
  //   radarpolar_out_msg.header.stamp.fromNSec(stamp_int);
  //   radarpolar_out_msg.header.frame_id = "radar_polar";
  //   radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
  //   radarpolar_out_msg.image    = radarpolar_image;
  //   auto msg = radarpolar_out_msg.toImageMsg();
  //   bag.write("/Navtech/Polar", msg->header.stamp, *msg);
  // }

  //////////////////// Point Cloud Save Bag ///////////////////////
  int ouster_counter = 1; 
  for (int i = 0; i < ouster_file_list_.size(); i++){
    std::cout << ouster_file_list_.at(i) << std::endl;
  } 
  
  for (auto && file_name : ouster_file_list_){

    std::cout << "File Name: " << file_name << std::endl;
    std::cout << ouster_counter++ << "/" << ouster_file_list_.size() << std::endl;
    ifstream input(data_folder_path_ + "/Ouster/" + file_name,ios::binary);

    if(!input.is_open()){
      std::cerr << "Could not read file." << std::endl;
      break;
    }

    pcl::PointCloud<PointXYZIRT> points;
    const size_t kMaxNumberOfPoints = 1e6;

    
    points.reserve(kMaxNumberOfPoints);

    for ( int i = 0; input.is_open() && !input.eof(); i++){
      PointXYZIRT point;

      input.read((char *)&point.x, sizeof(float));
      input.read((char *)&point.y, sizeof(float));
      input.read((char *)&point.z, sizeof(float));
      input.read((char *)&point.intensity, sizeof(float));

      points.push_back(point);
    }
  input.close();

  size_t lastindex = file_name.find_last_of(".");
  std::string stamp_str = file_name.substr(0, lastindex);
  int64_t  stamp_int;
  std::istringstream ( stamp_str ) >> stamp_int;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(points, output);
  output.header.frame_id = "base_link";
  output.header.stamp.fromNSec(stamp_int);
  bag.write("mulran/velo/pointclouds", output.header.stamp, output);

  }
  ////////////////////// OPEN GPS FILE //////////////////////////
  std::ifstream gps_file(data_folder_path_ + "/gps.csv");

  if(!gps_file.is_open()){
    std::cout << "Could not open file!" << std::endl;
  }

  std::string line;
  std::getline(gps_file,line);
  if(gps_file.is_open()){
  while(std::getline(gps_file, line)){

    std::stringstream ss(line);
    std::string field;
    std::vector<std::string> fields;

    while(std::getline(ss, field, ',')){
      fields.push_back(field);
    }
    
    sensor_msgs::NavSatFix msg;
    int64_t stamp_int_gps;
    std::string stamp_str = fields[0];
    std::istringstream (stamp_str) >> stamp_int_gps;
    msg.header.stamp.fromNSec(stamp_int_gps);
    msg.header.frame_id = "base_link";

    std::istringstream (fields[1]) >> msg.latitude;
    std::istringstream (fields[2]) >> msg.longitude;
    std::istringstream (fields[3]) >> msg.altitude;
    std::istringstream (fields[4]) >> msg.position_covariance[0];
    std::istringstream (fields[5]) >> msg.position_covariance[1];
    std::istringstream (fields[6]) >> msg.position_covariance[2];
    std::istringstream (fields[7]) >> msg.position_covariance[3];
    std::istringstream (fields[8]) >> msg.position_covariance[4];
    std::istringstream (fields[9]) >> msg.position_covariance[5];
    std::istringstream (fields[10]) >> msg.position_covariance[6];
    std::istringstream (fields[11]) >> msg.position_covariance[7];
    std::istringstream (fields[12]) >> msg.position_covariance[8];


    bag.write("mulran/gps/fix", msg.header.stamp, msg);


  }

  gps_file.close();

  }

  /////////////////////// IMU ///////////////////////////////////////


  std::ifstream imuFile(data_folder_path_ + "/xsens_imu.csv");

  if(!imuFile.is_open()){
    std::cout << "File could not open." << std::endl;
  }
  std::string line_imu;
  std::getline(imuFile,line);
  if(imuFile.is_open()){
  while(std::getline(imuFile, line)){

    std::stringstream ss(line);
    std::string field;
    std::vector<std::string> fields;

    while(std::getline(ss, field, ',')){
      fields.push_back(field);
    }
    
    sensor_msgs::Imu msg_imu;
    sensor_msgs::MagneticField msg_mag;
    int64_t stamp_int_imu;
    std::string stamp_str = fields[0];
    std::istringstream (stamp_str) >> stamp_int_imu;
    msg_imu.header.stamp.fromNSec(stamp_int_imu);
    msg_imu.header.frame_id = "base_link";

    std::istringstream (fields[1]) >> msg_imu.orientation.x;
    std::istringstream (fields[2]) >> msg_imu.orientation.y;
    std::istringstream (fields[3]) >> msg_imu.orientation.z;
    std::istringstream (fields[4]) >> msg_imu.orientation.w;

    std::istringstream (fields[8]) >> msg_imu.angular_velocity.x;
    std::istringstream (fields[9]) >> msg_imu.angular_velocity.y;
    std::istringstream (fields[10]) >> msg_imu.angular_velocity.z;

    std::istringstream (fields[11]) >> msg_imu.linear_acceleration.x;
    std::istringstream (fields[12]) >> msg_imu.linear_acceleration.y;
    std::istringstream (fields[13]) >> msg_imu.linear_acceleration.z;

    msg_imu.orientation_covariance[0] = 3;
    msg_imu.orientation_covariance[4] = 3;
    msg_imu.orientation_covariance[8] = 3;
    msg_imu.angular_velocity_covariance[0] = 3;
    msg_imu.angular_velocity_covariance[4] = 3;
    msg_imu.angular_velocity_covariance[8] = 3;
    msg_imu.linear_acceleration_covariance[0] = 3;
    msg_imu.linear_acceleration_covariance[4] = 3;
    msg_imu.linear_acceleration_covariance[8] = 3;

    std::istringstream (fields[14]) >> msg_mag.magnetic_field.x;
    std::istringstream (fields[15]) >> msg_mag.magnetic_field.y;
    std::istringstream (fields[16]) >> msg_mag.magnetic_field.z;

    bag.write("mulran/imu", msg_imu.header.stamp, msg_imu);


  }

  imuFile.close();

  }



  ////////////////////// OPEN GROUND TRUTH FILE /////////////////////

//   const std::string gt_csv_path = data_folder_path_+ std::string("/global_pose.csv");
//   fstream fin;
//   fin.open(gt_csv_path, ios::in);
//   if(fin.is_open()){
//     cout<<"loaded: "<<gt_csv_path<<endl;

//     std::string temp;
//     int count  = 0;
//     nav_msgs::Odometry Tgt_msg;
//     Tgt_msg.header.frame_id = "world";
//     while (fin >> temp) {
//       Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Zero();
//       T(3,3) = 1.0;

//       std::vector<string> row;

//       stringstream  ss(temp);
//       std::string str;
//       while (getline(ss, str, ','))
//         row.push_back(str);
//       if(row.size()!=13)
//         break;
//       int64_t stamp_int;

//       std::istringstream ( row[0] ) >> stamp_int;

      
//       for(int i=0;i<3;i++){
//         for(int j=0;j<4;j++){
//           double d = boost::lexical_cast<double> (row[1+(4*i)+j]);
//           T(i,j) = d;
//         }
//       }

//       Eigen::Affine3d Tgt(T);
//       //std::cout<<Tgt.matrix()<<std::endl;
//       tf::poseEigenToMsg(Tgt,Tgt_msg.pose.pose);
//       Tgt_msg.header.stamp.fromNSec(stamp_int);
//       bag.write("/gt", Tgt_msg.header.stamp, Tgt_msg);
//       //cout<<"Written: "<<count++<<" /gt nav_msgs/odometry poses to /gt"<<endl;
//       count++;
//     }
//   }
  cout<<"rosbag stored at: "<<bag_path<<endl;
  bag.close();
}
