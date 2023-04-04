# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3".split(';') if "${prefix}/include;/usr/local/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;geometry_msgs;message_runtime;image_transport;cv_bridge;dynamic_reconfigure;pcl_ros;pcl_conversions;pcl_msgs;eigen_conversions;camera_info_manager;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfile_player".split(';') if "-lfile_player" != "" else []
PROJECT_NAME = "file_player"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
