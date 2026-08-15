cmake_minimum_required(VERSION 3.8)
project(file_player)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

# Qt must be found first: PCL/VTK pull Qt5 in transitively, and once its imported
# targets exist the Qt5 config short-circuits and never defines Qt5Widgets_UIC_EXECUTABLE
# / Qt5Core_RCC_EXECUTABLE, which silently breaks qt5_wrap_ui() and qt5_add_resources().
find_package(Qt5 REQUIRED COMPONENTS Widgets Gui OpenGL)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (File_Player_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set (File_Player_QTLib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/ROSThread.h)
set (File_Player_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (File_Player_QTBin_src ${SRC_DIR}/main.cpp)

qt5_wrap_ui (File_Player_QTLib_ui_moc  ${File_Player_QTLib_ui})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

###########
## Build ##
###########
add_executable(file_player
  ${File_Player_QTBin_src}
  ${File_Player_QTLib_src}
  ${File_Player_QTLib_hdr}
  ${File_Player_QTLib_ui_moc}
  ${SHADER_RSC_ADDED}
)

target_include_directories(file_player PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SRC_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(file_player PRIVATE ${PCL_LIBRARY_DIRS})

ament_target_dependencies(file_player
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rosgraph_msgs
  cv_bridge
  pcl_conversions
  rosbag2_cpp
  tf2
  tf2_eigen
)

target_link_libraries(file_player
  Qt5::Widgets
  Qt5::Gui
  Qt5::OpenGL
  ${OPENGL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############
install(TARGETS file_player
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
