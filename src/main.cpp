#include <memory>

#include <QApplication>
#include <QMetaObject>
#include <rclcpp/rclcpp.hpp>

#include "mainwindow.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  QApplication a(argc, argv);

  // Ctrl+C (or a shutdown from `ros2 launch`) reaches rclcpp first, so hand it
  // over to Qt as well - otherwise the window would outlive the ROS context.
  rclcpp::on_shutdown([]() {
    if(qApp != nullptr){
      QMetaObject::invokeMethod(qApp, [](){ QCoreApplication::quit(); }, Qt::QueuedConnection);
    }
  });

  MainWindow w;
  w.RosInit(std::make_shared<rclcpp::Node>("file_player"));
  w.show();

  const int exit_code = a.exec();

  rclcpp::shutdown();
  return exit_code;
}
