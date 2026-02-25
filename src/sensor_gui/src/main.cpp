#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "sensor_gui/window/main_window.hpp"

int main(int argc, char** argv) {
  // rclcpp::init MUST be called before any ROS2 objects are created
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);

  // Set style for a more modern look
  app.setStyle("Fusion");

  auto ros_node = std::make_shared<RosNode>();

  // Spin ROS2 in a separate thread
  std::thread ros_thread([ros_node]() {
    rclcpp::spin(ros_node);
  });

  MainWindow window(ros_node);
  window.show();

  int result = app.exec();

  // Cleanup
  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  return result;
}
