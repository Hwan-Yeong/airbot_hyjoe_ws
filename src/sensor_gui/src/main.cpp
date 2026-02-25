#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "sensor_gui/window/main_window.hpp"

int main(int argc, char** argv) {
  printf("[DEBUG] main() started\n"); fflush(stdout);
  // rclcpp::init MUST be called before any ROS2 objects are created
  rclcpp::init(argc, argv);
  printf("[DEBUG] rclcpp::init() finished\n"); fflush(stdout);

  QApplication app(argc, argv);
  printf("[DEBUG] QApplication created\n"); fflush(stdout);

  // Set style for a more modern look
  app.setStyle("Fusion");

  printf("[DEBUG] Creating RosNode...\n"); fflush(stdout);
  auto ros_node = std::make_shared<RosNode>();
  printf("[DEBUG] RosNode created\n"); fflush(stdout);

  // Spin ROS2 in a separate thread
  printf("[DEBUG] Starting ROS spin thread...\n"); fflush(stdout);
  std::thread ros_thread([ros_node]() {
    printf("[DEBUG] ROS spin thread running\n"); fflush(stdout);
    rclcpp::spin(ros_node);
    printf("[DEBUG] ROS spin thread exiting\n"); fflush(stdout);
  });

  printf("[DEBUG] Creating MainWindow...\n"); fflush(stdout);
  MainWindow window(ros_node);
  printf("[DEBUG] MainWindow created\n"); fflush(stdout);
  window.show();
  printf("[DEBUG] window.show() called\n"); fflush(stdout);

  printf("[DEBUG] Entering app.exec()...\n"); fflush(stdout);
  int result = app.exec();
  printf("[DEBUG] app.exec() finished with result %d\n", result); fflush(stdout);

  // Cleanup
  printf("[DEBUG] Shutting down rclcpp...\n"); fflush(stdout);
  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    printf("[DEBUG] Joining ROS spin thread...\n"); fflush(stdout);
    ros_thread.join();
  }

  printf("[DEBUG] main() exiting\n"); fflush(stdout);
  return result;
}
