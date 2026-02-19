#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QShowEvent>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ros_node.hpp"
#include "point_cloud_visualizer.hpp"

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<RosNode> node);

protected:
  void showEvent(QShowEvent *event) override;

private slots:
  void onToggleSensorManager();
  void onToggleSensor();

private:
  void setupUi();
  void processCloud(const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::shared_ptr<RosNode> ros_node_;

  QPushButton* btn_sensor_manager_;
  std::map<SensorType, QPushButton*> sensor_buttons_;

  PointCloudVisualizer* visualizer_;
};
