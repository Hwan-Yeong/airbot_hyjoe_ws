#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QShowEvent>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QTableWidget>
#include <QKeyEvent>
#include <QSet>
#include <QTimer>
#include "ros_node.hpp"
#include "point_cloud_visualizer.hpp"
#include "teleop_window.hpp"

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<RosNode> node);

protected:
  void showEvent(QShowEvent *event) override;

private slots:
  void onToggleSensorManager();
  void onToggleSensor();
  void onParamChanged();
  void syncTFs();
  void onAddWall();
  void onDeleteWall();
  void onWallTableChanged(int row, int col);
  void onOpenTeleop();

private:
  void setupUi();
  void processCloud(const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::shared_ptr<RosNode> ros_node_;

  QPushButton* btn_sensor_manager_;
  std::map<SensorType, QPushButton*> sensor_buttons_;

  PointCloudVisualizer* visualizer_;

  QDoubleSpinBox* spin_tof_dist_;
  QDoubleSpinBox* spin_cam_dist_;
  QDoubleSpinBox* spin_cam_width_;
  QDoubleSpinBox* spin_cam_height_;
  QDoubleSpinBox* spin_tf_scale_;
  QDoubleSpinBox* spin_robot_x_;
  QDoubleSpinBox* spin_robot_y_;
  QDoubleSpinBox* spin_robot_z_;
  QDoubleSpinBox* spin_robot_yaw_;
  QDoubleSpinBox* spin_footprint_radius_;
  
  // Wall Manager
  QTableWidget* table_walls_;
  QPushButton* btn_add_wall_;
  QPushButton* btn_delete_wall_;
  
  // Simulation Environment
  QCheckBox* check_ground_clip_;
  QCheckBox* check_wall_sim_;
  QDoubleSpinBox* spin_wall_x_;

  QTimer* tf_timer_;
  TeleopWindow* teleop_window_ = nullptr;
};
