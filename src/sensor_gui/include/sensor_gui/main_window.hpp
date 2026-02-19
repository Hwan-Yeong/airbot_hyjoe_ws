#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QShowEvent>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include "ros_node.hpp"

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
  void initRviz();

  std::shared_ptr<RosNode> ros_node_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros_node_abstraction_;

  QPushButton* btn_sensor_manager_;
  std::map<SensorType, QPushButton*> sensor_buttons_;

  rviz_common::RenderPanel* render_panel_;
  rviz_common::VisualizationManager* manager_;
};
