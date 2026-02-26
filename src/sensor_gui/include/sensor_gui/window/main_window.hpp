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
#include <QSplitter>
#include <QScrollArea>
#include <QComboBox>
#include "sensor_gui/node/ros_node.hpp"
#include "sensor_gui/visualizer/point_cloud_visualizer.hpp"
#include "sensor_gui/window/teleop_window.hpp"
#include "sensor_gui/window/obstacle_window.hpp"
#include "sensor_gui/window/robot_model_window.hpp"
#include "sensor_gui/physics/physics_world.hpp"
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<RosNode> node);
  ~MainWindow();

protected:
  void showEvent(QShowEvent *event) override;

private slots:
  void onToggleSensorManager();
  void onToggleSensor();
  void onParamChanged();
  void onToggleBump(bool checked);
  void onPickBackgroundColor();
  void onToggleTheme();
  void syncTFs();
  void onOpenTeleop();
  void onOpenObstacleWindow();
  void onOpenRobotModelEditor();
  void onToggleSidebar();
  void stepPhysics();

private:
  void initConnections();
  void processCloud(const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::unique_ptr<Ui::MainWindow> ui;
  std::shared_ptr<RosNode> ros_node_;

  std::map<SensorType, QPushButton*> sensor_buttons_;

  QTimer* tf_timer_;
  TeleopWindow* teleop_window_ = nullptr;
  ObstacleWindow* obstacle_window_ = nullptr;
  RobotModelWindow* robot_model_window_ = nullptr;
  
  std::shared_ptr<PhysicsWorld> physics_world_;
  QTimer* physics_timer_ = nullptr;
  
  bool is_dark_mode_ = true;
};
