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
#include "ros_node.hpp"
#include "point_cloud_visualizer.hpp"
#include "teleop_window.hpp"
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
  void onAddWall();
  void onDeleteWall();
  void onWallTableChanged(int row, int col);
  void onOpenTeleop();
  void onLoadMap();
  void onSaveMap();
  void onObstacleMoved(int index, float x, float y);
  void onObstacleSelected(int index);
  void onToggleSidebar();
  void onTableSelectionChanged();

private:
  void initConnections();
  void processCloud(const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::unique_ptr<Ui::MainWindow> ui;
  std::shared_ptr<RosNode> ros_node_;

  std::map<SensorType, QPushButton*> sensor_buttons_;

  QTimer* tf_timer_;
  TeleopWindow* teleop_window_ = nullptr;
  
  bool is_dark_mode_ = true;
  
  QString last_map_path_;
  QComboBox* createTypeComboBox(ObstacleType type);
  void loadSettings();
  void saveSettings();
};
