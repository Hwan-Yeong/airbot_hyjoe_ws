#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_gui/main_window.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <QApplication>
#include <QPushButton>
#include <QCheckBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QSizePolicy>
#include <functional>
#include <tf2/utils.h>

MainWindow::MainWindow(std::shared_ptr<RosNode> node)
    : ros_node_(node), visualizer_(nullptr)
{
  setupUi();
  ros_node_->setCloudCallback([this](const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      QMetaObject::invokeMethod(this, [this, name, msg]() {
          processCloud(name, msg);
      }, Qt::QueuedConnection);
  });

  // TF Sync Timer
  tf_timer_ = new QTimer(this);
  connect(tf_timer_, &QTimer::timeout, this, &MainWindow::syncTFs);
  tf_timer_->start(100); // 10Hz
}

void MainWindow::showEvent(QShowEvent *event) {
  QMainWindow::showEvent(event);
}

void MainWindow::setupUi() {
  QWidget* central = new QWidget();
  QHBoxLayout* main_layout = new QHBoxLayout(central);

  // Sidebar
  QWidget* sidebar = new QWidget();
  sidebar->setFixedWidth(250);
  QVBoxLayout* sidebar_layout = new QVBoxLayout(sidebar);

  QLabel* title = new QLabel("Sensor Simulator");
  title->setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 20px;");
  sidebar_layout->addWidget(title);

  // Sensor Manager Control
  btn_sensor_manager_ = new QPushButton("Sensor Manager: OFF");
  btn_sensor_manager_->setCheckable(true);
  btn_sensor_manager_->setStyleSheet("height: 40px; background-color: #f44336; color: white; border-radius: 5px;");
  connect(btn_sensor_manager_, &QPushButton::clicked, this, &MainWindow::onToggleSensorManager);
  sidebar_layout->addWidget(btn_sensor_manager_);

  sidebar_layout->addSpacing(10);

  // Individual Sensors
  QGroupBox* sensor_group = new QGroupBox("Virtual Sensors");
  QVBoxLayout* group_layout = new QVBoxLayout();

  auto create_sensor_btn = [&](SensorType type, const QString& name) {
    QPushButton* btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setProperty("sensor_type", static_cast<int>(type));
    btn->setStyleSheet("height: 25px; margin-bottom: 2px;");
    connect(btn, &QPushButton::clicked, this, &MainWindow::onToggleSensor);
    sensor_buttons_[type] = btn;
    group_layout->addWidget(btn);
  };

  create_sensor_btn(SensorType::kTofMono, "ToF Mono");
  create_sensor_btn(SensorType::kTofMultiLeft, "ToF Multi Left");
  create_sensor_btn(SensorType::kTofMultiRight, "ToF Multi Right");
  create_sensor_btn(SensorType::kCamera, "Camera Object");
  create_sensor_btn(SensorType::kBottomIr, "Bottom IR");
  create_sensor_btn(SensorType::kCollisionFront, "Collision Front");
  create_sensor_btn(SensorType::kCollisionRear, "Collision Rear");

  sensor_group->setLayout(group_layout);
  sidebar_layout->addWidget(sensor_group);

  sidebar_layout->addSpacing(10);

  // Simulation Parameters
  QGroupBox* param_group = new QGroupBox("Simulation Params");
  QVBoxLayout* param_layout = new QVBoxLayout();

  auto create_param_row = [&](QLayout* layout, const QString& label, double val, double min, double max, QDoubleSpinBox** spin) {
    QHBoxLayout* row = new QHBoxLayout();
    row->addWidget(new QLabel(label));
    *spin = new QDoubleSpinBox();
    (*spin)->setRange(min, max);
    (*spin)->setValue(val);
    (*spin)->setSingleStep(0.1);
    (*spin)->setDecimals(2);
    connect(*spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onParamChanged);
    row->addWidget(*spin);
    static_cast<QVBoxLayout*>(layout)->addLayout(row);
  };

  create_param_row(param_layout, "ToF Dist:", 0.5, 0.05, 5.0, &spin_tof_dist_);
  create_param_row(param_layout, "Cam Dist:", 0.3, 0.1, 3.0, &spin_cam_dist_);
  create_param_row(param_layout, "Cam Width:", 0.4, 0.1, 2.0, &spin_cam_width_);
  create_param_row(param_layout, "Cam Height:", 0.2, 0.1, 2.0, &spin_cam_height_);
  create_param_row(param_layout, "TF Scale:", 1.0, 0.1, 10.0, &spin_tf_scale_);

  param_group->setLayout(param_layout);
  sidebar_layout->addWidget(param_group);

  // Robot Control
  QGroupBox* robot_group = new QGroupBox("Robot Control");
  QVBoxLayout* robot_layout = new QVBoxLayout();
  
  create_param_row(robot_layout, "Robot X:", 0.0, -10.0, 10.0, &spin_robot_x_);
  create_param_row(robot_layout, "Robot Y:", 0.0, -10.0, 10.0, &spin_robot_y_);
  create_param_row(robot_layout, "Robot Z:", 0.05, 0.0, 2.0, &spin_robot_z_); // 0.0267 m
  create_param_row(robot_layout, "Robot Yaw:", 0.0, -180.0, 180.0, &spin_robot_yaw_);
  create_param_row(robot_layout, "Footprint R:", 0.19, 0.05, 2.0, &spin_footprint_radius_);

  QPushButton* btn_open_teleop = new QPushButton("🎮  Open Teleop Window");
  btn_open_teleop->setStyleSheet(
      "height: 32px; background-color: #313244; color: #89b4fa;"
      "border: 1px solid #45475a; border-radius: 6px; font-weight: bold;");
  connect(btn_open_teleop, &QPushButton::clicked, this, &MainWindow::onOpenTeleop);
  robot_layout->addWidget(btn_open_teleop);

  robot_group->setLayout(robot_layout);
  sidebar_layout->addWidget(robot_group);

  // Simulation Environment
  QGroupBox* env_group = new QGroupBox("Simulation Env");
  QVBoxLayout* env_layout = new QVBoxLayout();
  
  check_ground_clip_ = new QCheckBox("Clip to Ground (Z=0)");
  check_ground_clip_->setChecked(false);
  env_layout->addWidget(check_ground_clip_);
  
  check_wall_sim_ = new QCheckBox("Simulate Wall (X)");
  check_wall_sim_->setChecked(false);
  env_layout->addWidget(check_wall_sim_);
  
  create_param_row(env_layout, "Wall X Pos:", 2.0, -5.0, 10.0, &spin_wall_x_);

  env_group->setLayout(env_layout);
  sidebar_layout->addWidget(env_group);

  connect(check_ground_clip_, &QCheckBox::toggled, this, &MainWindow::onParamChanged);
  connect(check_wall_sim_, &QCheckBox::toggled, this, &MainWindow::onParamChanged);

  // Wall Manager
  QGroupBox* wall_group = new QGroupBox("Maze Wall Manager");
  QVBoxLayout* wall_layout = new QVBoxLayout();
  
  table_walls_ = new QTableWidget(0, 6);
  table_walls_->setHorizontalHeaderLabels({"X", "Y", "Z", "SX", "SY", "SZ"});
  table_walls_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table_walls_->setFixedHeight(150);
  wall_layout->addWidget(table_walls_);
  
  QHBoxLayout* wall_btn_layout = new QHBoxLayout();
  btn_add_wall_ = new QPushButton("Add Wall");
  btn_delete_wall_ = new QPushButton("Delete Wall");
  wall_btn_layout->addWidget(btn_add_wall_);
  wall_btn_layout->addWidget(btn_delete_wall_);
  wall_layout->addLayout(wall_btn_layout);
  
  wall_group->setLayout(wall_layout);
  sidebar_layout->addWidget(wall_group);
  
  connect(btn_add_wall_, &QPushButton::clicked, this, &MainWindow::onAddWall);
  connect(btn_delete_wall_, &QPushButton::clicked, this, &MainWindow::onDeleteWall);
  connect(table_walls_, &QTableWidget::cellChanged, this, &MainWindow::onWallTableChanged);

  // Bottom IR Cliff Controls
  QGroupBox* ir_group = new QGroupBox("Bottom IR CLIFF (True/False)");
  QGridLayout* ir_grid = new QGridLayout();
  const char* labels[] = {"FF", "FL", "FR", "BB", "BL", "BR"};
  IrIndex indices[] = {IrIndex::kFF, IrIndex::kFL, IrIndex::kFR, IrIndex::kBB, IrIndex::kBL, IrIndex::kBR};

  for (int i = 0; i < 6; ++i) {
    QCheckBox* cb = new QCheckBox(labels[i]);
    cb->setChecked(true);
    connect(cb, &QCheckBox::toggled, [this, indices, i](bool checked) {
      ros_node_->setIrState(indices[i], checked);
    });
    ir_grid->addWidget(cb, i / 2, i % 2);
  }
  ir_group->setLayout(ir_grid);
  sidebar_layout->addWidget(ir_group);

  sidebar_layout->addStretch();

  // Custom PointCloud Visualizer
  visualizer_ = new PointCloudVisualizer(central);
  visualizer_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  visualizer_->setMinimumSize(800, 600);

  main_layout->addWidget(sidebar);
  main_layout->addWidget(visualizer_, 1);

  setCentralWidget(central);
  resize(1200, 800);
  setWindowTitle("Airbot Sensor Simulator & Custom Cloud Visualizer");
}

/**
 * @brief Process PointCloud2 message and update visualizer
 * @param name Sensor name
 * @param msg PointCloud2 message
 */
void MainWindow::processCloud(const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!visualizer_) return;

    std::vector<float> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        points.push_back(*iter_x);
        points.push_back(*iter_y);
        points.push_back(*iter_z);
    }

    QColor color = Qt::white;
    if      (name.find("ToF Mono") != std::string::npos)    color = Qt::yellow;
    else if (name.find("ToF Multi L") != std::string::npos) color = Qt::cyan;
    else if (name.find("ToF Multi R") != std::string::npos) color = Qt::magenta;
    else if (name.find("Camera") != std::string::npos)      color = Qt::green;
    else if (name.find("Bottom IR") != std::string::npos)   color = Qt::red;
    else if (name.find("Collision") != std::string::npos)   color = QColor(255, 165, 0); // Orange

    visualizer_->updateCloud(name, msg->header.frame_id, points, color);
}

void MainWindow::onToggleSensorManager() {
  bool active = btn_sensor_manager_->isChecked();
  ros_node_->toggleSensorManager(active);
  if (active) {
    btn_sensor_manager_->setText("Sensor Manager: ACTIVE");
    btn_sensor_manager_->setStyleSheet("height: 40px; background-color: #4CAF50; color: white; border-radius: 5px;");
  } else {
    btn_sensor_manager_->setText("Sensor Manager: OFF");
    btn_sensor_manager_->setStyleSheet("height: 40px; background-color: #f44336; color: white; border-radius: 5px;");
  }
}

void MainWindow::onToggleSensor() {
  auto btn = qobject_cast<QPushButton*>(sender());
  if (!btn) return;

  SensorType type = static_cast<SensorType>(btn->property("sensor_type").toInt());
  bool active = btn->isChecked();
  ros_node_->toggleSensor(type, active);

  if (active) {
    btn->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");
  } else {
    btn->setStyleSheet("height: 25px; margin-bottom: 2px;");
  }
}

void MainWindow::onParamChanged() {
  ros_node_->setToFDistance(spin_tof_dist_->value());
  ros_node_->setCameraParams(spin_cam_dist_->value(), spin_cam_width_->value(), spin_cam_height_->value());
  
  // Teleop 창이 없거나 닫혀 있을 때만 UI 에서 직접 제어
  if (!teleop_window_ || !teleop_window_->isVisible()) {
      ros_node_->setRobotPose(spin_robot_x_->value(), spin_robot_y_->value(), spin_robot_yaw_->value());
      ros_node_->setRobotZ(spin_robot_z_->value());
  }

  if (visualizer_) {
      visualizer_->setTfScale(spin_tf_scale_->value());
      visualizer_->setFootprintRadius(spin_footprint_radius_->value());
      visualizer_->setGroundClipping(check_ground_clip_->isChecked());
      visualizer_->setWallSimulation(check_wall_sim_->isChecked());
      visualizer_->setWallPosition(spin_wall_x_->value());
  }
}

void MainWindow::syncTFs() {
  if (!ros_node_ || !visualizer_) return;
  
  // Teleop 창이 열려 있으면 Node 의 현재 위치를 UI 에 반영
  if (teleop_window_ && teleop_window_->isVisible()) {
      spin_robot_x_->blockSignals(true);
      spin_robot_y_->blockSignals(true);
      spin_robot_yaw_->blockSignals(true);
      spin_robot_z_->blockSignals(true);
      spin_robot_x_->setValue(ros_node_->getRobotX());
      spin_robot_y_->setValue(ros_node_->getRobotY());
      spin_robot_yaw_->setValue(ros_node_->getRobotYaw());
      spin_robot_z_->setValue(ros_node_->getRobotZ());
      spin_robot_x_->blockSignals(false);
      spin_robot_y_->blockSignals(false);
      spin_robot_yaw_->blockSignals(false);
      spin_robot_z_->blockSignals(false);
  }

  auto buffer = ros_node_->getTFBuffer();
  if (!buffer) return;

  std::map<std::string, TfData> tfs;
  std::vector<std::string> frames;
  buffer->_getFrameStrings(frames);
  
  for (const auto& frame : frames) {
    if (frame == "map") continue;
    
    // Determine Nickname
    std::string nickname = frame;
    if (frame == "base_link") nickname = "{base_link}";
    else if (frame == "tof_mono_link") nickname = "{1d}";
    else if (frame.find("tof_multi") != std::string::npos) nickname = "{multi}";
    else if (frame == "tof_camera_link") nickname = "{cam}";
    else if (frame == "bottom_ir_link") nickname = "{ir}";
    else if (frame == "collision_front_link") nickname = "{col_f}";
    else if (frame == "collision_rear_link") nickname = "{col_r}";

    try {
      // Get transform relative to map
      auto transform = buffer->lookupTransform("map", frame, tf2::TimePointZero);
      TfData data;
      data.frame_id = frame;
      data.displayName = nickname;
      data.x = transform.transform.translation.x;
      data.y = transform.transform.translation.y;
      data.z = transform.transform.translation.z;
      data.qx = transform.transform.rotation.x;
      data.qy = transform.transform.rotation.y;
      data.qz = transform.transform.rotation.z;
      data.qw = transform.transform.rotation.w;
      tfs[frame] = data;
    } catch (...) {
    }
  }
  visualizer_->updateTFs(tfs);
}

void MainWindow::onAddWall() {
    int row = table_walls_->rowCount();
    table_walls_->insertRow(row);
    table_walls_->blockSignals(true);
    table_walls_->setItem(row, 0, new QTableWidgetItem("1.0")); // X
    table_walls_->setItem(row, 1, new QTableWidgetItem("0.0")); // Y
    table_walls_->setItem(row, 2, new QTableWidgetItem("0.5")); // Z
    table_walls_->setItem(row, 3, new QTableWidgetItem("0.1")); // SX
    table_walls_->setItem(row, 4, new QTableWidgetItem("2.0")); // SY
    table_walls_->setItem(row, 5, new QTableWidgetItem("1.0")); // SZ
    table_walls_->blockSignals(false);
    onWallTableChanged(row, 0);
}

void MainWindow::onDeleteWall() {
    int row = table_walls_->currentRow();
    if (row >= 0) {
        table_walls_->removeRow(row);
        onWallTableChanged(0, 0);
    }
}

void MainWindow::onWallTableChanged(int row, int col) {
    (void)row; (void)col;
    std::vector<BoxObject> walls;
    for (int i = 0; i < table_walls_->rowCount(); ++i) {
        BoxObject box;
        box.x = table_walls_->item(i, 0)->text().toFloat();
        box.y = table_walls_->item(i, 1)->text().toFloat();
        box.z = table_walls_->item(i, 2)->text().toFloat();
        box.sx = table_walls_->item(i, 3)->text().toFloat();
        box.sy = table_walls_->item(i, 4)->text().toFloat();
        box.sz = table_walls_->item(i, 5)->text().toFloat();
        walls.push_back(box);
    }
    if (visualizer_) visualizer_->setWalls(walls);
}

void MainWindow::onOpenTeleop() {
    if (!teleop_window_) {
        teleop_window_ = new TeleopWindow(nullptr); // 독립 창
        teleop_window_->setVelCallback([this](float vx, float vy, float vyaw) {
            if (ros_node_) ros_node_->setVelocities(vx, vy, vyaw);
        });
    }
    teleop_window_->show();
    teleop_window_->raise();
    teleop_window_->activateWindow();
    teleop_window_->setFocus();
}
