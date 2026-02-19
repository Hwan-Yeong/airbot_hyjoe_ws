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
  create_param_row(robot_layout, "Robot Yaw:", 0.0, -180.0, 180.0, &spin_robot_yaw_);
  create_param_row(robot_layout, "Footprint R:", 0.19, 0.05, 2.0, &spin_footprint_radius_);

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
    if (name == "ToF Mono") color = Qt::yellow;
    else if (name.find("ToF Multi L") != std::string::npos) color = Qt::cyan;
    else if (name.find("ToF Multi R") != std::string::npos) color = Qt::magenta;
    else if (name.find("Camera") != std::string::npos) color = Qt::green;
    else if (name.find("Bottom IR") != std::string::npos) color = Qt::red;
    else if (name.find("Collision") != std::string::npos) color = QColor(255, 165, 0); // Orange

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
  ros_node_->setRobotPose(spin_robot_x_->value(), spin_robot_y_->value(), spin_robot_yaw_->value());
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
