#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_gui/window/main_window.hpp"
#include "ui_main_window.h"
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
#include <QColorDialog>
#include <QScrollArea>
#include <QFileDialog>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QSplitter>
#include <functional>
#include <tf2/utils.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_gui/visualizer/robot_model.hpp"
#include <QComboBox>

MainWindow::MainWindow(std::shared_ptr<RosNode> node)
    : QMainWindow(nullptr), ui(new Ui::MainWindow), ros_node_(node)
{
  ui->setupUi(this);

  ros_node_->setUsePhysics(true);
  physics_world_ = std::make_shared<PhysicsWorld>();
  physics_world_->init();
  // URDF loading in initConnections requires physics_world_ to be initialized

  initConnections();
  ros_node_->setCloudCallback([this](const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      QMetaObject::invokeMethod(this, [this, name, msg]() {
          processCloud(name, msg);
      }, Qt::QueuedConnection);
  });

  physics_timer_ = new QTimer(this);
  connect(physics_timer_, &QTimer::timeout, this, &MainWindow::stepPhysics);
  physics_timer_->start(16); // ~60Hz

  // TF Sync Timer
  tf_timer_ = new QTimer(this);
  connect(tf_timer_, &QTimer::timeout, this, &MainWindow::syncTFs);
  tf_timer_->start(100); // 10Hz
}

MainWindow::~MainWindow() {
    if (teleop_window_) delete teleop_window_;
    if (obstacle_window_) delete obstacle_window_;
}

void MainWindow::showEvent(QShowEvent *event) {
  QMainWindow::showEvent(event);
}

void MainWindow::initConnections() {
  sensor_buttons_[SensorType::kTofMono] = ui->btn_sensor_tof_mono_;
  sensor_buttons_[SensorType::kTofMultiLeft] = ui->btn_sensor_tof_left_;
  sensor_buttons_[SensorType::kTofMultiRight] = ui->btn_sensor_tof_right_;
  sensor_buttons_[SensorType::kCamera] = ui->btn_sensor_cam_;
  sensor_buttons_[SensorType::kBottomIr] = ui->btn_sensor_bottom_ir_;
  sensor_buttons_[SensorType::kCollisionFront] = ui->btn_sensor_coll_front_;
  sensor_buttons_[SensorType::kCollisionRear] = ui->btn_sensor_coll_rear_;

  connect(ui->btn_sidebar_toggle_, &QPushButton::clicked, this, &MainWindow::onToggleSidebar);
  connect(ui->btn_theme_toggle_, &QPushButton::clicked, this, &MainWindow::onToggleTheme);
  connect(ui->btn_sensor_manager_, &QPushButton::clicked, this, &MainWindow::onToggleSensorManager);

  for (auto& pair : sensor_buttons_) {
      pair.second->setProperty("sensor_type", static_cast<int>(pair.first));
      connect(pair.second, &QPushButton::clicked, this, &MainWindow::onToggleSensor);
  }

  // Set default active buttons style for ToF sensors
  sensor_buttons_[SensorType::kTofMono]->setChecked(true);
  sensor_buttons_[SensorType::kTofMono]->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");
  sensor_buttons_[SensorType::kTofMultiLeft]->setChecked(true);
  sensor_buttons_[SensorType::kTofMultiLeft]->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");
  sensor_buttons_[SensorType::kTofMultiRight]->setChecked(true);
  sensor_buttons_[SensorType::kTofMultiRight]->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");

  auto connect_spin = [this](QDoubleSpinBox* spin) {
      connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onParamChanged);
  };
  connect_spin(ui->spin_tof_mono_dist_);
  connect_spin(ui->spin_tof_left_dist_);
  connect_spin(ui->spin_tof_right_dist_);
  connect_spin(ui->spin_cam_dist_);
  connect_spin(ui->spin_cam_width_);
  connect_spin(ui->spin_cam_height_);
  connect_spin(ui->spin_tf_scale_);
  connect_spin(ui->spin_footprint_radius_);
  connect_spin(ui->spin_wall_x_);

  connect(ui->btn_open_teleop_, &QPushButton::clicked, this, &MainWindow::onOpenTeleop);

  connect(ui->check_ground_clip_, &QCheckBox::toggled, this, &MainWindow::onParamChanged);
  connect(ui->check_wall_sim_, &QCheckBox::toggled, this, &MainWindow::onParamChanged);
  connect(ui->check_bump_sim_, &QCheckBox::toggled, this, &MainWindow::onToggleBump);
  connect(ui->btn_pick_bg_, &QPushButton::clicked, this, &MainWindow::onPickBackgroundColor);
  connect(ui->btn_top_view_, &QPushButton::clicked, this, [this]() {
      if (ui->visualizer_) ui->visualizer_->setTopView();
  });
  connect(ui->btn_side_view_, &QPushButton::clicked, this, [this]() {
      if (ui->visualizer_) ui->visualizer_->setSideView();
  });

  connect(ui->btn_edit_wall, &QPushButton::clicked, this, &MainWindow::onOpenObstacleWindow);
  connect(ui->btn_edit_robot_model, &QPushButton::clicked, this, &MainWindow::onOpenRobotModelEditor);

  // Dynamic Bottom IR Cliff Controls
  QGroupBox* ir_group = new QGroupBox("Bottom IR CLIFF (True/False)");
  QGridLayout* ir_grid = new QGridLayout();
  const char* labels[] = {"FF", "FL", "FR", "BB", "BL", "BR"};
  IrIndex indices[] = {IrIndex::kFF, IrIndex::kFL, IrIndex::kFR, IrIndex::kBB, IrIndex::kBL, IrIndex::kBR};

  for (int i = 0; i < 6; ++i) {
    IrIndex idx = indices[i];
    QCheckBox* cb = new QCheckBox(labels[i]);
    cb->setChecked(true);
    connect(cb, &QCheckBox::toggled, [this, idx](bool checked) {
      ros_node_->setIrState(idx, checked);
    });
    ir_grid->addWidget(cb, i / 2, i % 2);
  }
  ir_group->setLayout(ir_grid);
  
  if (QVBoxLayout* vbox = qobject_cast<QVBoxLayout*>(ui->scrollAreaWidgetContents->layout())) {
      vbox->insertWidget(vbox->count() - 1, ir_group);
  }

  // Obstacle Window logic moved to its own class

  ui->main_splitter_->setStretchFactor(1, 1);

  // Force initial state to Dark Mode
  is_dark_mode_ = false;
  onToggleTheme(); 
  onParamChanged();

  // Load URDF from package share directory
  try {
      std::string share_dir = ament_index_cpp::get_package_share_directory("sensor_gui");
      std::string urdf_path = share_dir + "/urdf/robot_urdf.xml";
      if (ui->visualizer_) {
          if (!ui->visualizer_->setRobotModelFromUrdf(urdf_path)) {
              std::cerr << "Failed to load URDF from: " << urdf_path << std::endl;
          } else {
              std::cout << "Successfully loaded URDF: " << urdf_path << std::endl;
              // Pass the URDF parameters to physics
              if (physics_world_ && ui->visualizer_->getRobotModel()) {
                  float wheelbase = ui->visualizer_->getRobotModel()->getWheelbase();
                  float radius = ui->visualizer_->getRobotModel()->getWheelRadius();
                  float robot_mass = ros_node_ ? ros_node_->getRobotMass() : 10.0f;
                  std::cout << "[DEBUG] Initializing Physics Robot: Radius=" << radius 
                            << ", Wheelbase=" << wheelbase << ", Mass=" << robot_mass << std::endl;
                  physics_world_->initRobot(radius, wheelbase, robot_mass);
              }
          }
      }
  } catch (...) {
      std::cerr << "Could not find package share directory for sensor_gui" << std::endl;
  }
}


/**
 * @brief Process PointCloud2 message and update visualizer
 * @param name Sensor name
 * @param msg PointCloud2 message
 */
void MainWindow::processCloud(const std::string& name, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!ui->visualizer_) return;

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

    ui->visualizer_->updateCloud(name, msg->header.frame_id, points, color);
}

void MainWindow::onToggleSensorManager() {
  bool active = ui->btn_sensor_manager_->isChecked();
  ros_node_->toggleSensorManager(active);
  if (active) {
    ui->btn_sensor_manager_->setText("Sensor Manager: ACTIVE");
    ui->btn_sensor_manager_->setStyleSheet("height: 40px; background-color: #4CAF50; color: white; border-radius: 5px;");
  } else {
    ui->btn_sensor_manager_->setText("Sensor Manager: OFF");
    ui->btn_sensor_manager_->setStyleSheet("height: 40px; background-color: #f44336; color: white; border-radius: 5px;");
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
  ros_node_->setTofMonoDist(ui->spin_tof_mono_dist_->value());
  ros_node_->setTofLeftDist(ui->spin_tof_left_dist_->value());
  ros_node_->setTofRightDist(ui->spin_tof_right_dist_->value());
  ros_node_->setCameraParams(ui->spin_cam_dist_->value(), ui->spin_cam_width_->value(), ui->spin_cam_height_->value());
  
  // Note: Robot pose (X/Y/Z/Yaw) spinboxes were removed. 
  // It's now primarily controlled by Teleop window or internal node logic.

  if (ui->visualizer_) {
      ui->visualizer_->setTfScale(ui->spin_tf_scale_->value());
      ui->visualizer_->setFootprintRadius(ui->spin_footprint_radius_->value());
      ui->visualizer_->setGroundClipping(ui->check_ground_clip_->isChecked());
      ui->visualizer_->setWallSimulation(ui->check_wall_sim_->isChecked());
      ui->visualizer_->setWallPosition(ui->spin_wall_x_->value());
  }
}

void MainWindow::syncTFs() {
  if (!ros_node_ || !ui->visualizer_) return;
  
  // Note: Robot pose spinboxes were removed, so we no longer sync UI from Node here.

  auto buffer = ros_node_->getTFBuffer();
  if (!buffer) return;

  std::map<std::string, TfData> tfs;
  std::vector<std::string> frames;
  buffer->_getFrameStrings(frames);
  
  for (const auto& frame : frames) {
    if (frame == "map") continue;
    
    // Determine Nickname
    std::string nickname = frame;
    if (frame == "base_link") {
        if (ros_node_->getUsePhysics()) continue; // Skip sync if physics simulates it directly
        nickname = "{base_link}";
    }
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
  ui->visualizer_->updateTFs(tfs);
}

void MainWindow::stepPhysics() {
    if (!physics_world_ || !ui->visualizer_ || !ros_node_) return;

    // Send velocity command to physics
    float vx = ros_node_->getSmoothedVx();
    float vy = ros_node_->getSmoothedVy();
    float vyaw = ros_node_->getSmoothedVyawRad();
    physics_world_->setRobotVelocity(vx, vy, vyaw);

    // Sync obstacles (read from UI)
    auto obs = ui->visualizer_->getObstacles();
    physics_world_->syncObstacles(obs);
    
    // Step simulation
    physics_world_->stepSimulation(0.016f); // 16ms
    
    // Update visualizer with new object positions
    ui->visualizer_->setObstacles(obs);

    // Update Robot pose in RosNode
    // Physics 'rz' is the center of the chassis, which is at 0.15m height.
    // The visual 'base_link' expects to be around 0.045m (wheel radius).
    // So we subtract the difference: chassis_z_center - wheel_radius
    float rx = 0.0f, ry = 0.0f, rz = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    physics_world_->getRobotPose(rx, ry, rz, roll, pitch, yaw);
    ros_node_->setRobotPose(rx, ry, yaw * 180.0f / M_PI);
    
    float radius = 0.045f;
    if (ui->visualizer_->getRobotModel()) {
        radius = ui->visualizer_->getRobotModel()->getWheelRadius();
    }
    float visual_z = rz - 0.105f;
    ros_node_->setRobotZ(visual_z); 

    // Directly update the visualizer's base_link at 60Hz for smooth rendering (avoids 10Hz TF lag)
    if (ui->visualizer_) {
        TfData base_tf;
        base_tf.frame_id = "base_link";
        base_tf.displayName = "{base_link}";
        base_tf.x = rx; base_tf.y = ry; base_tf.z = visual_z;
        tf2::Quaternion q; q.setRPY(roll, pitch, yaw);
        base_tf.qx = q.x(); base_tf.qy = q.y(); base_tf.qz = q.z(); base_tf.qw = q.w();
        ui->visualizer_->updateSingleTf(base_tf);
    }

    // Sync Wheel Pos to Visualizer
    float lx = 0, ly = 0, lz = 0, lqx = 0, lqy = 0, lqz = 0, lqw = 1;
    float wrx = 0, wry = 0, wrz = 0, wrqx = 0, wrqy = 0, wrqz = 0, wrqw = 1;
    physics_world_->getWheelPoses(lx, ly, lz, lqx, lqy, lqz, lqw, 
                                  wrx, wry, wrz, wrqx, wrqy, wrqz, wrqw);
    
    TfData left_td, right_td;
    left_td.x = lx; left_td.y = ly; left_td.z = lz; left_td.qx = lqx; left_td.qy = lqy; left_td.qz = lqz; left_td.qw = lqw;
    right_td.x = wrx; right_td.y = wry; right_td.z = wrz; right_td.qx = wrqx; right_td.qy = wrqy; right_td.qz = wrqz; right_td.qw = wrqw;
    ui->visualizer_->setWheelPoses(left_td, right_td);
}

void MainWindow::onToggleBump(bool checked) {
    if (!ui->visualizer_) return;
    
    std::vector<SimObstacle> current_walls;
    // Note: Manage walls specifically for this demo
    if (checked) {
        SimObstacle bump;
        bump.name = "bump";
        bump.type = ObstacleType::kBox;
        bump.x = 1.0f; bump.y = 0.0f; bump.z = 0.015f;
        bump.sx = 0.4f; bump.sy = 4.0f; bump.sz = 0.03f;
        current_walls.push_back(bump);
    }
    ui->visualizer_->setObstacles(current_walls);
}

// Obstacle management functions have been moved to ObstacleWindow

void MainWindow::onPickBackgroundColor() {
    if (!ui->visualizer_) return;
    QColor color = QColorDialog::getColor(is_dark_mode_ ? QColor(13, 13, 26) : QColor(242, 242, 230), 
                                         this, "Select 3D View Background Color");
    if (color.isValid()) {
        ui->visualizer_->setBackgroundColor(color);
    }
}

void MainWindow::onToggleTheme() {
    is_dark_mode_ = !is_dark_mode_;
    
    if (is_dark_mode_) {
        ui->btn_theme_toggle_->setText("🌙 Dark Mode");
        centralWidget()->setStyleSheet(R"(
            QWidget { background-color: #1e1e2e; color: #cdd6f4; font-family: 'Segoe UI', sans-serif; }
            QGroupBox { border: 1px solid #45475a; border-radius: 8px; margin-top: 15px; font-weight: bold; background-color: #242438; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #f5c2e7; }
            QPushButton { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 6px; padding: 5px; height: 24px; }
            QPushButton:hover { background-color: #45475a; }
            QPushButton:checked { background-color: #89b4fa; color: #1e1e2e; }
            QDoubleSpinBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 2px; }
            QCheckBox { spacing: 5px; }
            QLabel { color: #cdd6f4; }
        )");
        if (ui->visualizer_) ui->visualizer_->setBackgroundColor(QColor(13, 13, 26));
    } else {
        ui->btn_theme_toggle_->setText("☀️ Light Mode");
        centralWidget()->setStyleSheet(R"(
            QWidget { background-color: #FDFBF7; color: #2C2C2C; font-family: 'Segoe UI', sans-serif; }
            QGroupBox { border: 1px solid #D1D1D1; border-radius: 8px; margin-top: 15px; font-weight: bold; background-color: #FFFFFF; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #4A4A4A; }
            QPushButton { background-color: #FFFFFF; color: #2C2C2C; border: 1px solid #D1D1D1; border-radius: 6px; padding: 5px; height: 24px; }
            QPushButton:hover { background-color: #F0F0F0; }
            QPushButton:checked { background-color: #AED9FF; border-color: #7BB8FF; }
            QDoubleSpinBox { background-color: #FFFFFF; color: #2C2C2C; border: 1px solid #D1D1D1; border-radius: 4px; padding: 2px; }
            QCheckBox { spacing: 5px; }
            QLabel { color: #2C2C2C; }
        )");
        if (ui->visualizer_) ui->visualizer_->setBackgroundColor(QColor(242, 242, 230));
    }

    if (teleop_window_) {
        teleop_window_->setTheme(is_dark_mode_);
    }
}

void MainWindow::onOpenTeleop() {
    if (!teleop_window_) {
        teleop_window_ = new TeleopWindow(nullptr); // 독립 창
        // No callback setting, TeleopWindow uses dedicated /cmd_vel publisher
        teleop_window_->setTheme(is_dark_mode_);
    }
    teleop_window_->show();
    teleop_window_->raise();
    teleop_window_->activateWindow();
    teleop_window_->setFocus();
}
void MainWindow::onToggleSidebar() {
    if (!ui->sidebar_scroll_) return;
    
    bool is_visible = ui->sidebar_scroll_->isVisible();
    ui->sidebar_scroll_->setVisible(!is_visible);
    
    if (is_visible) {
        ui->btn_sidebar_toggle_->setText("▶ Show Sidebar");
    } else {
        ui->btn_sidebar_toggle_->setText("◀ Hide Sidebar");
    }
}

void MainWindow::onOpenRobotModelEditor() {
    if (!robot_model_window_) {
        robot_model_window_ = new RobotModelWindow(this);
        connect(robot_model_window_, &RobotModelWindow::parametersApplied, [this](const std::map<std::string, float>& params) {
            if (physics_world_) {
                physics_world_->setPhysicsParams(params);
                std::cout << "Applied new physics parameters." << std::endl;
            }
        });
        
        connect(robot_model_window_, &RobotModelWindow::colorChanged, [this](const std::string& name, float r, float g, float b, float a) {
            if (ui->visualizer_ && ui->visualizer_->getRobotModel()) {
                ui->visualizer_->getRobotModel()->setMaterialColor(name, r, g, b, a);
                ui->visualizer_->update(); // 즉각 렌더링하도록 확실하게 update 호출
            }
        });
    }

    // Refresh parameters from physics world before showing
    if (physics_world_) {
        robot_model_window_->setParameters(physics_world_->getPhysicsParams());
    }

    // Refresh colors only if the layout_colors is empty (prevents breaking states during consecutive double clicks)
    if (ui->visualizer_ && ui->visualizer_->getRobotModel() && robot_model_window_->layout()->count() > 0) {
        auto mats = ui->visualizer_->getRobotModel()->getMaterials();
        std::map<std::string, std::array<float, 4>> colors;
        for (const auto& [k, v] : mats) {
             colors[k] = {v.r, v.g, v.b, v.a};
        }
        robot_model_window_->setColors(colors);
    }
    
    robot_model_window_->show();
    robot_model_window_->activateWindow();
    robot_model_window_->raise();
}

void MainWindow::onOpenObstacleWindow() {
    if (!obstacle_window_) {
        obstacle_window_ = new ObstacleWindow(ui->visualizer_, nullptr);
        // Note: Theme propagation to ObstacleWindow can be implemented if needed
    }
    obstacle_window_->show();
    obstacle_window_->raise();
    obstacle_window_->activateWindow();
    obstacle_window_->updateTableFromVisualizer();
}
