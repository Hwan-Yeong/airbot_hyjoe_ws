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
#include <QColorDialog>
#include <QScrollArea>
#include <functional>
#include <tf2/utils.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_gui/robot_model.hpp"

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
  central_widget_ptr_ = new QWidget();
  QHBoxLayout* main_layout = new QHBoxLayout(central_widget_ptr_);

  // Sidebar with Scroll Area
  QScrollArea* scroll_area = new QScrollArea();
  scroll_area->setFixedWidth(260); // Slightly wider to accommodate scrollbar
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QWidget* sidebar = new QWidget();
  sidebar->setFixedWidth(250);
  QVBoxLayout* sidebar_layout = new QVBoxLayout(sidebar);
  sidebar_layout->setContentsMargins(10, 10, 10, 10);
  
  scroll_area->setWidget(sidebar);
  main_layout->addWidget(scroll_area);

  // Theme Toggle Button at the top
  btn_theme_toggle_ = new QPushButton("🌙 Dark Mode");
  btn_theme_toggle_->setStyleSheet("height: 35px; font-weight: bold; margin-bottom: 10px;");
  connect(btn_theme_toggle_, &QPushButton::clicked, this, &MainWindow::onToggleTheme);
  sidebar_layout->addWidget(btn_theme_toggle_);

  QLabel* title = new QLabel("Sensor Simulator");
  title->setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 20px; color: #2C2C2C;");
  sidebar_layout->addWidget(title);

  // Set Light Theme for Main Window
  central_widget_ptr_->setStyleSheet(R"(
    QWidget { 
        background-color: #FDFBF7; 
        color: #2C2C2C; 
        font-family: 'Segoe UI', sans-serif;
    }
    QGroupBox { 
        border: 1px solid #D1D1D1; 
        border-radius: 8px; 
        margin-top: 15px; 
        font-weight: bold;
        background-color: #FFFFFF;
    }
    QGroupBox::title { 
        subcontrol-origin: margin; 
        left: 10px; 
        padding: 0 5px; 
        color: #4A4A4A;
    }
    QPushButton {
        background-color: #FFFFFF;
        border: 1px solid #D1D1D1;
        border-radius: 6px;
        padding: 5px;
        height: 24px;
    }
    QPushButton:hover { background-color: #F0F0F0; }
    QPushButton:checked { background-color: #AED9FF; border-color: #7BB8FF; }
    QDoubleSpinBox {
        background-color: #FFFFFF;
        border: 1px solid #D1D1D1;
        border-radius: 4px;
        padding: 2px;
    }
    QCheckBox { spacing: 5px; }
  )");

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
  
  // Set default active buttons style for ToF sensors
  sensor_buttons_[SensorType::kTofMono]->setChecked(true);
  sensor_buttons_[SensorType::kTofMono]->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");
  sensor_buttons_[SensorType::kTofMultiLeft]->setChecked(true);
  sensor_buttons_[SensorType::kTofMultiLeft]->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");
  sensor_buttons_[SensorType::kTofMultiRight]->setChecked(true);
  sensor_buttons_[SensorType::kTofMultiRight]->setStyleSheet("height: 25px; margin-bottom: 2px; background-color: #2196F3; color: white;");

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

  auto create_param_row = [&](QVBoxLayout* layout, const QString& label, double val, double min, double max, QDoubleSpinBox** spin) {
    QHBoxLayout* row = new QHBoxLayout();
    row->addWidget(new QLabel(label));
    *spin = new QDoubleSpinBox();
    (*spin)->setRange(min, max);
    (*spin)->setValue(val);
    (*spin)->setSingleStep(0.1);
    (*spin)->setDecimals(2);
    connect(*spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onParamChanged);
    row->addWidget(*spin);
    layout->addLayout(row);
  };

  // 1. ToF Params
  QLabel* l_tof = new QLabel("── ToF Sensors ────────────────");
  l_tof->setStyleSheet("color: #6272a4; font-weight: bold; margin-top: 5px;");
  param_layout->addWidget(l_tof);
  create_param_row(param_layout, "  Mono Dist:", 0.5, 0.05, 5.0, &spin_tof_mono_dist_);
  create_param_row(param_layout, "  Left Dist:", 1.0, 0.05, 5.0, &spin_tof_left_dist_);
  create_param_row(param_layout, "  Right Dist:", 1.0, 0.05, 5.0, &spin_tof_right_dist_);

  // 2. Camera Params
  QLabel* l_cam = new QLabel("── Camera Array ───────────────");
  l_cam->setStyleSheet("color: #6272a4; font-weight: bold; margin-top: 10px;");
  param_layout->addWidget(l_cam);
  create_param_row(param_layout, "  Dist:", 0.3, 0.1, 3.0, &spin_cam_dist_);
  create_param_row(param_layout, "  Width:", 0.4, 0.1, 2.0, &spin_cam_width_);
  create_param_row(param_layout, "  Height:", 0.2, 0.1, 2.0, &spin_cam_height_);

  // 3. Global Params
  QLabel* l_glob = new QLabel("── Global / TF ────────────────");
  l_glob->setStyleSheet("color: #6272a4; font-weight: bold; margin-top: 10px;");
  param_layout->addWidget(l_glob);
  create_param_row(param_layout, "  TF Scale:", 1.0, 0.1, 10.0, &spin_tf_scale_);

  param_group->setLayout(param_layout);
  sidebar_layout->addWidget(param_group);

  // Robot Control
  QGroupBox* robot_group = new QGroupBox("Robot Control");
  QVBoxLayout* robot_layout = new QVBoxLayout();
  
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
  check_ground_clip_->setChecked(true);
  env_layout->addWidget(check_ground_clip_);
  
  check_wall_sim_ = new QCheckBox("Simulate Wall (X)");
  check_wall_sim_->setChecked(false);
  env_layout->addWidget(check_wall_sim_);

  check_bump_sim_ = new QCheckBox("Simulate Bump (Threshold)");
  connect(check_bump_sim_, &QCheckBox::toggled, this, &MainWindow::onToggleBump);
  env_layout->addWidget(check_bump_sim_);
  
  create_param_row(env_layout, "Wall X Pos:", 2.0, -5.0, 10.0, &spin_wall_x_);

  env_group->setLayout(env_layout);
  sidebar_layout->addWidget(env_group);

  connect(check_ground_clip_, &QCheckBox::toggled, this, &MainWindow::onParamChanged);
  connect(check_wall_sim_, &QCheckBox::toggled, this, &MainWindow::onParamChanged);

  QPushButton* btn_pick_bg = new QPushButton("🎨 Pick Background Color");
  btn_pick_bg->setStyleSheet("height: 30px; margin-top: 5px; background-color: #ffffff; border: 1px solid #d1d1d1; border-radius: 4px;");
  connect(btn_pick_bg, &QPushButton::clicked, this, &MainWindow::onPickBackgroundColor);
  env_layout->addWidget(btn_pick_bg);

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
    IrIndex idx = indices[i];
    QCheckBox* cb = new QCheckBox(labels[i]);
    cb->setChecked(true);
    connect(cb, &QCheckBox::toggled, [this, idx](bool checked) {
      ros_node_->setIrState(idx, checked);
    });
    ir_grid->addWidget(cb, i / 2, i % 2);
  }
  ir_group->setLayout(ir_grid);
  sidebar_layout->addWidget(ir_group);

  sidebar_layout->addStretch();

  // Custom PointCloud Visualizer
  visualizer_ = new PointCloudVisualizer(central_widget_ptr_);
  visualizer_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  visualizer_->setMinimumSize(800, 600);

  main_layout->addWidget(visualizer_, 1);

  setCentralWidget(central_widget_ptr_);
  resize(1200, 800);
  setWindowTitle("Airbot Sensor Simulator & Custom Cloud Visualizer");

  // Force initial state to Dark Mode (user preferred "이전이 낫다")
  is_dark_mode_ = false; // Set false first so toggle makes it true
  onToggleTheme(); 
  
  onParamChanged(); // Sync initial values to Node and Visualizer

  // Load URDF from package share directory
  try {
      std::string share_dir = ament_index_cpp::get_package_share_directory("sensor_gui");
      std::string urdf_path = share_dir + "/urdf/robot_urdf.xml";
      if (visualizer_) {
          if (!visualizer_->setRobotModelFromUrdf(urdf_path)) {
              std::cerr << "Failed to load URDF from: " << urdf_path << std::endl;
          } else {
              std::cout << "Successfully loaded URDF: " << urdf_path << std::endl;
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
  ros_node_->setTofMonoDist(spin_tof_mono_dist_->value());
  ros_node_->setTofLeftDist(spin_tof_left_dist_->value());
  ros_node_->setTofRightDist(spin_tof_right_dist_->value());
  ros_node_->setCameraParams(spin_cam_dist_->value(), spin_cam_width_->value(), spin_cam_height_->value());
  
  // Note: Robot pose (X/Y/Z/Yaw) spinboxes were removed. 
  // It's now primarily controlled by Teleop window or internal node logic.

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

  // Ground Following Logic (Implementation)
  float rx = ros_node_->getRobotX();
  float ground_z = 0.0f;
  
  if (check_bump_sim_ && check_bump_sim_->isChecked()) {
      // Bump at X=[0.8, 1.2], Width=0.4m, Height=0.03m
      if (rx > 0.8f && rx < 1.2f) {
          ground_z = 0.03f;
      }
  }

  float target_robot_z = ground_z + 0.045f; // Add wheel radius
  ros_node_->setRobotZ(target_robot_z);
}

void MainWindow::onToggleBump(bool checked) {
    if (!visualizer_) return;
    
    std::vector<BoxObject> current_walls;
    // Note: Manage walls specifically for this demo
    if (checked) {
        BoxObject bump;
        bump.x = 1.0f; bump.y = 0.0f; bump.z = 0.015f;
        bump.sx = 0.4f; bump.sy = 4.0f; bump.sz = 0.03f;
        current_walls.push_back(bump);
    }
    visualizer_->setWalls(current_walls);
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

void MainWindow::onPickBackgroundColor() {
    if (!visualizer_) return;
    QColor color = QColorDialog::getColor(is_dark_mode_ ? QColor(13, 13, 26) : QColor(242, 242, 230), 
                                         this, "Select 3D View Background Color");
    if (color.isValid()) {
        visualizer_->setBackgroundColor(color);
    }
}

void MainWindow::onToggleTheme() {
    is_dark_mode_ = !is_dark_mode_;
    
    if (is_dark_mode_) {
        btn_theme_toggle_->setText("🌙 Dark Mode");
        central_widget_ptr_->setStyleSheet(R"(
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
        if (visualizer_) visualizer_->setBackgroundColor(QColor(13, 13, 26));
    } else {
        btn_theme_toggle_->setText("☀️ Light Mode");
        central_widget_ptr_->setStyleSheet(R"(
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
        if (visualizer_) visualizer_->setBackgroundColor(QColor(242, 242, 230));
    }

    if (teleop_window_) {
        teleop_window_->setTheme(is_dark_mode_);
    }
}

void MainWindow::onOpenTeleop() {
    if (!teleop_window_) {
        teleop_window_ = new TeleopWindow(nullptr); // 독립 창
        teleop_window_->setVelCallback([this](float vx, float vy, float vyaw) {
            if (ros_node_) ros_node_->setVelocities(vx, vy, vyaw);
        });
        teleop_window_->setTheme(is_dark_mode_);
    }
    teleop_window_->show();
    teleop_window_->raise();
    teleop_window_->activateWindow();
    teleop_window_->setFocus();
}
