#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_gui/main_window.hpp"
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
#include "sensor_gui/robot_model.hpp"
#include <QComboBox>

MainWindow::MainWindow(std::shared_ptr<RosNode> node)
    : QMainWindow(nullptr), ui(new Ui::MainWindow), ros_node_(node)
{
  ui->setupUi(this);
  initConnections();
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

MainWindow::~MainWindow() = default;

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

  ui->table_walls_->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->table_walls_->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(ui->table_walls_, &QTableWidget::itemSelectionChanged, this, &MainWindow::onTableSelectionChanged);

  connect(ui->btn_add_wall_, &QPushButton::clicked, this, &MainWindow::onAddWall);
  connect(ui->btn_delete_wall_, &QPushButton::clicked, this, &MainWindow::onDeleteWall);
  connect(ui->table_walls_, &QTableWidget::cellChanged, this, &MainWindow::onWallTableChanged);

  connect(ui->btn_load_map_, &QPushButton::clicked, this, &MainWindow::onLoadMap);
  connect(ui->btn_save_map_, &QPushButton::clicked, this, &MainWindow::onSaveMap);

  connect(ui->visualizer_, &PointCloudVisualizer::obstacleMoved, this, &MainWindow::onObstacleMoved);
  connect(ui->visualizer_, &PointCloudVisualizer::obstacleSelected, this, &MainWindow::onObstacleSelected);

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

  // Load Maps
  loadSettings();

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
  ui->visualizer_->updateTFs(tfs);

  // Ground Following Logic (Implementation)
  float rx = ros_node_->getRobotX();
  float ground_z = 0.0f;
  
  if (ui->check_bump_sim_ && ui->check_bump_sim_->isChecked()) {
      // Bump at X=[0.8, 1.2], Width=0.4m, Height=0.03m
      if (rx > 0.8f && rx < 1.2f) {
          ground_z = 0.03f;
      }
  }

  float target_robot_z = ground_z + 0.045f; // Add wheel radius
  ros_node_->setRobotZ(target_robot_z);
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

QComboBox* MainWindow::createTypeComboBox(ObstacleType type) {
    QComboBox* combo = new QComboBox();
    combo->addItem("Box", (int)ObstacleType::kBox);
    combo->addItem("Cylinder", (int)ObstacleType::kCylinder);
    combo->addItem("Cone", (int)ObstacleType::kCone);
    
    int index = combo->findData((int)type);
    if (index >= 0) combo->setCurrentIndex(index);
    
    connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this, combo](int){
        for(int i=0; i<ui->table_walls_->rowCount(); ++i) {
            if(ui->table_walls_->cellWidget(i, 0) == combo) {
                onWallTableChanged(i, 0);
                break;
            }
        }
    });
    return combo;
}

void MainWindow::onAddWall() {
    if (!ui->visualizer_) return;
    std::vector<SimObstacle> obs = ui->visualizer_->getObstacles();
    
    SimObstacle new_ob;
    new_ob.type = ObstacleType::kBox;
    new_ob.x = 1.0f; new_ob.y = 0.0f; new_ob.z = 0.5f;
    new_ob.sx = 0.5f; new_ob.sy = 1.0f; new_ob.sz = 1.0f;
    
    // Copy from last if exists
    if (!obs.empty()) {
        new_ob = obs.back();
    }
    
    obs.push_back(new_ob);
    ui->visualizer_->setObstacles(obs);
    
    ui->table_walls_->blockSignals(true);
    int row = ui->table_walls_->rowCount();
    ui->table_walls_->insertRow(row);
    
    ui->table_walls_->setCellWidget(row, 0, createTypeComboBox(new_ob.type));
    ui->table_walls_->setItem(row, 1, new QTableWidgetItem(QString::number(new_ob.x, 'f', 2)));
    ui->table_walls_->setItem(row, 2, new QTableWidgetItem(QString::number(new_ob.y, 'f', 2)));
    ui->table_walls_->setItem(row, 3, new QTableWidgetItem(QString::number(new_ob.z, 'f', 2)));
    ui->table_walls_->setItem(row, 4, new QTableWidgetItem(QString::number(new_ob.sx, 'f', 2)));
    ui->table_walls_->setItem(row, 5, new QTableWidgetItem(QString::number(new_ob.sy, 'f', 2)));
    ui->table_walls_->setItem(row, 6, new QTableWidgetItem(QString::number(new_ob.sz, 'f', 2)));
    ui->table_walls_->blockSignals(false);
}

void MainWindow::onDeleteWall() {
    int row = ui->table_walls_->currentRow();
    if (row >= 0 && ui->visualizer_) {
        std::vector<SimObstacle> obs = ui->visualizer_->getObstacles();
        if (row < (int)obs.size()) {
            obs.erase(obs.begin() + row);
            ui->visualizer_->setObstacles(obs);
        }
        ui->table_walls_->removeRow(row);
    }
}

void MainWindow::onWallTableChanged(int row, int col) {
    if (!ui->table_walls_ || !ui->visualizer_) return;
    (void)col;
    
    std::vector<SimObstacle> obs = ui->visualizer_->getObstacles();
    
    if (row >= 0 && row < ui->table_walls_->rowCount() && row < (int)obs.size()) {
        SimObstacle& ob = obs[row];
        QWidget* widget = ui->table_walls_->cellWidget(row, 0);
        QComboBox* combo = qobject_cast<QComboBox*>(widget);
        if (combo) ob.type = static_cast<ObstacleType>(combo->currentData().toInt());
        
        if (ui->table_walls_->item(row, 1)) ob.x = ui->table_walls_->item(row, 1)->text().toFloat();
        if (ui->table_walls_->item(row, 2)) ob.y = ui->table_walls_->item(row, 2)->text().toFloat();
        if (ui->table_walls_->item(row, 3)) ob.z = ui->table_walls_->item(row, 3)->text().toFloat();
        if (ui->table_walls_->item(row, 4)) ob.sx = ui->table_walls_->item(row, 4)->text().toFloat();
        if (ui->table_walls_->item(row, 5)) ob.sy = ui->table_walls_->item(row, 5)->text().toFloat();
        if (ui->table_walls_->item(row, 6)) ob.sz = ui->table_walls_->item(row, 6)->text().toFloat();
    }
    
    ui->visualizer_->setObstacles(obs);
}

void MainWindow::onObstacleMoved(int index, float x, float y) {
    if (index < 0 || index >= ui->table_walls_->rowCount()) return;
    ui->table_walls_->blockSignals(true);
    ui->table_walls_->item(index, 1)->setText(QString::number(x, 'f', 2)); // Shifted index
    ui->table_walls_->item(index, 2)->setText(QString::number(y, 'f', 2));
    ui->table_walls_->blockSignals(false);
}

void MainWindow::onObstacleSelected(int index) {
    ui->table_walls_->blockSignals(true);
    if (index >= 0) {
        ui->table_walls_->selectRow(index);
    } else {
        ui->table_walls_->clearSelection();
    }
    ui->table_walls_->blockSignals(false);
}

void MainWindow::loadSettings() {
    QSettings settings("Airbot", "SensorSimulator");
    last_map_path_ = settings.value("last_map_dir", "").toString();
}

void MainWindow::saveSettings() {
    QSettings settings("Airbot", "SensorSimulator");
    settings.setValue("last_map_dir", last_map_path_);
}

void MainWindow::onLoadMap() {
    QString filter = "Map Files (*.json);;All Files (*)";
    QString path = QFileDialog::getOpenFileName(this, "Load Map JSON", last_map_path_, filter);
    if (path.isEmpty()) return;

    last_map_path_ = QFileInfo(path).path();
    saveSettings();

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return;

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    QJsonArray arr = doc.array();

    std::vector<SimObstacle> obs;
    for (int i = 0; i < arr.size(); ++i) {
        QJsonObject obj = arr[i].toObject();
        SimObstacle o;
        o.type = static_cast<ObstacleType>(obj["type"].toInt());
        o.x = obj["x"].toDouble();
        o.y = obj["y"].toDouble();
        o.z = obj["z"].toDouble();
        o.sx = obj["sx"].toDouble();
        o.sy = obj["sy"].toDouble();
        o.sz = obj["sz"].toDouble();
        o.name = obj["name"].toString().toStdString();
        obs.push_back(o);
    }
    
    if (ui->visualizer_) ui->visualizer_->setObstacles(obs);
    
    ui->table_walls_->blockSignals(true);
    ui->table_walls_->setRowCount(0);

    for (size_t i = 0; i < obs.size(); ++i) {
        int row = ui->table_walls_->rowCount();
        ui->table_walls_->insertRow(row);
        
        ui->table_walls_->setCellWidget(row, 0, createTypeComboBox(obs[i].type));
        ui->table_walls_->setItem(row, 1, new QTableWidgetItem(QString::number(obs[i].x, 'f', 2)));
        ui->table_walls_->setItem(row, 2, new QTableWidgetItem(QString::number(obs[i].y, 'f', 2)));
        ui->table_walls_->setItem(row, 3, new QTableWidgetItem(QString::number(obs[i].z, 'f', 2)));
        ui->table_walls_->setItem(row, 4, new QTableWidgetItem(QString::number(obs[i].sx, 'f', 2)));
        ui->table_walls_->setItem(row, 5, new QTableWidgetItem(QString::number(obs[i].sy, 'f', 2)));
        ui->table_walls_->setItem(row, 6, new QTableWidgetItem(QString::number(obs[i].sz, 'f', 2)));
    }
    ui->table_walls_->blockSignals(false);
}

void MainWindow::onSaveMap() {
    QString filter = "Map Files (*.json);;All Files (*)";
    QString path = QFileDialog::getSaveFileName(this, "Save Map JSON", last_map_path_, filter);
    if (path.isEmpty()) return;
    if (!path.endsWith(".json")) path += ".json";

    last_map_path_ = QFileInfo(path).path();
    saveSettings();

    QJsonArray arr;
    auto obs = ui->visualizer_->getObstacles();
    for (const auto& o : obs) {
        QJsonObject obj;
        obj["type"] = static_cast<int>(o.type);
        obj["x"] = o.x;
        obj["y"] = o.y;
        obj["z"] = o.z;
        obj["sx"] = o.sx;
        obj["sy"] = o.sy;
        obj["sz"] = o.sz;
        obj["name"] = QString::fromStdString(o.name);
        arr.append(obj);
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) return;
    file.write(QJsonDocument(arr).toJson());
}

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

void MainWindow::onTableSelectionChanged() {
    if (!ui->visualizer_ || !ui->table_walls_) return;
    auto items = ui->table_walls_->selectedItems();
    if (items.isEmpty()) {
        ui->visualizer_->setSelectedObstacleIndex(-1);
    } else {
        ui->visualizer_->setSelectedObstacleIndex(items.first()->row());
    }
}
