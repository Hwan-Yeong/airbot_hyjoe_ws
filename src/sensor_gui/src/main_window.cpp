#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>

#include "sensor_gui/main_window.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <QApplication>

MainWindow::MainWindow(std::shared_ptr<RosNode> node)
    : ros_node_(node), manager_(nullptr)
{
  setupUi();
}

void MainWindow::showEvent(QShowEvent *event) {
  QMainWindow::showEvent(event);
  if (!manager_) {
    QTimer::singleShot(200, this, [this]() {
        initRviz();
    });
  }
}

void MainWindow::setupUi() {
  auto central = new QWidget();
  auto main_layout = new QHBoxLayout();

  // Sidebar
  auto sidebar = new QWidget();
  sidebar->setFixedWidth(250);
  auto sidebar_layout = new QVBoxLayout(sidebar);

  auto title = new QLabel("Sensor Simulator");
  title->setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 20px;");
  sidebar_layout->addWidget(title);

  // Sensor Manager Control
  btn_sensor_manager_ = new QPushButton("Sensor Manager: OFF");
  btn_sensor_manager_->setCheckable(true);
  btn_sensor_manager_->setStyleSheet("height: 40px; background-color: #f44336; color: white; border-radius: 5px;");
  connect(btn_sensor_manager_, &QPushButton::clicked, this, &MainWindow::onToggleSensorManager);
  sidebar_layout->addWidget(btn_sensor_manager_);

  sidebar_layout->addSpacing(20);

  // Individual Sensors
  auto sensor_group = new QGroupBox("Virtual Sensors");
  auto group_layout = new QVBoxLayout();

  auto create_sensor_btn = [&](SensorType type, const QString& name) {
    auto btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setProperty("sensor_type", static_cast<int>(type));
    btn->setStyleSheet("height: 30px; margin-bottom: 5px;");
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

  sidebar_layout->addStretch();

  // RViz Panel
  render_panel_ = new rviz_common::RenderPanel();

  main_layout->addWidget(sidebar);
  main_layout->addWidget(render_panel_, 1);

  central->setLayout(main_layout);
  setCentralWidget(central);
  
  resize(1200, 800);
  setWindowTitle("Airbot Sensor Simulator & RViz2");
}

void MainWindow::initRviz() {
  if (!QGuiApplication::instance()) return;

  ros_node_abstraction_ =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
        "rviz_embedded_node");

  // Ensure render_panel is truly ready
  render_panel_->show();
  qApp->processEvents();
  render_panel_->winId();
  
  auto clock = ros_node_->get_clock();

  try {
    manager_ = new rviz_common::VisualizationManager(
        render_panel_, 
        ros_node_abstraction_, 
        nullptr, 
        clock);
  } catch (...) {
    return;
  }
  
  if (!manager_) return;

  render_panel_->initialize(manager_, false);
  manager_->initialize();
  manager_->startUpdate();

  manager_->setFixedFrame("map");

  auto grid = manager_->createDisplay("rviz_default_plugins/Grid", "Global Grid", true);
  if (grid) {
    grid->subProp("Line Style")->setValue("Lines");
  }

  manager_->createDisplay("rviz_default_plugins/TF", "Transforms", true);

  auto add_pc = [this](const QString& name, const QString& topic) {
    auto pc = manager_->createDisplay("rviz_default_plugins/PointCloud2", name, true);
    if (pc) {
      pc->subProp("Topic")->setValue(topic);
      pc->subProp("Size (m)")->setValue(0.01);
    }
  };

  add_pc("ToF Mono PC", "/sensor_to_pointcloud/tof/mono");
  add_pc("ToF Multi L PC", "/sensor_to_pointcloud/tof/multi/left");
  add_pc("ToF Multi R PC", "/sensor_to_pointcloud/tof/multi/right");
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
    btn->setStyleSheet("height: 30px; margin-bottom: 5px; background-color: #2196F3; color: white;");
  } else {
    btn->setStyleSheet("height: 30px; margin-bottom: 5px;");
  }
}
