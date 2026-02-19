#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_rendering/render_window.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_gui/main_window.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <QApplication>
#include <QPushButton>
#include <QSizePolicy>
#include <functional>

MainWindow::MainWindow(std::shared_ptr<RosNode> node)
    : ros_node_(node), manager_(nullptr)
{
  setupUi();
}

void MainWindow::showEvent(QShowEvent *event) {
  QMainWindow::showEvent(event);
  if (!manager_) {
    QTimer::singleShot(1000, this, [this]() {
        initRviz();
    });
  }
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

  sidebar_layout->addSpacing(20);

  // Individual Sensors
  QGroupBox* sensor_group = new QGroupBox("Virtual Sensors");
  QVBoxLayout* group_layout = new QVBoxLayout();

  auto create_sensor_btn = [&](SensorType type, const QString& name) {
    QPushButton* btn = new QPushButton(name);
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
  render_panel_ = new rviz_common::RenderPanel(central);
  render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  render_panel_->setMinimumSize(800, 600);

  main_layout->addWidget(sidebar);
  main_layout->addWidget(render_panel_, 1);

  setCentralWidget(central);
  resize(1200, 800);
  setWindowTitle("Airbot Sensor Simulator & RViz2");
}

void MainWindow::initRviz() {
  if (!QGuiApplication::instance()) return;

  printf("[INFO] MainWindow::initRviz() started (Delayed)\n"); fflush(stdout);
  
  ros_node_abstraction_ =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
        "rviz_embedded_node");

  // Ensure render_panel is truly ready
  printf("[DEBUG] RenderPanel size before init: %d x %d\n", render_panel_->width(), render_panel_->height());
  
  // Test if widget is visible at all
  render_panel_->setStyleSheet("background-color: magenta;");
  
  // CRITICAL for Ogre embedding: Prevent Qt from clearing the background
  render_panel_->setAttribute(Qt::WA_NoSystemBackground, true);
  render_panel_->setAttribute(Qt::WA_OpaquePaintEvent, true);
  render_panel_->setAttribute(Qt::WA_NativeWindow, true);
  render_panel_->setAttribute(Qt::WA_PaintOnScreen, true);
  
  render_panel_->show();
  qApp->processEvents();
  uint64_t window_id = render_panel_->winId();
  printf("[DEBUG] RenderPanel Window ID: %lu\n", window_id);
  
  auto clock = ros_node_->get_clock();

  try {
    printf("[DEBUG] Creating VisualizationManager...\n"); fflush(stdout);
    manager_ = new rviz_common::VisualizationManager(
        render_panel_, 
        ros_node_abstraction_, 
        nullptr, 
        clock);
    
    // 1. Initialize manager first (creates RenderSystem)
    printf("[DEBUG] Initializing VisualizationManager...\n"); fflush(stdout);
    manager_->initialize();
    
    // 2. Initialize RenderPanel (creates RenderWindow)
    printf("[DEBUG] Initializing RenderPanel...\n"); fflush(stdout);
    render_panel_->initialize(manager_);
    
    // 3. Start update loop
    manager_->startUpdate();

    // --- Load Package RViz Config ---
    bool config_loaded = false;
    try {
      std::string package_share = ament_index_cpp::get_package_share_directory("sensor_gui");
      std::string config_path = package_share + "/rviz/default.rviz";
      printf("[INFO] Loading package RViz config from: %s\n", config_path.c_str());
      
      rviz_common::YamlConfigReader reader;
      rviz_common::Config config;
      reader.readFile(config, QString::fromStdString(config_path));
      
      if (!reader.error()) {
        rviz_common::Config v_config = config.mapGetChild("Visualization Manager");
        if (v_config.isValid()) {
          printf("[INFO] Found 'Visualization Manager' node, loading displays/views...\n");
          manager_->load(v_config);
          config_loaded = true;
        } else {
          printf("[WARN] 'Visualization Manager' node not found. Trying entire config...\n");
          manager_->load(config);
          config_loaded = true;
        }
      } else {
        printf("[WARN] Could not load package config: %s\n", 
               reader.errorMessage().toStdString().c_str());
      }
    } catch (const std::exception& e) {
       printf("[WARN] Exception finding package share directory: %s\n", e.what());
    }

    // --- Fallback or Add Required Displays if not already there ---
    manager_->setFixedFrame("base_link");
    
    // Set a very distinctive background color to see if Ogre is rendering at all
    auto global_options = manager_->getRootDisplayGroup()->subProp("Global Options");
    if (global_options) {
      global_options->subProp("Background Color")->setValue(QColor(255, 255, 0)); // Bright Yellow
    }

    if (config_loaded) {
      if (manager_->getViewManager() && manager_->getViewManager()->getCurrent()) {
        printf("[INFO] Resetting camera view and setting distance...\n");
        manager_->getViewManager()->getCurrent()->reset();
        manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue(50.0f);
      }
    }
    
    // Always ensure some visual reference
    auto grid = manager_->createDisplay("rviz_default_plugins/Grid", "Reference Grid", true);
    if (grid) {
      grid->subProp("Color")->setValue(QColor(0, 255, 0)); // Bright Green
      grid->subProp("Cell Size")->setValue(10.0);
      grid->subProp("Plane Cell Count")->setValue(10);
    }

    auto axes = manager_->createDisplay("rviz_default_plugins/Axes", "Reference Axes", true);
    if (axes) {
      axes->subProp("Length")->setValue(5.0);
      axes->subProp("Radius")->setValue(0.2);
    }
    
    printf("[INFO] MainWindow::initRviz() successfully completed\n"); fflush(stdout);
    
    // Status Monitor Timer (to see if frames and displays are OK)
    auto status_timer = new QTimer(this);
    connect(status_timer, &QTimer::timeout, this, [this]() {
        if (!manager_) return;
        printf("\n--- RViz Runtime Status ---\n");
        printf("  Fixed Frame: %s\n", manager_->getFixedFrame().toStdString().c_str());
        
        auto root = manager_->getRootDisplayGroup();
        if (root) {
            printf("  Displays (%d):\n", root->numDisplays());
            for (int i = 0; i < root->numDisplays(); ++i) {
                auto disp = root->getDisplayAt(i);
                printf("    - [%s] %s\n", 
                       disp->getBool() ? "ON " : "OFF",
                       disp->getName().toStdString().c_str());
            }
        }
        fflush(stdout);
        
        // Force rendering updates
        render_panel_->update();
        this->update();
    });
    status_timer->start(2000); // 2 second intervals

    render_panel_->update();
    this->update();

  } catch (const std::exception& e) {
    printf("[ERROR] initRviz exception: %s\n", e.what()); fflush(stdout);
  } catch (...) {
    printf("[ERROR] initRviz unknown exception\n"); fflush(stdout);
  }
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
