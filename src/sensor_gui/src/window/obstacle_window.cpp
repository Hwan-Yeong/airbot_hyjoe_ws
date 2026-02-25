#include "sensor_gui/window/obstacle_window.hpp"
#include "ui_obstacle_window.h"
#include <QFileDialog>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QFileInfo>

ObstacleWindow::ObstacleWindow(PointCloudVisualizer* visualizer, QWidget* parent)
    : QWidget(parent), ui(new Ui::Obstacles), visualizer_(visualizer) 
{
    ui->setupUi(this);
    setWindowFlags(Qt::Window); // Make it an independent window
    
    loadSettings();
    initConnections();
    updateTableFromVisualizer();
}

ObstacleWindow::~ObstacleWindow() = default;

void ObstacleWindow::initConnections() {
    ui->table_walls_1->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->table_walls_1->setSelectionMode(QAbstractItemView::SingleSelection);

    connect(ui->table_walls_1, &QTableWidget::itemSelectionChanged, this, &ObstacleWindow::onTableSelectionChanged);
    connect(ui->btn_spawn_box, &QPushButton::clicked, this, [this](){ onSpawnShape(ObstacleType::kBox); });
    connect(ui->btn_spawn_cylinder, &QPushButton::clicked, this, [this](){ onSpawnShape(ObstacleType::kCylinder); });
    connect(ui->btn_spawn_cone, &QPushButton::clicked, this, [this](){ onSpawnShape(ObstacleType::kCone); });
    connect(ui->btn_spawn_sphere, &QPushButton::clicked, this, [this](){ onSpawnShape(ObstacleType::kSphere); });
    
    connect(ui->btn_add_wall_, &QPushButton::clicked, this, &ObstacleWindow::onAddWall);
    connect(ui->btn_delete_wall_, &QPushButton::clicked, this, &ObstacleWindow::onDeleteWall);
    
    // Properties SpinBox changes
    auto spinChanged = QOverload<double>::of(&QDoubleSpinBox::valueChanged);
    connect(ui->spin_x, spinChanged, this, &ObstacleWindow::onPropertySpinBoxChanged);
    connect(ui->spin_y, spinChanged, this, &ObstacleWindow::onPropertySpinBoxChanged);
    connect(ui->spin_z, spinChanged, this, &ObstacleWindow::onPropertySpinBoxChanged);
    connect(ui->spin_sx, spinChanged, this, &ObstacleWindow::onPropertySpinBoxChanged);
    connect(ui->spin_sy, spinChanged, this, &ObstacleWindow::onPropertySpinBoxChanged);
    connect(ui->spin_sz, spinChanged, this, &ObstacleWindow::onPropertySpinBoxChanged);

    connect(ui->table_walls_1, &QTableWidget::cellChanged, this, &ObstacleWindow::onWallTableChanged);

    connect(ui->btn_load_map_, &QPushButton::clicked, this, &ObstacleWindow::onLoadMap);
    connect(ui->btn_save_map_, &QPushButton::clicked, this, &ObstacleWindow::onSaveMap);

    if (visualizer_) {
        connect(visualizer_, &PointCloudVisualizer::obstacleMoved, this, &ObstacleWindow::onObstacleMoved);
        connect(visualizer_, &PointCloudVisualizer::obstacleSelected, this, &ObstacleWindow::onObstacleSelected);
    }
}

QComboBox* ObstacleWindow::createTypeComboBox(ObstacleType type) {
    QComboBox* combo = new QComboBox();
    combo->addItem("Box", (int)ObstacleType::kBox);
    combo->addItem("Cylinder", (int)ObstacleType::kCylinder);
    combo->addItem("Cone", (int)ObstacleType::kCone);
    combo->addItem("Sphere", (int)ObstacleType::kSphere);
    
    int index = combo->findData((int)type);
    if (index >= 0) combo->setCurrentIndex(index);
    
    connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this, combo](int){
        for(int i=0; i<ui->table_walls_1->rowCount(); ++i) {
            if(ui->table_walls_1->cellWidget(i, 1) == combo) {
                onWallTableChanged(i, 1);
                updatePropertiesPanel(i);
                break;
            }
        }
    });
    return combo;
}

void ObstacleWindow::updateTableFromVisualizer() {
    if (!visualizer_) return;
    
    ui->table_walls_1->blockSignals(true);
    int current_row = ui->table_walls_1->currentRow();
    ui->table_walls_1->setRowCount(0);
    
    auto obs = visualizer_->getObstacles();
    for (size_t i = 0; i < obs.size(); ++i) {
        int row = ui->table_walls_1->rowCount();
        ui->table_walls_1->insertRow(row);
        
        QString default_name = QString::fromStdString(obs[i].name);
        if (default_name.isEmpty()) default_name = QString("Obstacle_%1").arg(i);

        ui->table_walls_1->setItem(row, 0, new QTableWidgetItem(default_name));
        ui->table_walls_1->setCellWidget(row, 1, createTypeComboBox(obs[i].type));
    }
    ui->table_walls_1->blockSignals(false);
    
    if (current_row >= 0 && current_row < ui->table_walls_1->rowCount()) {
        ui->table_walls_1->selectRow(current_row);
    } else {
        updatePropertiesPanel(-1);
    }
}

void ObstacleWindow::onAddWall() {
    if (!visualizer_) return;
    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    
    SimObstacle new_ob;
    new_ob.type = ObstacleType::kBox;
    new_ob.x = 1.0f; new_ob.y = 0.0f; new_ob.z = 0.5f;
    new_ob.sx = 0.5f; new_ob.sy = 1.0f; new_ob.sz = 1.0f;
    
    // Copy from last if exists
    if (!obs.empty()) {
        new_ob = obs.back();
    }
    
    obs.push_back(new_ob);
    visualizer_->setObstacles(obs);
    
    updateTableFromVisualizer();
}

void ObstacleWindow::onSpawnShape(ObstacleType type) {
    if (!visualizer_) return;
    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    
    SimObstacle new_ob;
    new_ob.type = type;
    new_ob.x = 1.0f; new_ob.y = 0.0f; new_ob.z = 0.5f;
    new_ob.sx = 0.5f; new_ob.sy = 1.0f; new_ob.sz = 1.0f;
    
    if (!obs.empty()) {
        new_ob.sx = obs.back().sx;
        new_ob.sy = obs.back().sy;
        new_ob.sz = obs.back().sz;
        new_ob.z = obs.back().z;
    }
    
    obs.push_back(new_ob);
    visualizer_->setObstacles(obs);
    
    updateTableFromVisualizer();
}

void ObstacleWindow::onDeleteWall() {
    int row = ui->table_walls_1->currentRow();
    if (row >= 0 && visualizer_) {
        std::vector<SimObstacle> obs = visualizer_->getObstacles();
        if (row < (int)obs.size()) {
            obs.erase(obs.begin() + row);
            visualizer_->setObstacles(obs);
        }
        ui->table_walls_1->removeRow(row);
    }
}

void ObstacleWindow::onWallTableChanged(int row, int col) {
    if (!ui->table_walls_1 || !visualizer_) return;
    
    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    
    if (row >= 0 && row < ui->table_walls_1->rowCount() && row < (int)obs.size()) {
        SimObstacle& ob = obs[row];
        if (col == 0) {
            ob.name = ui->table_walls_1->item(row, 0)->text().toStdString();
        } else if (col == 1) {
            QWidget* widget = ui->table_walls_1->cellWidget(row, 1);
            QComboBox* combo = qobject_cast<QComboBox*>(widget);
            if (combo) ob.type = static_cast<ObstacleType>(combo->currentData().toInt());
        }
    }
    visualizer_->setObstacles(obs);
}

void ObstacleWindow::onPropertySpinBoxChanged() {
    int row = ui->table_walls_1->currentRow();
    if (row < 0 || !visualizer_) return;

    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    if (row >= (int)obs.size()) return;

    SimObstacle& ob = obs[row];
    ob.x = ui->spin_x->value();
    ob.y = ui->spin_y->value();
    ob.z = ui->spin_z->value();
    
    ob.sx = ui->spin_sx->value();
    if (ui->spin_sy->isVisible()) ob.sy = ui->spin_sy->value();
    if (ui->spin_sz->isVisible()) ob.sz = ui->spin_sz->value();
    
    visualizer_->setObstacles(obs);
}

void ObstacleWindow::updatePropertiesPanel(int index) {
    bool enable = (index >= 0);
    ui->groupBox_properties->setEnabled(enable);
    if (!enable || !visualizer_) return;

    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    if (index >= (int)obs.size()) return;

    const SimObstacle& ob = obs[index];
    
    ui->spin_x->blockSignals(true); ui->spin_y->blockSignals(true); ui->spin_z->blockSignals(true);
    ui->spin_sx->blockSignals(true); ui->spin_sy->blockSignals(true); ui->spin_sz->blockSignals(true);
    
    ui->spin_x->setValue(ob.x);
    ui->spin_y->setValue(ob.y);
    ui->spin_z->setValue(ob.z);
    
    ui->spin_sx->setValue(ob.sx);
    ui->spin_sy->setValue(ob.sy);
    ui->spin_sz->setValue(ob.sz);

    // Dynamic Labels and Visibility
    if (ob.type == ObstacleType::kBox) {
        ui->lbl_sx->setText("Length (X):"); ui->spin_sx->setVisible(true); ui->lbl_sx->setVisible(true);
        ui->lbl_sy->setText("Width (Y):"); ui->spin_sy->setVisible(true); ui->lbl_sy->setVisible(true);
        ui->lbl_sz->setText("Height (Z):"); ui->spin_sz->setVisible(true); ui->lbl_sz->setVisible(true);
    } else if (ob.type == ObstacleType::kCylinder || ob.type == ObstacleType::kCone) {
        ui->lbl_sx->setText("Radius:"); ui->spin_sx->setVisible(true); ui->lbl_sx->setVisible(true);
        ui->lbl_sy->setText("Width (Y):"); ui->spin_sy->setVisible(false); ui->lbl_sy->setVisible(false);
        ui->lbl_sz->setText("Height:"); ui->spin_sz->setVisible(true); ui->lbl_sz->setVisible(true);
    } else if (ob.type == ObstacleType::kSphere) {
        ui->lbl_sx->setText("Radius:"); ui->spin_sx->setVisible(true); ui->lbl_sx->setVisible(true);
        ui->lbl_sy->setText("Width (Y):"); ui->spin_sy->setVisible(false); ui->lbl_sy->setVisible(false);
        ui->lbl_sz->setText("Height (Z):"); ui->spin_sz->setVisible(false); ui->lbl_sz->setVisible(false);
    }
    
    ui->spin_x->blockSignals(false); ui->spin_y->blockSignals(false); ui->spin_z->blockSignals(false);
    ui->spin_sx->blockSignals(false); ui->spin_sy->blockSignals(false); ui->spin_sz->blockSignals(false);
}

void ObstacleWindow::onObstacleMoved(int index, float x, float y) {
    if (index < 0 || index >= ui->table_walls_1->rowCount() || !visualizer_) return;
    
    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    if (index < (int)obs.size()) {
        if (ui->table_walls_1->currentRow() == index) {
            ui->spin_x->blockSignals(true);
            ui->spin_y->blockSignals(true);
            ui->spin_x->setValue(x);
            ui->spin_y->setValue(y);
            ui->spin_x->blockSignals(false);
            ui->spin_y->blockSignals(false);
        }
    }
}

void ObstacleWindow::onObstacleSelected(int index) {
    ui->table_walls_1->blockSignals(true);
    if (index >= 0 && index < ui->table_walls_1->rowCount()) {
        ui->table_walls_1->selectRow(index);
    } else {
        ui->table_walls_1->clearSelection();
    }
    ui->table_walls_1->blockSignals(false);
}

void ObstacleWindow::onTableSelectionChanged() {
    if (!visualizer_ || !ui->table_walls_1) return;
    auto items = ui->table_walls_1->selectedItems();
    if (items.isEmpty()) {
        visualizer_->setSelectedObstacleIndex(-1);
        updatePropertiesPanel(-1);
    } else {
        int row = items.first()->row();
        visualizer_->setSelectedObstacleIndex(row);
        updatePropertiesPanel(row);
    }
}

void ObstacleWindow::loadSettings() {
    QSettings settings("Airbot", "SensorSimulator");
    last_map_path_ = settings.value("last_map_dir", "").toString();
}

void ObstacleWindow::saveSettings() {
    QSettings settings("Airbot", "SensorSimulator");
    settings.setValue("last_map_dir", last_map_path_);
}

void ObstacleWindow::onLoadMap() {
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
    
    if (visualizer_) visualizer_->setObstacles(obs);
    
    updateTableFromVisualizer();
}

void ObstacleWindow::onSaveMap() {
    QString filter = "Map Files (*.json);;All Files (*)";
    QString path = QFileDialog::getSaveFileName(this, "Save Map JSON", last_map_path_, filter);
    if (path.isEmpty()) return;
    if (!path.endsWith(".json")) path += ".json";

    last_map_path_ = QFileInfo(path).path();
    saveSettings();

    QJsonArray arr;
    if (!visualizer_) return;
    
    auto obs = visualizer_->getObstacles();
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
