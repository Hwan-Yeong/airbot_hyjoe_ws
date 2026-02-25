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
    connect(ui->btn_add_wall_, &QPushButton::clicked, this, &ObstacleWindow::onAddWall);
    connect(ui->btn_delete_wall_, &QPushButton::clicked, this, &ObstacleWindow::onDeleteWall);
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
    
    int index = combo->findData((int)type);
    if (index >= 0) combo->setCurrentIndex(index);
    
    connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this, combo](int){
        for(int i=0; i<ui->table_walls_1->rowCount(); ++i) {
            if(ui->table_walls_1->cellWidget(i, 0) == combo) {
                onWallTableChanged(i, 0);
                break;
            }
        }
    });
    return combo;
}

void ObstacleWindow::updateTableFromVisualizer() {
    if (!visualizer_) return;
    
    ui->table_walls_1->blockSignals(true);
    ui->table_walls_1->setRowCount(0);
    
    auto obs = visualizer_->getObstacles();
    for (size_t i = 0; i < obs.size(); ++i) {
        int row = ui->table_walls_1->rowCount();
        ui->table_walls_1->insertRow(row);
        
        ui->table_walls_1->setCellWidget(row, 0, createTypeComboBox(obs[i].type));
        ui->table_walls_1->setItem(row, 1, new QTableWidgetItem(QString::number(obs[i].x, 'f', 2)));
        ui->table_walls_1->setItem(row, 2, new QTableWidgetItem(QString::number(obs[i].y, 'f', 2)));
        ui->table_walls_1->setItem(row, 3, new QTableWidgetItem(QString::number(obs[i].z, 'f', 2)));
        ui->table_walls_1->setItem(row, 4, new QTableWidgetItem(QString::number(obs[i].sx, 'f', 2)));
        ui->table_walls_1->setItem(row, 5, new QTableWidgetItem(QString::number(obs[i].sy, 'f', 2)));
        ui->table_walls_1->setItem(row, 6, new QTableWidgetItem(QString::number(obs[i].sz, 'f', 2)));
    }
    ui->table_walls_1->blockSignals(false);
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
    (void)col;
    
    std::vector<SimObstacle> obs = visualizer_->getObstacles();
    
    if (row >= 0 && row < ui->table_walls_1->rowCount() && row < (int)obs.size()) {
        SimObstacle& ob = obs[row];
        QWidget* widget = ui->table_walls_1->cellWidget(row, 0);
        QComboBox* combo = qobject_cast<QComboBox*>(widget);
        if (combo) ob.type = static_cast<ObstacleType>(combo->currentData().toInt());
        
        if (ui->table_walls_1->item(row, 1)) ob.x = ui->table_walls_1->item(row, 1)->text().toFloat();
        if (ui->table_walls_1->item(row, 2)) ob.y = ui->table_walls_1->item(row, 2)->text().toFloat();
        if (ui->table_walls_1->item(row, 3)) ob.z = ui->table_walls_1->item(row, 3)->text().toFloat();
        if (ui->table_walls_1->item(row, 4)) ob.sx = ui->table_walls_1->item(row, 4)->text().toFloat();
        if (ui->table_walls_1->item(row, 5)) ob.sy = ui->table_walls_1->item(row, 5)->text().toFloat();
        if (ui->table_walls_1->item(row, 6)) ob.sz = ui->table_walls_1->item(row, 6)->text().toFloat();
    }
    
    visualizer_->setObstacles(obs);
}

void ObstacleWindow::onObstacleMoved(int index, float x, float y) {
    if (index < 0 || index >= ui->table_walls_1->rowCount()) return;
    ui->table_walls_1->blockSignals(true);
    ui->table_walls_1->item(index, 1)->setText(QString::number(x, 'f', 2));
    ui->table_walls_1->item(index, 2)->setText(QString::number(y, 'f', 2));
    ui->table_walls_1->blockSignals(false);
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
    } else {
        visualizer_->setSelectedObstacleIndex(items.first()->row());
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
