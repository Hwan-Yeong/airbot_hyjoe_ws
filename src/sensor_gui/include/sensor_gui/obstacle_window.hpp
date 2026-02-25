#pragma once

#include <QWidget>
#include <vector>
#include <memory>
#include <QString>
#include "point_cloud_visualizer.hpp"
#include <QComboBox>
#include <QTableWidgetItem>

QT_BEGIN_NAMESPACE
namespace Ui { class Obstacles; }
QT_END_NAMESPACE

class ObstacleWindow : public QWidget {
    Q_OBJECT

public:
    explicit ObstacleWindow(PointCloudVisualizer* visualizer, QWidget* parent = nullptr);
    ~ObstacleWindow();

    void updateTableFromVisualizer();

private slots:
    void onAddWall();
    void onDeleteWall();
    void onWallTableChanged(int row, int col);
    void onLoadMap();
    void onSaveMap();
    void onObstacleMoved(int index, float x, float y);
    void onObstacleSelected(int index);
    void onTableSelectionChanged();

private:
    std::unique_ptr<Ui::Obstacles> ui;
    PointCloudVisualizer* visualizer_;
    QString last_map_path_;

    void initConnections();
    QComboBox* createTypeComboBox(ObstacleType type);
    void loadSettings();
    void saveSettings();
};
