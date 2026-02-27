#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QPoint>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <QColor>
#include <memory>
#include "sensor_gui/visualizer/obstacle_renderer.hpp"

struct ColoredCloud {
    std::string frame_id;
    std::vector<float> points; // [x, y, z, ...]
    QColor color;
};

class RobotModel;

struct TfData {
    std::string frame_id;
    std::string displayName;
    float x, y, z;
    float qx, qy, qz, qw;
};

class PointCloudVisualizer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit PointCloudVisualizer(QWidget* parent = nullptr);
    virtual ~PointCloudVisualizer();

    void updateCloud(const std::string& name, const std::string& frame_id, const std::vector<float>& points, QColor color = Qt::white);
    void updateTFs(const std::map<std::string, TfData>& tfs);
    void updateSingleTf(const TfData& tf) { std::lock_guard<std::mutex> lock(cloud_mutex_); tfs_[tf.frame_id] = tf; update(); }
    bool setRobotModelFromUrdf(const std::string& path);
    void setTfScale(float s) { tf_scale_ = s; update(); }
    void setFootprintRadius(float r) { footprint_radius_ = r; update(); }
    void setGroundClipping(bool enabled) { ground_clipping_ = enabled; update(); }
    void setWallSimulation(bool enabled) { wall_sim_ = enabled; update(); }
    void setBackgroundColor(const QColor& color);
    void setWallPosition(float x) { wall_x_ = x; update(); }
    void setObstacles(const std::vector<SimObstacle>& obs) { obstacles_ = obs; update(); }
    void setWheelPoses(const TfData& left, const TfData& right) { left_wheel_ = left; right_wheel_ = right; draw_wheels_ = true; update(); }
    std::vector<SimObstacle> getObstacles() const { return obstacles_; }
    const std::unique_ptr<RobotModel>& getRobotModel() const { return robot_model_; }
    void setTopView();
    void setSideView();

    int getSelectedObstacleIndex() const { return selected_idx_; }
    void setSelectedObstacleIndex(int idx) { selected_idx_ = idx; update(); }

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void paintEvent(QPaintEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void drawAxes();
    void drawGrid();
    void drawRobotFootprint();
    void drawWheels();
    void drawObstacles();
    void loadGroundTexture();
    void applyTf(const TfData& tf);
    void transformPoint(const TfData& tf, float lx, float ly, float lz, float& wx, float& wy, float& wz);

    std::map<std::string, ColoredCloud> clouds_;
    std::map<std::string, TfData> tfs_;
    std::vector<SimObstacle> obstacles_;
    std::mutex cloud_mutex_;

    float tf_scale_ = 1.0f;
    float footprint_radius_ = 0.19f;
    bool ground_clipping_ = true;
    bool wall_sim_ = false;
    float wall_x_ = 2.0f;
    QColor bg_color_ = QColor(242, 242, 230); // Default Ivory

    TfData left_wheel_;
    TfData right_wheel_;
    bool draw_wheels_ = false;

    GLuint ground_texture_ = 0;

    // Camera state
    float yaw_ = -45.0f;   // degrees
    float pitch_ = -60.0f; // degrees
    float distance_ = 10.0f;
    float pan_x_ = 0.0f;
    float pan_y_ = 0.0f;
    float pan_z_ = 0.0f;

    std::unique_ptr<RobotModel> robot_model_;
    QPoint last_mouse_pos_;
    int selected_idx_ = -1;
    bool dragging_ = false;

    std::map<ObstacleType, std::unique_ptr<IObstacleRenderer>> renderers_;

signals:
    void obstacleMoved(int index, float x, float y);
    void obstacleSelected(int index);
};
