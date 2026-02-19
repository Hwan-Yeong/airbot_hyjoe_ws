#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QPoint>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <QColor>

struct ColoredCloud {
    std::vector<float> points; // [x, y, z, ...]
    QColor color;
};

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
    virtual ~PointCloudVisualizer() = default;

    void updateCloud(const std::string& name, const std::vector<float>& points, QColor color = Qt::white);
    void updateTFs(const std::map<std::string, TfData>& tfs);
    void setTfScale(float s) { tf_scale_ = s; update(); }

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

    std::map<std::string, ColoredCloud> clouds_;
    std::map<std::string, TfData> tfs_;
    std::mutex cloud_mutex_;

    float tf_scale_ = 1.0f;

    // Camera state
    float yaw_ = -45.0f;   // degrees
    float pitch_ = -60.0f; // degrees
    float distance_ = 10.0f;
    float pan_x_ = 0.0f;
    float pan_y_ = 0.0f;
    float pan_z_ = 0.0f;

    QPoint last_mouse_pos_;
};
