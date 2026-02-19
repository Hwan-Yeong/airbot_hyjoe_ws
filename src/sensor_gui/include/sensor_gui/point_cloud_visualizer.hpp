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

class PointCloudVisualizer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit PointCloudVisualizer(QWidget* parent = nullptr);
    virtual ~PointCloudVisualizer() = default;

    void updateCloud(const std::string& name, const std::vector<float>& points, QColor color = Qt::white);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void drawAxes();
    void drawGrid();

    std::map<std::string, ColoredCloud> clouds_;
    std::mutex cloud_mutex_;

    // Camera state
    float yaw_ = 45.0f;   // degrees
    float pitch_ = 35.0f; // degrees
    float distance_ = 5.0f;
    QPoint last_mouse_pos_;
};
