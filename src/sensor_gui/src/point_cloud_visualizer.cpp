#include "sensor_gui/point_cloud_visualizer.hpp"
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>
#include <GL/gl.h>
#include <GL/glu.h>

PointCloudVisualizer::PointCloudVisualizer(QWidget* parent)
    : QOpenGLWidget(parent) {}

void PointCloudVisualizer::updateCloud(const std::string& name, const std::vector<float>& points, QColor color) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    clouds_[name] = {points, color};
    update(); // Request repaint
}

void PointCloudVisualizer::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void PointCloudVisualizer::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = static_cast<float>(w) / static_cast<float>(h ? h : 1);
    gluPerspective(45.0, aspect, 0.1, 1000.0);
}

void PointCloudVisualizer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera Transformation (Orbit)
    // distance * z -> pitch -> yaw
    glTranslatef(0, 0, -distance_);
    glRotatef(pitch_, 1, 0, 0);
    glRotatef(yaw_, 0, 0, 1);

    drawGrid();
    drawAxes();

    // Draw Clouds
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    glPointSize(2.0f);
    for (auto const& [name, cloud] : clouds_) {
        glColor3f(cloud.color.redF(), cloud.color.greenF(), cloud.color.blueF());
        glBegin(GL_POINTS);
        for (size_t i = 0; i + 2 < cloud.points.size(); i += 3) {
            glVertex3f(cloud.points[i], cloud.points[i+1], cloud.points[i+2]);
        }
        glEnd();
    }
}

void PointCloudVisualizer::drawAxes() {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X - Red
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(1, 0, 0);
    // Y - Green
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 1, 0);
    // Z - Blue
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1);
    glEnd();
}

void PointCloudVisualizer::drawGrid() {
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i) {
        glVertex3f(i, -10, 0); glVertex3f(i, 10, 0);
        glVertex3f(-10, i, 0); glVertex3f(10, i, 0);
    }
    glEnd();
}

void PointCloudVisualizer::mousePressEvent(QMouseEvent* event) {
    last_mouse_pos_ = event->pos();
}

void PointCloudVisualizer::mouseMoveEvent(QMouseEvent* event) {
    if (event->buttons() & Qt::LeftButton) {
        int dx = event->x() - last_mouse_pos_.x();
        int dy = event->y() - last_mouse_pos_.y();
        yaw_ += dx * 0.5f;
        pitch_ += dy * 0.5f;
        update();
    }
    last_mouse_pos_ = event->pos();
}

void PointCloudVisualizer::wheelEvent(QWheelEvent* event) {
    float delta = event->angleDelta().y() / 120.0f;
    distance_ -= delta * 0.5f;
    if (distance_ < 0.1f) distance_ = 0.1f;
    update();
}
