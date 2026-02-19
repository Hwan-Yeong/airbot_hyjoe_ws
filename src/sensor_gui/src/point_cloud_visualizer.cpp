#include "sensor_gui/point_cloud_visualizer.hpp"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <cmath>
#include <GL/gl.h>
#include <GL/glu.h>

PointCloudVisualizer::PointCloudVisualizer(QWidget* parent)
    : QOpenGLWidget(parent) {}

void PointCloudVisualizer::updateCloud(const std::string& name, const std::vector<float>& points, QColor color) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (clouds_.find(name) == clouds_.end()) {
        printf("[INFO] PointCloudVisualizer: New source %s (%zu points)\n", name.c_str(), points.size()/3);
    }
    clouds_[name] = {points, color};
    update(); // Request repaint
}

void PointCloudVisualizer::updateTFs(const std::map<std::string, TfData>& tfs) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    tfs_ = tfs;
    update();
}

void PointCloudVisualizer::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f); // Darker blueish background
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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

    // Camera Transformation (Orbit + Screen-Relative Pan)
    glTranslatef(-pan_x_, pan_z_, 0); // Screen-X and Screen-Y (mapped to pan_z for convenience)
    glTranslatef(0, 0, -distance_);
    glRotatef(pitch_, 1, 0, 0);
    glRotatef(yaw_, 0, 0, 1);
    // Removed world-space pan

    drawGrid();
    drawAxes(); // Main Origin Axes

    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // Draw TFs
    for (auto const& [frame_id, tf] : tfs_) {
        glPushMatrix();
        glTranslatef(tf.x, tf.y, tf.z);
        
        // Draw small axis with scaling
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.1f * tf_scale_, 0, 0);
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0.1f * tf_scale_, 0);
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.1f * tf_scale_);
        glEnd();
        glPopMatrix();
    }

    // Draw Clouds
    glPointSize(10.0f);
    for (auto const& [name, cloud] : clouds_) {
        glColor3f(cloud.color.redF(), cloud.color.greenF(), cloud.color.blueF());
        glBegin(GL_POINTS);
        for (size_t i = 0; i + 2 < cloud.points.size(); i += 3) {
            glVertex3f(cloud.points[i], cloud.points[i+1], cloud.points[i+2]);
        }
        glEnd();
    }
}

void PointCloudVisualizer::paintEvent(QPaintEvent* event) {
    // 1. Render OpenGL content
    makeCurrent();
    paintGL();

    // 2. Render Text Overlays using QPainter
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 10, QFont::Bold));

    // Get current matrices to project 3D to 2D
    GLdouble model[16], proj[16];
    GLint view[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetIntegerv(GL_VIEWPORT, view);

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (auto const& [frame_id, tf] : tfs_) {
        GLdouble winX, winY, winZ;
        if (gluProject(tf.x, tf.y, tf.z, model, proj, view, &winX, &winY, &winZ)) {
            // winY is inverted in OpenGL vs Qt
            if (winZ < 1.0) { // Only draw if in front of camera
                QString label = QString::fromStdString(tf.displayName.empty() ? tf.frame_id : tf.displayName);
                painter.drawText(static_cast<int>(winX) + 5, height() - static_cast<int>(winY) - 5, label);
            }
        }
    }
    painter.end();
}

void PointCloudVisualizer::drawAxes() {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X - Red
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.5, 0, 0);
    // Y - Green
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0.5, 0);
    // Z - Blue
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.5);
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
    int dx = event->x() - last_mouse_pos_.x();
    int dy = event->y() - last_mouse_pos_.y();

    if (event->buttons() & Qt::LeftButton) {
        yaw_ += dx * 0.5f;
        pitch_ += dy * 0.5f;
        update();
    } else if (event->buttons() & Qt::MiddleButton) {
        // Direct Screen-Relative Panning
        float factor = 0.005f * distance_;
        pan_x_ -= dx * factor;
        pan_z_ -= dy * factor; // Reusing pan_z variable for screen-Y
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
