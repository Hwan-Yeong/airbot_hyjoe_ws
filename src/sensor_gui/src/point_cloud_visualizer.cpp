#include "sensor_gui/point_cloud_visualizer.hpp"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <cmath>
#include <GL/gl.h>
#include <GL/glu.h>

PointCloudVisualizer::PointCloudVisualizer(QWidget* parent)
    : QOpenGLWidget(parent) {}

void PointCloudVisualizer::updateCloud(const std::string& name, const std::string& frame_id, const std::vector<float>& points, QColor color) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    clouds_[name] = {frame_id, points, color};
    update();
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

    // Camera Transformation (Orbit + Target Pan)
    glTranslatef(0, 0, -distance_);
    glRotatef(pitch_, 1, 0, 0);
    glRotatef(yaw_, 0, 0, 1);
    glTranslatef(-pan_x_, -pan_y_, -pan_z_); // Rotate around panned target

    drawGrid();
    drawAxes(); // Main Origin Axes

    std::lock_guard<std::mutex> lock(cloud_mutex_);

    drawGrid();
    drawRobotFootprint();
    drawWalls();

    // Draw TFs
    for (auto const& [frame_id, tf] : tfs_) {
        glPushMatrix();
        applyTf(tf);
        
        // Draw axis with scaling
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glColor3f(1, 0.2f, 0.2f); glVertex3f(0, 0, 0); glVertex3f(0.1f * tf_scale_, 0, 0);
        glColor3f(0.2f, 1, 0.2f); glVertex3f(0, 0, 0); glVertex3f(0, 0.1f * tf_scale_, 0);
        glColor3f(0.2f, 0.2f, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.1f * tf_scale_);
        glEnd();
        glPopMatrix();
    }

    // Draw Clouds
    glPointSize(5.0f);
    for (auto const& [name, cloud] : clouds_) {
        glPushMatrix();
        // Transform cloud from local to world
        auto it = tfs_.find(cloud.frame_id);
        TfData sensor_tf;
        bool has_tf = false;
        if (it != tfs_.end()) {
            sensor_tf = it->second;
            applyTf(sensor_tf);
            has_tf = true;
        }

        glColor3f(cloud.color.redF(), cloud.color.greenF(), cloud.color.blueF());
        glBegin(GL_POINTS);
        for (size_t i = 0; i + 2 < cloud.points.size(); i += 3) {
            float px = cloud.points[i];
            float py = cloud.points[i+1];
            float pz = cloud.points[i+2];

            if (has_tf && (ground_clipping_ || wall_sim_ || !walls_.empty())) {
                float wx, wy, wz;
                transformPoint(sensor_tf, px, py, pz, wx, wy, wz);
                
                float t = 1.0f;
                // Clip against ground (z=0)
                if (ground_clipping_) {
                    float sz = sensor_tf.z;
                    float dz = wz - sz;
                    if (std::abs(dz) > 1e-6f) {
                        float t_ground = -sz / dz;
                        if (t_ground > 0.0f && t_ground < t) t = t_ground;
                    }
                }
                
                // Clip against legacy wall (x=wall_x)
                if (wall_sim_) {
                    float sx = sensor_tf.x;
                    float dx = wx - sx;
                    if (std::abs(dx) > 1e-6f) {
                        float t_wall = (wall_x_ - sx) / dx;
                        if (t_wall > 0.0f && t_wall < t) t = t_wall;
                    }
                }

                // Clip against all box walls
                for (const auto& box : walls_) {
                    float sx = sensor_tf.x, sy = sensor_tf.y, sz = sensor_tf.z;
                    float dx = wx - sx, dy = wy - sy, dz = wz - sz;
                    
                    float tmin = -1e30f, tmax = 1e30f;
                    
                    auto check_axis = [&](float start, float delta, float bmin, float bmax) {
                        if (std::abs(delta) < 1e-6f) {
                            if (start < bmin || start > bmax) return false;
                        } else {
                            float t1 = (bmin - start) / delta;
                            float t2 = (bmax - start) / delta;
                            if (t1 > t2) std::swap(t1, t2);
                            tmin = std::max(tmin, t1);
                            tmax = std::min(tmax, t2);
                        }
                        return true;
                    };

                    if (check_axis(sx, dx, box.x - box.sx/2, box.x + box.sx/2) &&
                        check_axis(sy, dy, box.y - box.sy/2, box.y + box.sy/2) &&
                        check_axis(sz, dz, box.z - box.sz/2, box.z + box.sz/2)) {
                        
                        if (tmin <= tmax && tmax > 0.0f) {
                            float hit_t = (tmin > 0.0f) ? tmin : 0.0f;
                            if (hit_t < t) t = hit_t;
                        }
                    }
                }

                px *= t; py *= t; pz *= t;
            }
            glVertex3f(px, py, pz);
        }
        glEnd();
        glPopMatrix();
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
    // World Origin (fixed small size, greyish)
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glColor3f(0.5f, 0.5f, 0.5f);
    glVertex3f(0, 0, 0); glVertex3f(0.1f, 0, 0);
    glVertex3f(0, 0, 0); glVertex3f(0, 0.1f, 0);
    glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.1f);
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

void PointCloudVisualizer::drawRobotFootprint() {
    // Find base_link position/orientation
    float bx = 0, by = 0, bz = 0;
    // We draw relative to world, but base_link pose is already in tfs_
    auto it = tfs_.find("base_link");
    if (it != tfs_.end()) {
        bx = it->second.x;
        by = it->second.y;
        bz = it->second.z;
    }

    glPushMatrix();
    glTranslatef(bx, by, bz + 0.01f); // Slightly above ground
    
    // Skyblue circle
    glColor3f(0.53f, 0.81f, 0.98f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 360; i += 10) {
        float rad = i * M_PI / 180.0f;
        glVertex3f(std::cos(rad) * footprint_radius_, std::sin(rad) * footprint_radius_, 0);
    }
    glEnd();
    glPopMatrix();
}
void PointCloudVisualizer::applyTf(const TfData& tf) {
    glTranslatef(tf.x, tf.y, tf.z);
    
    // Quaternion to Rotation Matrix
    float x = tf.qx, y = tf.qy, z = tf.qz, w = tf.qw;
    float mat[16];
    
    mat[0]  = 1.0f - 2.0f * (y * y + z * z);
    mat[1]  = 2.0f * (x * y + z * w);
    mat[2]  = 2.0f * (x * z - y * w);
    mat[3]  = 0.0f;
    
    mat[4]  = 2.0f * (x * y - z * w);
    mat[5]  = 1.0f - 2.0f * (x * x + z * z);
    mat[6]  = 2.0f * (y * z + x * w);
    mat[7]  = 0.0f;
    
    mat[8]  = 2.0f * (x * z + y * w);
    mat[9]  = 2.0f * (y * z - x * w);
    mat[10] = 1.0f - 2.0f * (x * x + y * y);
    mat[11] = 0.0f;
    
    mat[12] = 0.0f;
    mat[13] = 0.0f;
    mat[14] = 0.0f;
    mat[15] = 1.0f;
    
    glMultMatrixf(mat);
}

void PointCloudVisualizer::transformPoint(const TfData& tf, float lx, float ly, float lz, float& wx, float& wy, float& wz) {
    // Rotation by Quaternion
    float x = tf.qx, y = tf.qy, z = tf.qz, w = tf.qw;
    
    // P_rot = q * P_local * q_inv
    float px = lx, py = ly, pz = lz;
    
    float vx = w*px + y*pz - z*py;
    float vy = w*py + z*px - x*pz;
    float vz = w*pz + x*py - y*px;
    float vw = -x*px - y*py - z*pz;
    
    wx = vx*w + vw*-x + vy*-z - vz*-y + tf.x;
    wy = vy*w + vw*-y + vz*-x - vx*-z + tf.y;
    wz = vz*w + vw*-z + vx*-y - vy*-x + tf.z;
}

void PointCloudVisualizer::drawWalls() {
    if (!wall_sim_ && walls_.empty()) return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Legacy Wall
    if (wall_sim_) {
        glColor4f(0.8f, 0.2f, 0.2f, 0.3f);
        glBegin(GL_QUADS);
        glVertex3f(wall_x_, -5.0f, 0.0f);
        glVertex3f(wall_x_,  5.0f, 0.0f);
        glVertex3f(wall_x_,  5.0f, 5.0f);
        glVertex3f(wall_x_, -5.0f, 5.0f);
        glEnd();
    }

    // Box Walls
    for (const auto& box : walls_) {
        glColor4f(0.5f, 0.5f, 0.8f, 0.4f);
        float x1 = box.x - box.sx/2, x2 = box.x + box.sx/2;
        float y1 = box.y - box.sy/2, y2 = box.y + box.sy/2;
        float z1 = box.z - box.sz/2, z2 = box.z + box.sz/2;

        glBegin(GL_QUADS);
        // Bottom
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x1, y2, z1);
        // Top
        glVertex3f(x1, y1, z2); glVertex3f(x2, y1, z2); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        // Front
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y1, z2); glVertex3f(x1, y1, z2);
        // Back
        glVertex3f(x1, y2, z1); glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        // Left
        glVertex3f(x1, y1, z1); glVertex3f(x1, y2, z1); glVertex3f(x1, y2, z2); glVertex3f(x1, y1, z2);
        // Right
        glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2); glVertex3f(x2, y1, z2);
        glEnd();
        
        // Wireframe for edges
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
        glBegin(GL_QUADS);
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x1, y2, z1);
        glVertex3f(x1, y1, z2); glVertex3f(x2, y1, z2); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glVertex3f(x1, y1, z1); glVertex3f(x2, y1, z1); glVertex3f(x2, y1, z2); glVertex3f(x1, y1, z2);
        glVertex3f(x1, y2, z1); glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2); glVertex3f(x1, y2, z2);
        glVertex3f(x1, y1, z1); glVertex3f(x1, y2, z1); glVertex3f(x1, y2, z2); glVertex3f(x1, y1, z2);
        glVertex3f(x2, y1, z1); glVertex3f(x2, y2, z1); glVertex3f(x2, y2, z2); glVertex3f(x2, y1, z2);
        glEnd();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glDisable(GL_BLEND);
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
        float factor = 0.001f * distance_;
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
