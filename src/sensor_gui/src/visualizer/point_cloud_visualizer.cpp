#include "sensor_gui/visualizer/point_cloud_visualizer.hpp"
#include "sensor_gui/visualizer/robot_model.hpp"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <cmath>
#include <GL/gl.h>
#include <GL/glu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

PointCloudVisualizer::PointCloudVisualizer(QWidget* parent)
    : QOpenGLWidget(parent) {
    robot_model_ = std::make_unique<RobotModel>();
    
    renderers_[ObstacleType::kBox] = std::make_unique<BoxRenderer>();
    renderers_[ObstacleType::kCylinder] = std::make_unique<CylinderRenderer>();
    renderers_[ObstacleType::kCone] = std::make_unique<ConeRenderer>();
    renderers_[ObstacleType::kSphere] = std::make_unique<SphereRenderer>();
}

PointCloudVisualizer::~PointCloudVisualizer() {}

/**
 * @brief Update point cloud data
 * @param name Sensor name
 * @param frame_id Frame ID
 * @param points Point data (x, y, z, ...)
 * @param color Color for visualization
 */
void PointCloudVisualizer::updateCloud(const std::string& name, const std::string& frame_id, const std::vector<float>& points, QColor color) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    clouds_[name] = {frame_id, points, color};
    update();
}

/**
 * @brief Update TF data
 * @param tfs TF data map
 */
void PointCloudVisualizer::updateTFs(const std::map<std::string, TfData>& tfs) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    // Preserve base_link if we currently have it but it's not in the incoming ROS TF updates
    // This happens because Physics step updates base_link directly at 60Hz and syncTFs skips it.
    bool has_base_link = tfs_.count("base_link");
    TfData base_tf;
    if (has_base_link) {
        base_tf = tfs_["base_link"];
    }
    
    tfs_ = tfs;
    
    if (has_base_link && !tfs_.count("base_link")) {
        tfs_["base_link"] = base_tf;
    }
    
    update();
}

bool PointCloudVisualizer::setRobotModelFromUrdf(const std::string& path) {
    if (robot_model_) {
        return robot_model_->loadFromFile(path);
    }
    return false;
}

void PointCloudVisualizer::setBackgroundColor(const QColor& color) {
    bg_color_ = color;
    update();
}

void PointCloudVisualizer::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(bg_color_.redF(), bg_color_.greenF(), bg_color_.blueF(), 1.0f);
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
    glClearColor(bg_color_.redF(), bg_color_.greenF(), bg_color_.blueF(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera Transformation (Orbit + Target Pan)
    glTranslatef(0, 0, -distance_);
    glRotatef(pitch_, 1, 0, 0);
    glRotatef(yaw_, 0, 0, 1);
    glTranslatef(-pan_x_, -pan_y_, -pan_z_); // Rotate around panned target

    drawAxes();
    drawGrid();

    // Calculate robot alpha
    float robot_alpha = 1.0f;
    if (selected_idx_ >= 0) robot_alpha = 0.5f;

    // Draw Robot using TF relative to base_link
    if (robot_model_) {
        // Draw base link (chassis)
        if (tfs_.count("base_link")) {
            robot_model_->draw(tfs_["base_link"], robot_alpha);
        }
        
        // Draw physical wheels overlay
        drawWheels();
        
        // Draw physical casters overlay
        drawCasters();
    }
    drawObstacles();

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

    auto check_axis = [](float start, float delta, float bmin, float bmax, float& tmin, float& tmax) -> bool {
        if (std::abs(delta) < 1e-6f) {
            if (start < bmin || start > bmax) return false;
        } else {
            float t1 = (bmin - start) / delta;
            float t2 = (bmax - start) / delta;
            if (t1 > t2) std::swap(t1, t2);
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
        }
        return true;
    };

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

        struct ObstacleCache { tf2::Quaternion q_inv; float sx, sy, sz; };
        std::vector<ObstacleCache> ob_caches;
        if (has_tf && !obstacles_.empty()) {
            ob_caches.reserve(obstacles_.size());
            for (const auto& ob : obstacles_) {
                tf2::Quaternion q_inv = tf2::Quaternion(ob.qx, ob.qy, ob.qz, ob.qw).inverse();
                tf2::Vector3 ray_orig(sensor_tf.x - ob.x, sensor_tf.y - ob.y, sensor_tf.z - ob.z);
                ray_orig = tf2::quatRotate(q_inv, ray_orig);
                ob_caches.push_back({q_inv, static_cast<float>(ray_orig.x()), static_cast<float>(ray_orig.y()), static_cast<float>(ray_orig.z())});
            }
        }

        glColor3f(cloud.color.redF(), cloud.color.greenF(), cloud.color.blueF());
        glBegin(GL_POINTS);
        for (size_t i = 0; i + 2 < cloud.points.size(); i += 3) {
            float px = cloud.points[i];
            float py = cloud.points[i+1];
            float pz = cloud.points[i+2];

            if (has_tf && (ground_clipping_ || wall_sim_ || !obstacles_.empty())) {
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

                // Clip against all obstacles
                for (size_t k = 0; k < obstacles_.size(); ++k) {
                    const auto& ob = obstacles_[k];
                    const auto& cache = ob_caches[k];
                    
                    // Ray direction in world frame
                    tf2::Vector3 ray_dir(wx - sensor_tf.x, wy - sensor_tf.y, wz - sensor_tf.z);
                    
                    // Rotate ray direction to obstacle local frame
                    ray_dir = tf2::quatRotate(cache.q_inv, ray_dir);
                    
                    float sx = cache.sx, sy = cache.sy, sz = cache.sz;
                    float dx = ray_dir.x(), dy = ray_dir.y(), dz = ray_dir.z();
                    
                    if (ob.type == ObstacleType::kBox) {
                        float tmin = -1e30f, tmax = 1e30f;
                        if (check_axis(sx, dx, -ob.sx/2, ob.sx/2, tmin, tmax) &&
                            check_axis(sy, dy, -ob.sy/2, ob.sy/2, tmin, tmax) &&
                            check_axis(sz, dz, -ob.sz/2, ob.sz/2, tmin, tmax)) {
                            if (tmin <= tmax && tmax > 0.0f) {
                                float hit_t = (tmin > 0.0f) ? tmin : 0.0f;
                                if (hit_t < t) t = hit_t;
                            }
                        }
                    } else if (ob.type == ObstacleType::kCylinder) {
                        // Ray-Cylinder intersection (Infinite cylinder first)
                        // Ray-Cylinder intersection in local frame
                        float ox = sx;
                        float oy = sy;
                        float R = ob.sx;
                        float A = dx*dx + dy*dy;
                        float B = 2.0f * (dx*ox + dy*oy);
                        float C = ox*ox + oy*oy - R*R;
                        float disc = B*B - 4.0f*A*C;
                        if (disc >= 0 && A > 1e-6f) {
                            float t1 = (-B - std::sqrt(disc)) / (2.0f*A);
                            if (t1 > 0 && t1 < t) {
                                float hit_z = sz + t1 * dz;
                                if (hit_z > -ob.sz/2 && hit_z < ob.sz/2) t = t1;
                            }
                        }
                    } else if (ob.type == ObstacleType::kSphere) {
                        float ox = sx;
                        float oy = sy;
                        float oz = sz;
                        float R = ob.sx;
                        float A = dx*dx + dy*dy + dz*dz;
                        float B = 2.0f * (dx*ox + dy*oy + dz*oz);
                        float C = ox*ox + oy*oy + oz*oz - R*R;
                        float disc = B*B - 4.0f*A*C;
                        if (disc >= 0 && A > 1e-6f) {
                            float t1 = (-B - std::sqrt(disc)) / (2.0f*A);
                            if (t1 > 0 && t1 < t) t = t1;
                        }
                    } else if (ob.type == ObstacleType::kCone) {
                        float H = ob.sz;
                        float R = ob.sx;
                        float k = R / H;
                        float k2 = k * k;
                        float V = H / 2.0f - sz;
                        
                        float A = dx*dx + dy*dy - k2*dz*dz;
                        float B = 2.0f * (sx*dx + sy*dy + k2*V*dz);
                        float C = sx*sx + sy*sy - k2*V*V;
                        
                        float hit_t = 1e30f;
                        
                        // Check side surface
                        if (std::abs(A) > 1e-6f) {
                            float disc = B*B - 4.0f*A*C;
                            if (disc >= 0) {
                                float sq_disc = std::sqrt(disc);
                                float t1 = (-B - sq_disc) / (2.0f*A);
                                float t2 = (-B + sq_disc) / (2.0f*A);
                                
                                if (t1 > 0) {
                                    float hz1 = sz + t1*dz;
                                    if (hz1 >= -H/2.0f && hz1 <= H/2.0f) hit_t = std::min(hit_t, t1);
                                }
                                if (t2 > 0) {
                                    float hz2 = sz + t2*dz;
                                    if (hz2 >= -H/2.0f && hz2 <= H/2.0f) hit_t = std::min(hit_t, t2);
                                }
                            }
                        } else if (std::abs(B) > 1e-6f) {
                            float t1 = -C / B;
                            if (t1 > 0) {
                                float hz1 = sz + t1*dz;
                                if (hz1 >= -H/2.0f && hz1 <= H/2.0f) hit_t = std::min(hit_t, t1);
                            }
                        }
                        
                        // Check base cap at z = -H/2
                        if (std::abs(dz) > 1e-6f) {
                            float t_base = (-H/2.0f - sz) / dz;
                            if (t_base > 0 && t_base < hit_t) {
                                float hx = sx + t_base*dx;
                                float hy = sy + t_base*dy;
                                if (hx*hx + hy*hy <= R*R) {
                                    hit_t = std::min(hit_t, t_base);
                                }
                            }
                        }
                        
                        if (hit_t > 0 && hit_t < t) {
                            t = hit_t;
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
    float bx = 0, by = 0;
    auto it = tfs_.find("base_link");
    if (it != tfs_.end()) {
        bx = it->second.x;
        by = it->second.y;
    }

    glPushMatrix();
    glTranslatef(bx, by, 0.001f);
    
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

void PointCloudVisualizer::drawWheels() {
    if (!draw_wheels_) return;

    auto drawOneWheel = [this](const TfData& tf) {
        glPushMatrix();
        glTranslatef(tf.x, tf.y, tf.z);
        // Apply quaternion rotation
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
        mat[12] = 0.0f; mat[13] = 0.0f; mat[14] = 0.0f; mat[15] = 1.0f;
        glMultMatrixf(mat);

        float radius = 0.045f;
        float width = 0.04f;
        
        // Solid black wheel cylinder
        glColor3f(0.2f, 0.2f, 0.2f);
        int slices = 16;
        float halfLen = width / 2.0f;
        glBegin(GL_TRIANGLE_STRIP);
        for (int i = 0; i <= slices; ++i) {
            float angle = i * 2.0f * M_PI / slices;
            float cx = std::cos(angle) * radius;
            float cz = std::sin(angle) * radius;
            // cylinder aligned along Y axis
            glVertex3f(cx, -halfLen, cz);
            glVertex3f(cx, halfLen, cz);
        }
        glEnd();
        // End caps
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, halfLen, 0);
        for (int i = 0; i <= slices; ++i) {
            float angle = i * 2.0f * M_PI / slices;
            glVertex3f(std::cos(angle) * radius, halfLen, std::sin(angle) * radius);
        }
        glEnd();
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, -halfLen, 0);
        for (int i = 0; i <= slices; ++i) {
            float angle = i * 2.0f * M_PI / slices;
            glVertex3f(std::cos(angle) * radius, -halfLen, -std::sin(angle) * radius);
        }
        glEnd();

        // White line indicating rotation along the diameter
        glColor3f(1.0f, 1.0f, 1.0f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        glVertex3f(-radius, halfLen + 0.001f, 0);
        glVertex3f(radius, halfLen + 0.001f, 0);
        glVertex3f(-radius, -halfLen - 0.001f, 0);
        glVertex3f(radius, -halfLen - 0.001f, 0);
        glEnd();

        glPopMatrix();
    };

    drawOneWheel(left_wheel_);
    drawOneWheel(right_wheel_);
}

void PointCloudVisualizer::drawCasters() {
    if (!draw_casters_) return;

    auto drawOneCaster = [this](const TfData& tf) {
        glPushMatrix();
        glTranslatef(tf.x, tf.y, tf.z);
        // Apply quaternion rotation
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
        mat[12] = 0.0f; mat[13] = 0.0f; mat[14] = 0.0f; mat[15] = 1.0f;
        glMultMatrixf(mat);

        float radius = 0.015f;
        
        // Solid white caster sphere
        glColor3f(1.0f, 1.0f, 1.0f);
        GLUquadric* quad = gluNewQuadric();
        gluSphere(quad, radius, 16, 16);
        
        // Draw the inner piston strut (connecting the caster to the chassis)
        // length = 0.08m, points UP (+Z) into the chassis
        glColor3f(0.6f, 0.6f, 0.6f); // Grey metallic
        gluCylinder(quad, 0.006, 0.006, 0.08, 16, 1);
        
        // Draw a spiral spring around the strut
        glColor3f(0.8f, 0.2f, 0.2f); // Red spring for high visibility
        glLineWidth(3.0f);
        glBegin(GL_LINE_STRIP);
        float spring_radius = 0.012f;
        float spring_length = 0.06f; // Length of the spring
        int coils = 6;
        int segments_per_coil = 20;
        int total_segments = coils * segments_per_coil;
        for (int i = 0; i <= total_segments; ++i) {
            float t = (float)i / total_segments;
            float angle = t * coils * 2.0f * M_PI;
            float cx = std::cos(angle) * spring_radius;
            float cy = std::sin(angle) * spring_radius;
            float cz = t * spring_length + 0.008f; // Start slightly above the sphere
            glVertex3f(cx, cy, cz);
        }
        glEnd();

        gluDeleteQuadric(quad);

        glPopMatrix();
    };

    drawOneCaster(caster_front_);
    drawOneCaster(caster_rear_);
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

void PointCloudVisualizer::drawObstacles() {
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        auto type = obstacles_[i].type;
        if (renderers_.count(type)) {
            renderers_[type]->draw(obstacles_[i], (int)i == selected_idx_);
        }
    }
}

void PointCloudVisualizer::mousePressEvent(QMouseEvent* event) {
    last_mouse_pos_ = event->pos();
    
    if (event->button() == Qt::LeftButton) {
        // Simple hit-test for deselection if Shift is NOT held
        if (!(event->modifiers() & Qt::ShiftModifier)) {
            // If user clicks without shift, and we want to allow deselection:
            // (In a real app, we'd do ray-casting to see if they clicked 'nothing')
            // For now, let's keep it simple: clicking lets you rotate, 
            // but we can provide a way to deselect via double click or a specific area.
        }
    } else if (event->button() == Qt::RightButton) {
        // Right click to deselect
        selected_idx_ = -1;
        emit obstacleSelected(-1);
        update();
    }
}

void PointCloudVisualizer::mouseMoveEvent(QMouseEvent* event) {
    int dx = event->x() - last_mouse_pos_.x();
    int dy = event->y() - last_mouse_pos_.y();

    if (event->buttons() & Qt::LeftButton) {
        // Use Shift for Dragging to avoid blocking Camera Rotation
        if (selected_idx_ >= 0 && selected_idx_ < (int)obstacles_.size()) {
            float rad = yaw_ * M_PI / 180.0f;
            float cos_y = std::cos(rad);
            float sin_y = std::sin(rad);
            
            float pitch_rad = pitch_ * M_PI / 180.0f;
            float cos_p = std::abs(std::cos(pitch_rad));
            if (cos_p < 0.1f) cos_p = 0.1f;
            
            float factor_x = 0.001f * distance_;
            float factor_y = factor_x / cos_p;
            
            float dx_cam = dx * factor_x;
            float dy_cam = -dy * factor_y;
            
            float move_x = cos_y * dx_cam + sin_y * dy_cam;
            float move_y = -sin_y * dx_cam + cos_y * dy_cam;
            
            obstacles_[selected_idx_].x += move_x;
            obstacles_[selected_idx_].y += move_y;
            emit obstacleMoved(selected_idx_, obstacles_[selected_idx_].x, obstacles_[selected_idx_].y);
            update();
        } else {
            // Normal Camera Rotation
            yaw_ += dx * 0.5f;
            pitch_ += dy * 0.5f;
            if (pitch_ > -5.0f) pitch_ = -5.0f;
            if (pitch_ < -89.0f) pitch_ = -89.0f;
            update();
        }
    } else if (event->buttons() & Qt::MiddleButton) {
        float rad = yaw_ * M_PI / 180.0f;
        float cos_y = std::cos(rad);
        float sin_y = std::sin(rad);
        
        float pitch_rad = pitch_ * M_PI / 180.0f;
        float cos_p = std::abs(std::cos(pitch_rad));
        if (cos_p < 0.1f) cos_p = 0.1f;
        
        float factor_x = 0.001f * distance_;
        float factor_y = factor_x / cos_p;
        
        float dx_cam = dx * factor_x;
        float dy_cam = -dy * factor_y;
        
        float move_x = cos_y * dx_cam + sin_y * dy_cam;
        float move_y = -sin_y * dx_cam + cos_y * dy_cam;

        // For pan, camera moves opposite to world movement
        pan_x_ -= move_x;
        pan_y_ -= move_y;
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
