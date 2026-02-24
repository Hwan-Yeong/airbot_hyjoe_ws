#pragma once

#include <QWidget>
#include <QLabel>
#include <QKeyEvent>
#include <QTimer>
#include <QPainter>
#include <QSet>
#include <functional>
#include <chrono>
#include "sensor_gui/s_curve_profile.hpp"

// ─────────────────────────────────────────────────────────
//  TeleopWindow
//  별도 팝업 창으로, 방향키 / WASD 입력을 받아 cmd_vel에
//  해당하는 (vx, vy, vyaw) 콜백을 20Hz 로 호출합니다.
// ─────────────────────────────────────────────────────────
class TeleopWindow : public QWidget {
    Q_OBJECT

public:
    using VelCallback = std::function<void(float vx, float vy, float vyaw)>;

    explicit TeleopWindow(QWidget* parent = nullptr);

    void setVelCallback(VelCallback cb) { vel_callback_ = cb; }
    void setLinearSpeed(float s)  { linear_speed_  = s; }
    void setAngularSpeed(float s) { angular_speed_ = s; }

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
    void focusOutEvent(QFocusEvent* event) override;

private slots:
    void onTimer();

private:
    void drawArrowKey(QPainter& p, int x, int y, int w, int h,
                      const QString& label, Qt::Key key, bool active);

    VelCallback vel_callback_;
    QTimer*     timer_;
    QSet<int>   pressed_keys_;

    float linear_speed_  = 0.5f;  // m/s
    float angular_speed_ = 60.0f; // deg/s

    airbot::SCurveProfile smoother_vx_;
    airbot::SCurveProfile smoother_vy_;
    airbot::SCurveProfile smoother_vyaw_;

    std::chrono::steady_clock::time_point last_time_;
    bool first_update_ = true;
};
