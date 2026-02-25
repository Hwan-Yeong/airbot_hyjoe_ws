#pragma once

#include <QWidget>
#include <QLabel>
#include <QKeyEvent>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPainter>
#include <QSet>
#include <functional>
#include <chrono>
#include "sensor_gui/util/s_curve_profile.hpp"
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui { class TeleopWindow; }
QT_END_NAMESPACE


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
    ~TeleopWindow();

    void setVelCallback(VelCallback cb) { vel_callback_ = cb; }
    void setLinearSpeed(float s)  { linear_speed_  = s; }
    void setAngularSpeed(float s) { angular_speed_ = s; }
    void setTheme(bool is_dark);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
    void focusOutEvent(QFocusEvent* event) override;

private slots:
    void onTimer();


private:
    std::unique_ptr<Ui::TeleopWindow> ui;

    VelCallback vel_callback_;
    QTimer*     timer_;
    QSet<int>   pressed_keys_;
    bool        is_dark_ = false;

    float linear_speed_  = 0.5f;  // m/s
    float angular_speed_ = 60.0f; // deg/s


    airbot::SCurveProfile smoother_vx_;
    airbot::SCurveProfile smoother_vy_;
    airbot::SCurveProfile smoother_vyaw_;

    std::chrono::steady_clock::time_point last_time_;
    bool first_update_ = true;
};


// ─────────────────────────────────────────────────────────
//  TeleopKeyArea (Custom Widget for paintEvent)
// ─────────────────────────────────────────────────────────
class TeleopKeyArea : public QWidget {
    Q_OBJECT
public:
    explicit TeleopKeyArea(QWidget* parent = nullptr) : QWidget(parent) {}
    void updateState(const QSet<int>& pressed, bool is_dark);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawKeyButton(QPainter& p, int x, int y, int w, int h, const QString& label, bool active, bool is_dark);

    QSet<int> pressed_keys_;
    bool is_dark_ = false;
};
