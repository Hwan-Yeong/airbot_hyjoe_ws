#include "sensor_gui/teleop_window.hpp"
#include "ui_teleop_window.h"
//  Constructor
// ─────────────────────────────────────────────
TeleopWindow::TeleopWindow(QWidget* parent)
    : QWidget(parent), ui(new Ui::TeleopWindow)
{
    ui->setupUi(this);
    setWindowTitle("Robot Teleop");
    setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
    setFixedSize(320, 480); // Increased height for speed controls
    setFocusPolicy(Qt::StrongFocus);

    // Background style
    setStyleSheet(R"(
        QWidget {
            background-color: #FDFBF7;
            color: #2C2C2C;
        }
        QGroupBox {
            border: 1px solid #D1D1D1;
            border-radius: 8px;
            margin-top: 10px;
            font-weight: bold;
            color: #4A4A4A;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px 0 3px;
        }
        QLabel {
            color: #4A4A4A;
        }
        QDoubleSpinBox {
            background-color: #FFFFFF;
            color: #2C2C2C;
            border: 1px solid #D1D1D1;
            border-radius: 4px;
            padding: 2px;
        }
    )");

    // Timer: 20Hz velocity publish
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &TeleopWindow::onTimer);
    timer_->start(50);

    // Initialize smoothers with default limits derived from airbot_teleop_velocity_smoother
    smoother_vx_.setLimits(linear_speed_, 0.6, 2.0);
    smoother_vy_.setLimits(linear_speed_, 0.6, 2.0);
    // Angular limits converted from rad to deg (45 deg/s, 143.24 deg/s^2, 458.37 deg/s^3)
    smoother_vyaw_.setLimits(angular_speed_, 143.24, 458.37);

    ui->spin_linear_speed_->setValue(linear_speed_);
    ui->spin_angular_speed_->setValue(angular_speed_);

    connect(ui->spin_linear_speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double val){
        linear_speed_ = static_cast<float>(val);
        // smoother limit takes place in onTimer
    });
    connect(ui->spin_angular_speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double val){
        angular_speed_ = static_cast<float>(val);
    });
}

TeleopWindow::~TeleopWindow() = default;


// ─────────────────────────────────────────────
//  Key Events
// ─────────────────────────────────────────────
void TeleopWindow::keyPressEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        pressed_keys_.insert(event->key());
        ui->key_area_->updateState(pressed_keys_, is_dark_);
    }
}

void TeleopWindow::keyReleaseEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        pressed_keys_.remove(event->key());
        ui->key_area_->updateState(pressed_keys_, is_dark_);
    }
}

void TeleopWindow::focusOutEvent(QFocusEvent* event) {
    // 창이 포커스를 잃으면 모든 키 해제 (안전 정지)
    pressed_keys_.clear();
    if (vel_callback_) {
        vel_callback_(0.0f, 0.0f, 0.0f);
        smoother_vx_.reset();
        smoother_vy_.reset();
        smoother_vyaw_.reset();
        first_update_ = true;
    }
    ui->key_area_->updateState(pressed_keys_, is_dark_);
    QWidget::focusOutEvent(event);
}

void TeleopWindow::closeEvent(QCloseEvent* event) {
    // 창 닫을 때 로봇 정지
    pressed_keys_.clear();
    if (vel_callback_) {
        vel_callback_(0.0f, 0.0f, 0.0f);
        smoother_vx_.reset();
        smoother_vy_.reset();
        smoother_vyaw_.reset();
        first_update_ = true;
    }
    QWidget::closeEvent(event);
}

// ─────────────────────────────────────────────
//  Timer → VelCallback
// ─────────────────────────────────────────────
void TeleopWindow::onTimer() {
    if (!vel_callback_) return;

    float target_vx = 0.0f, target_vy = 0.0f, target_vyaw = 0.0f;
    bool emergency_stop = pressed_keys_.contains(Qt::Key_S);

    if (emergency_stop) {
        // Emergency stop: target 0 and reset smoothers immediately
        target_vx = 0.0f;
        target_vy = 0.0f;
        target_vyaw = 0.0f;
        smoother_vx_.reset(0.0, 0.0);
        smoother_vy_.reset(0.0, 0.0);
        smoother_vyaw_.reset(0.0, 0.0);
    } else {
        // Forward/Backward
        if (pressed_keys_.contains(Qt::Key_Up)    || pressed_keys_.contains(Qt::Key_W)) target_vx +=  linear_speed_;
        if (pressed_keys_.contains(Qt::Key_Down)  || pressed_keys_.contains(Qt::Key_X)) target_vx -=  linear_speed_;
        
        // Strafe
        if (pressed_keys_.contains(Qt::Key_Q)) target_vy +=  linear_speed_;
        if (pressed_keys_.contains(Qt::Key_E)) target_vy -=  linear_speed_;
        
        // Turn
        if (pressed_keys_.contains(Qt::Key_Left)  || pressed_keys_.contains(Qt::Key_A)) target_vyaw += angular_speed_;
        if (pressed_keys_.contains(Qt::Key_Right) || pressed_keys_.contains(Qt::Key_D)) target_vyaw -= angular_speed_;
    }

    // Calculate dt
    auto now = std::chrono::steady_clock::now();
    double dt = 0.05; // Default dt (20Hz)
    
    if (first_update_) {
        first_update_ = false;
        smoother_vx_.reset(0.0, 0.0);
        smoother_vy_.reset(0.0, 0.0);
        smoother_vyaw_.reset(0.0, 0.0);
    } else {
        dt = std::chrono::duration<double>(now - last_time_).count();
    }
    last_time_ = now;

    // Apply S-Curve Smoothing
    // We update limits in case they changed via setLinearSpeed/setAngularSpeed
    smoother_vx_.setLimits(linear_speed_, 0.6, 2.0);
    smoother_vy_.setLimits(linear_speed_, 0.6, 2.0);
    smoother_vyaw_.setLimits(angular_speed_, 143.24, 458.37);

    float smoothed_vx = static_cast<float>(smoother_vx_.update(target_vx, dt));
    float smoothed_vy = static_cast<float>(smoother_vy_.update(target_vy, dt));
    float smoothed_vyaw = static_cast<float>(smoother_vyaw_.update(target_vyaw, dt));

    vel_callback_(smoothed_vx, smoothed_vy, smoothed_vyaw);
}

// ─────────────────────────────────────────────
//  Paint: arrow key layout
// ─────────────────────────────────────────────
void TeleopKeyArea::drawKeyButton(QPainter& p, int x, int y, int w, int h,
                          const QString& label, bool active, bool is_dark)
{
    // Shadow
    p.setPen(Qt::NoPen);
    p.setBrush(is_dark ? QColor(0, 0, 0, 80) : QColor(0, 0, 0, 30));
    p.drawRoundedRect(x + (is_dark ? 3 : 2), y + (is_dark ? 3 : 2), w, h, 8, 8);

    // Key body
    QColor base, text_c, border;
    if (is_dark) {
        base   = active ? QColor("#89b4fa") : QColor("#313244");
        text_c = active ? QColor("#1e1e2e") : QColor("#cdd6f4");
        border = active ? QColor("#74c7ec") : QColor("#45475a");
    } else {
        base   = active ? QColor("#AED9FF") : QColor("#FFFFFF");
        text_c = active ? QColor("#004A99") : QColor("#2C2C2C");
        border = active ? QColor("#7BB8FF") : QColor("#D1D1D1");
    }

    p.setBrush(base);
    p.setPen(QPen(border, 1.5));
    p.drawRoundedRect(x, y, w, h, 8, 8);

    // Label
    p.setPen(text_c);
    QFont f = p.font();
    f.setPixelSize(16);
    f.setBold(true);
    p.setFont(f);
    p.drawText(QRect(x, y, w, h), Qt::AlignCenter, label);
}

void TeleopKeyArea::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Title
    p.setPen(is_dark_ ? QColor("#cba6f7") : QColor("#4A4A4A"));
    QFont tf = p.font();
    tf.setPixelSize(16);
    tf.setBold(true);
    p.setFont(tf);
    p.drawText(QRect(0, 12, width(), 28), Qt::AlignCenter, "🎮  Robot Teleop Control");

    // Key legend
    p.setPen(is_dark_ ? QColor("#6c7086") : QColor("#7F7F7F"));
    QFont lf = p.font();
    lf.setPixelSize(11);
    lf.setBold(false);
    p.setFont(lf);
    p.drawText(QRect(0, 42, width(), 18), Qt::AlignCenter, "↑↓/WX = Fwd/Back   AD = Turn   QE = Strafe   S = STOP");

    // ── Arrow key positions ──────────────────────
    int kw = 60, kh = 52, gap = 8;
    int row1_y = 90;
    int row2_y = row1_y + kh + gap;
    int cx = (width() - kw * 3 - gap * 2) / 2;

    // Input monitoring
    bool up_active    = pressed_keys_.contains(Qt::Key_Up)    || pressed_keys_.contains(Qt::Key_W);
    bool down_active  = pressed_keys_.contains(Qt::Key_Down)  || pressed_keys_.contains(Qt::Key_X);
    bool turn_l_active = pressed_keys_.contains(Qt::Key_Left) || pressed_keys_.contains(Qt::Key_A);
    bool turn_r_active = pressed_keys_.contains(Qt::Key_Right)|| pressed_keys_.contains(Qt::Key_D);
    bool strafe_l_active = pressed_keys_.contains(Qt::Key_Q);
    bool strafe_r_active = pressed_keys_.contains(Qt::Key_E);
    bool stop_active  = pressed_keys_.contains(Qt::Key_S);

    // Row 1: UP
    drawKeyButton(p, cx + kw + gap, row1_y, kw, kh, "↑  W",  up_active, is_dark_);

    // Row 2: TurnL  STOP  TurnR
    {
        // Custom draw for Emergency Stop (Red)
        int sx = cx + kw + gap, sy = row2_y;
        p.setPen(Qt::NoPen);
        p.setBrush(is_dark_ ? QColor(0, 0, 0, 80) : QColor(0, 0, 0, 30));
        p.drawRoundedRect(sx + (is_dark_ ? 3 : 2), sy + (is_dark_ ? 3 : 2), kw, kh, 8, 8);

        QColor base, border, text_c;
        if (is_dark_) {
            base = stop_active ? QColor("#f38ba8") : QColor("#450000");
            border = stop_active ? QColor("#eba0ac") : QColor("#2a0000");
            text_c = stop_active ? QColor("#1e1e2e") : QColor("#f38ba8");
        } else {
            base = stop_active ? QColor("#FF6B6B") : QColor("#FFE5E5");
            border = stop_active ? QColor("#FA5252") : QColor("#FFA8A8");
            text_c = stop_active ? QColor("#FFFFFF") : QColor("#C92A2A");
        }

        p.setBrush(base);
        p.setPen(QPen(border, 1.5));
        p.drawRoundedRect(sx, sy, kw, kh, 8, 8);

        p.setPen(text_c);
        p.drawText(QRect(sx, sy, kw, kh), Qt::AlignCenter, "S\nSTOP");
    }

    drawKeyButton(p, cx,                     row2_y, kw, kh, "←  A",  turn_l_active, is_dark_);
    drawKeyButton(p, cx + (kw + gap) * 2,   row2_y, kw, kh, "→  D",  turn_r_active, is_dark_);

    // Row 3: Strafe & Down
    int row3_y = row2_y + kh + gap;
    drawKeyButton(p, cx,                     row3_y, kw, kh, "Q",  strafe_l_active, is_dark_);
    drawKeyButton(p, cx + kw + gap,          row3_y, kw, kh, "↓  X",  down_active, is_dark_);
    drawKeyButton(p, cx + (kw + gap) * 2,   row3_y, kw, kh, "E",  strafe_r_active, is_dark_);

    // Focus hint
    p.setPen(is_dark_ ? QColor("#6c7086") : QColor("#7F7F7F"));
    QFont hf = p.font();
    hf.setPixelSize(11);
    p.setFont(hf);
    p.drawText(QRect(0, height() - 22, width(), 20), Qt::AlignCenter,
               hasFocus() ? "▶  Keys active" : "⚠  Click here to activate");
}

void TeleopWindow::setTheme(bool is_dark) {
    is_dark_ = is_dark;
    if (is_dark) {
        setStyleSheet(R"(
            QWidget { background-color: #1e1e2e; color: #cdd6f4; }
            QGroupBox { border: 1px solid #45475a; border-radius: 8px; margin-top: 10px; font-weight: bold; color: #f5c2e7; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
            QLabel { color: #bac2de; }
            QDoubleSpinBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 2px; }
        )");
    } else {
        setStyleSheet(R"(
            QWidget { background-color: #FDFBF7; color: #2C2C2C; }
            QGroupBox { border: 1px solid #D1D1D1; border-radius: 8px; margin-top: 10px; font-weight: bold; color: #4A4A4A; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
            QLabel { color: #4A4A4A; }
            QDoubleSpinBox { background-color: #FFFFFF; color: #2C2C2C; border: 1px solid #D1D1D1; border-radius: 4px; padding: 2px; }
        )");
    }
    ui->key_area_->updateState(pressed_keys_, is_dark_);
}

void TeleopKeyArea::updateState(const QSet<int>& pressed, bool is_dark) {
    pressed_keys_ = pressed;
    is_dark_ = is_dark;
    update();
}
