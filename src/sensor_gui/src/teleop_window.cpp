#include "sensor_gui/teleop_window.hpp"
//  Constructor
// ─────────────────────────────────────────────
TeleopWindow::TeleopWindow(QWidget* parent)
    : QWidget(parent)
{
    setWindowTitle("Robot Teleop");
    setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
    setFixedSize(320, 360);
    setFocusPolicy(Qt::StrongFocus);

    // Background style
    setStyleSheet(R"(
        QWidget {
            background-color: #1e1e2e;
            color: #cdd6f4;
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
}

// ─────────────────────────────────────────────
//  Key Events
// ─────────────────────────────────────────────
void TeleopWindow::keyPressEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        pressed_keys_.insert(event->key());
        update();
    }
}

void TeleopWindow::keyReleaseEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        pressed_keys_.remove(event->key());
        update();
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
    update();
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
static void drawKeyButton(QPainter& p, int x, int y, int w, int h,
                          const QString& label, bool active)
{
    // Shadow
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 80));
    p.drawRoundedRect(x + 3, y + 3, w, h, 8, 8);

    // Key body
    QColor base   = active ? QColor("#89b4fa") : QColor("#313244");
    QColor text_c = active ? QColor("#1e1e2e") : QColor("#cdd6f4");
    p.setBrush(base);
    p.setPen(QPen(active ? QColor("#74c7ec") : QColor("#45475a"), 1.5));
    p.drawRoundedRect(x, y, w, h, 8, 8);

    // Label
    p.setPen(text_c);
    QFont f = p.font();
    f.setPixelSize(16);
    f.setBold(true);
    p.setFont(f);
    p.drawText(QRect(x, y, w, h), Qt::AlignCenter, label);
}

void TeleopWindow::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Title
    p.setPen(QColor("#cba6f7"));
    QFont tf = p.font();
    tf.setPixelSize(16);
    tf.setBold(true);
    p.setFont(tf);
    p.drawText(QRect(0, 12, width(), 28), Qt::AlignCenter, "🎮  Robot Teleop Control");

    // Key legend
    p.setPen(QColor("#6c7086"));
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
    drawKeyButton(p, cx + kw + gap, row1_y, kw, kh, "↑  W",  up_active);

    // Row 2: TurnL  STOP  TurnR
    {
        // Custom draw for Emergency Stop (Red)
        int sx = cx + kw + gap, sy = row2_y;
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 0, 0, 80));
        p.drawRoundedRect(sx + 3, sy + 3, kw, kh, 8, 8);

        QColor base = stop_active ? QColor("#f38ba8") : QColor("#450000"); // Reddish
        p.setBrush(base);
        p.setPen(QPen(stop_active ? QColor("#eba0ac") : QColor("#2a0000"), 1.5));
        p.drawRoundedRect(sx, sy, kw, kh, 8, 8);

        p.setPen(stop_active ? QColor("#1e1e2e") : QColor("#f38ba8"));
        p.drawText(QRect(sx, sy, kw, kh), Qt::AlignCenter, "S\nSTOP");
    }

    drawKeyButton(p, cx,                     row2_y, kw, kh, "←  A",  turn_l_active);
    drawKeyButton(p, cx + (kw + gap) * 2,   row2_y, kw, kh, "→  D",  turn_r_active);

    // Row 3: Strafe & Down
    int row3_y = row2_y + kh + gap;
    drawKeyButton(p, cx,                     row3_y, kw, kh, "Q",  strafe_l_active);
    drawKeyButton(p, cx + kw + gap,          row3_y, kw, kh, "↓  X",  down_active);
    drawKeyButton(p, cx + (kw + gap) * 2,   row3_y, kw, kh, "E",  strafe_r_active);

    // Speed readout
    p.setPen(QColor("#a6e3a1"));
    QFont sf = p.font();
    sf.setPixelSize(12);
    p.setFont(sf);
    p.drawText(QRect(0, row3_y + kh + 16, width(), 20), Qt::AlignCenter,
               QString("Linear: %1 m/s   Angular: %2 °/s")
                   .arg(linear_speed_, 0, 'f', 2)
                   .arg(angular_speed_, 0, 'f', 1));

    // Focus hint
    p.setPen(QColor("#6c7086"));
    QFont hf = p.font();
    hf.setPixelSize(11);
    p.setFont(hf);
    p.drawText(QRect(0, height() - 22, width(), 20), Qt::AlignCenter,
               hasFocus() ? "▶  Keys active" : "⚠  Click here to activate");
}
