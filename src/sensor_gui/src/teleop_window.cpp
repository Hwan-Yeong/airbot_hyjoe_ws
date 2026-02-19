#include "sensor_gui/teleop_window.hpp"
#include <QPainter>
#include <QVBoxLayout>
#include <QLabel>
#include <QFontDatabase>

// ─────────────────────────────────────────────
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
    if (vel_callback_) vel_callback_(0.0f, 0.0f, 0.0f);
    update();
    QWidget::focusOutEvent(event);
}

void TeleopWindow::closeEvent(QCloseEvent* event) {
    // 창 닫을 때 로봇 정지
    pressed_keys_.clear();
    if (vel_callback_) vel_callback_(0.0f, 0.0f, 0.0f);
    QWidget::closeEvent(event);
}

// ─────────────────────────────────────────────
//  Timer → VelCallback
// ─────────────────────────────────────────────
void TeleopWindow::onTimer() {
    if (!vel_callback_) return;

    float vx = 0.0f, vy = 0.0f, vyaw = 0.0f;

    if (pressed_keys_.contains(Qt::Key_Up)    || pressed_keys_.contains(Qt::Key_W)) vx +=  linear_speed_;
    if (pressed_keys_.contains(Qt::Key_Down)  || pressed_keys_.contains(Qt::Key_S)) vx -=  linear_speed_;
    if (pressed_keys_.contains(Qt::Key_A))                                           vy +=  linear_speed_;
    if (pressed_keys_.contains(Qt::Key_D))                                           vy -=  linear_speed_;
    if (pressed_keys_.contains(Qt::Key_Left)  || pressed_keys_.contains(Qt::Key_Q)) vyaw += angular_speed_;
    if (pressed_keys_.contains(Qt::Key_Right) || pressed_keys_.contains(Qt::Key_E)) vyaw -= angular_speed_;

    vel_callback_(vx, vy, vyaw);
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
    p.drawText(QRect(0, 42, width(), 18), Qt::AlignCenter, "↑↓ = Forward/Back   ←→ = Turn   A/D = Strafe");

    // ── Arrow key positions ──────────────────────
    int kw = 60, kh = 52, gap = 8;
    int row1_y = 90;
    int row2_y = row1_y + kh + gap;
    int cx = (width() - kw * 3 - gap * 2) / 2;

    // Row 1: only UP (center)
    bool up_active    = pressed_keys_.contains(Qt::Key_Up)    || pressed_keys_.contains(Qt::Key_W);
    bool down_active  = pressed_keys_.contains(Qt::Key_Down)  || pressed_keys_.contains(Qt::Key_S);
    bool left_active  = pressed_keys_.contains(Qt::Key_Left)  || pressed_keys_.contains(Qt::Key_Q);
    bool right_active = pressed_keys_.contains(Qt::Key_Right) || pressed_keys_.contains(Qt::Key_E);
    bool a_active     = pressed_keys_.contains(Qt::Key_A);
    bool d_active     = pressed_keys_.contains(Qt::Key_D);

    drawKeyButton(p, cx + kw + gap,         row1_y, kw, kh, "↑  W",  up_active);

    // Row 2: LEFT  DOWN  RIGHT
    drawKeyButton(p, cx,                     row2_y, kw, kh, "←  Q",  left_active);
    drawKeyButton(p, cx + kw + gap,          row2_y, kw, kh, "↓  S",  down_active);
    drawKeyButton(p, cx + (kw + gap) * 2,   row2_y, kw, kh, "→  E",  right_active);

    // Strafe row
    int row3_y = row2_y + kh + gap + 10;
    int sw = 80;
    int scx = (width() - sw * 2 - gap * 3) / 2;
    drawKeyButton(p, scx,           row3_y, sw, kh, "A  Strafe←", a_active);
    drawKeyButton(p, scx + sw + gap*3, row3_y, sw, kh, "D  Strafe→", d_active);

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
