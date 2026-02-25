#include "sensor_gui/util/s_curve_profile.hpp"

namespace airbot {

void SCurveProfile::setLimits(double max_vel, double max_acc, double max_jerk)
{
    max_v_ = std::abs(max_vel);
    max_a_ = std::abs(max_acc);
    max_j_ = std::abs(max_jerk);
}

void SCurveProfile::reset(double vel, double acc)
{
    v_ = vel;
    a_ = acc;
}

double SCurveProfile::update(double target_vel, double dt)
{
    target_vel = clamp(target_vel, -max_v_, max_v_);

    double dv = target_vel - v_;
    double dir = (dv >= 0.0) ? 1.0 : -1.0;

    double dv_abs = std::abs(dv);

    // ===== Time-optimal preview =====
    // jerk-limited stopping distance in velocity space
    double dv_stop = (a_ * a_) / (2.0 * max_j_);

    bool need_brake = dv_abs <= dv_stop;

    // ===== desired acceleration =====
    double a_target;

    if (need_brake)
        a_target = 0.0;
    else
        a_target = dir * max_a_;

    // ===== jerk limiting =====
    double da = a_target - a_;
    double max_da = max_j_ * dt;
    da = clamp(da, -max_da, max_da);

    a_ += da;
    a_ = clamp(a_, -max_a_, max_a_);

    // ===== integrate velocity =====
    v_ += a_ * dt;

    // ===== overshoot protection =====
    if ((dir > 0.0 && v_ > target_vel) ||
        (dir < 0.0 && v_ < target_vel))
    {
        v_ = target_vel;
        a_ = 0.0;
    }

    return v_;
}

double SCurveProfile::clamp(double v, double min, double max)
{
    return std::max(min, std::min(v, max));
}

} // namespace airbot
