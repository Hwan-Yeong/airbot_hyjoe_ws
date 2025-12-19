#pragma once
#include <cmath>
#include <algorithm>

namespace airbot {

class SCurveProfile
{
public:
    SCurveProfile() = default;

    void setLimits(double max_vel, double max_acc, double max_jerk);
    void reset(double vel = 0.0, double acc = 0.0);

    double update(double target_vel, double dt);

private:
    double v_{0.0};
    double a_{0.0};

    double max_v_{0.0};
    double max_a_{0.0};
    double max_j_{0.0};

    double clamp(double v, double min, double max);
};

} // namespace airbot
