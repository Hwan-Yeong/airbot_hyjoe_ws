//
// Created by changun on 2/25/25.
//

#include "application/trans/TfTrans.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

double TfTrans::quaternion_to_radian(double x, double y, double z, double w) {
    tf2::Quaternion quaternion(x, y, z, w);
    tf2::Matrix3x3 m(quaternion);
    double tr,tp,ty=0.0;
    m.getRPY(tr,tp,ty);
    return ty;
}
