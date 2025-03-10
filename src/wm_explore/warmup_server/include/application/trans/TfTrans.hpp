//
// Created by changun on 2/25/25.
//

#ifndef WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_TF_TRANS_HPP_
#define WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_TF_TRANS_HPP_

#include "tf2/LinearMath/Quaternion.h"

class TfTrans {
 public :
    double quaternion_to_radian(double x,double y, double z, double w);
};

#endif  //WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_TF_TRANS_HPP_
