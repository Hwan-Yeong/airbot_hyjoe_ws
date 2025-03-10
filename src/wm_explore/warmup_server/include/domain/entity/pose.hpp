//
// Created by changun on 2/25/25.
//

#ifndef WARMUP_SERVER_INCLUDE_DOMAIN_ENTITY_POSE_HPP_
#define WARMUP_SERVER_INCLUDE_DOMAIN_ENTITY_POSE_HPP_

namespace entity{
    class Pose{
     private :
          double x_;
          double y_;
          double theta_;

         public:
          double GetX() const { return x_; }

          double GetY() const { return y_; }

          double GetTheta() const { return theta_; }

          void SetX(double x) { x_ = x; }
          void SetY(double y) { y_ = y; }
          void SetTheta(double theta) { theta_ = theta; }
          explicit Pose(double x, double y, double theta){
              this->x_= x;
              this->y_= y;
              this->theta_ = theta;
          }

    };

}

#endif  //WARMUP_SERVER_INCLUDE_DOMAIN_ENTITY_POSE_HPP_
