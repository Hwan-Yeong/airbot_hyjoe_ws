//
// Created by changun on 2/23/25.
//

#ifndef WARMUP_SERVER_INCLUDE_DOMAIN_ENTITY_VERTEX_HPP_
#define WARMUP_SERVER_INCLUDE_DOMAIN_ENTITY_VERTEX_HPP_
namespace entity {
      class Vertex {
       private:
        float x_;
        float y_;

       public:
        Vertex() : x_(0),y_(0){
        }
        Vertex(float x, float y) : x_(x), y_(y) {}
        float GetY() const { return y_; }
        void SetY(float y) { y_ = y; }
        float GetX() const { return x_; }
        void SetX(float x) { x_ = x; }
      };
      }
#endif  //WARMUP_SERVER_INCLUDE_DOMAIN_ENTITY_VERTEX_HPP_
