//
// Created by changun on 2/23/25.
//

#ifndef WARMUP_SERVER_INCLUDE_DOMAIN_SEARCH_DEFINE_HPP_
#define WARMUP_SERVER_INCLUDE_DOMAIN_SEARCH_DEFINE_HPP_
namespace search{
  enum class Priority{
      kFront = 0,
      kLeftFront = 1,
      kLeftRear = 2,
      kRear = 3,
      kRightRear =4,
      kRightFront=5,
      kNone=6
};

}


#endif  //WARMUP_SERVER_INCLUDE_DOMAIN_SEARCH_DEFINE_HPP_
