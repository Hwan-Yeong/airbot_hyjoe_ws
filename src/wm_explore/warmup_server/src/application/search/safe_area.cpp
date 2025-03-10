//
// Created by changun on 2/23/25.
//

#include "application/search/safe_area.hpp"
#include <cmath>
#include <functional>

#include "domain/ros_define.hpp"
#include "rcutils/logging_macros.h"

#define SCAN_FILTER_DIST 0.5
#define SCAN_MIN_DIST 0.001

search::SafeArea::SafeArea() {}
bool search::SafeArea::area_check(
    double point_x,
    double point_y,
    int *area_number,std::vector<entity::Vertex> obs_area_vector) {
      int num_crosses=0;
      const int area_num=4;
      for(int lp_area=0;lp_area<6;lp_area++){
        for( int lp=0;lp < area_num;lp++){
          int lp_y = (lp+1)%area_num;
          if((obs_area_vector[lp_area*4+lp].GetY() > point_y) != (obs_area_vector[lp_area*4+lp_y].GetY() > point_y)){
            double atX=((obs_area_vector[lp_area*4+lp_y].GetX())- (obs_area_vector[lp_area*4+lp].GetX()))
                             * (point_y - (obs_area_vector[lp_area*4+lp].GetY()))
                             / ((obs_area_vector[lp_area*4+lp_y].GetY()) - (obs_area_vector[lp_area*4+lp].GetY()))
                         + (obs_area_vector[lp_area*4+lp].GetX());
            if( point_x < atX ){
              num_crosses++;
            }
          }
        }
        if(num_crosses%2 > 0)
        {
          *area_number=lp_area;
          return true;
        }
        num_crosses=0;
      }
      return false;
}
std::vector<entity::Vertex> search::SafeArea::setting_area(int area_num) {
      std::vector<entity::Vertex> obs_area_vector;
      //tmp value
      float area_start_x, area_start_y =0.0;
      // 영역 가로
      float half_area_width=0.25;
      // 영역 세로
      float a1_area_height=1.0;
      // 영역 오프셋
      float front_offset=0.3;

      float area_x1, area_x2, area_y1, area_y2,area_x3,area_y3,area_x4,area_y4 = 0.0;
      area_start_x=front_offset*cos(area_num*M_PI/3);
      area_start_y=front_offset*sin(area_num*M_PI/3);

      area_x1=area_start_x+half_area_width*cos(area_num*M_PI/3+90*M_PI/180);
      area_y1=area_start_y+half_area_width*sin(area_num*M_PI/3+90*M_PI/180);

      area_x2=area_x1+a1_area_height*cos(area_num*M_PI/3);
      area_y2=area_y1+a1_area_height*sin(area_num*M_PI/3);

      area_x3=area_start_x+half_area_width*cos(area_num*M_PI/3-90*M_PI/180);
      area_y3=area_start_y+half_area_width*sin(area_num*M_PI/3-90*M_PI/180);

      area_x4=area_x3+a1_area_height*cos(area_num*M_PI/3);
      area_y4=area_y3+a1_area_height*sin(area_num*M_PI/3);

      obs_area_vector.push_back(entity::Vertex(area_x1,area_y1));
      obs_area_vector.push_back(entity::Vertex(area_x2,area_y2));
      obs_area_vector.push_back(entity::Vertex(area_x3,area_y3));
      obs_area_vector.push_back(entity::Vertex(area_x4,area_y4));

      return obs_area_vector;
}
entity::Vertex search::SafeArea::search_goal(
    const std::shared_ptr<sensor_msgs::msg::LaserScan> scan) {

  std::vector<entity::Vertex> obs_area_vector;
  
  // 영역 각도
  double global_angle=-90*M_PI/180;

  //RCLCPP_INFO(this->get_logger(),"LINE=%d",__LINE__);

  for(int area_lp=0;area_lp<6;area_lp++){
    std::vector<entity::Vertex> setting_area_vector;
    setting_area_vector=setting_area(area_lp);

    std::function<entity::Vertex(float,float)> createVertex =[](float x ,float y)->entity::Vertex{ return entity::Vertex(x,y); };
    obs_area_vector.push_back(createVertex(setting_area_vector[0].GetX(),setting_area_vector[0].GetY()));
    obs_area_vector.push_back(createVertex(setting_area_vector[1].GetX(),setting_area_vector[1].GetY()));
    obs_area_vector.push_back(createVertex(setting_area_vector[3].GetX(),setting_area_vector[3].GetY()));
    obs_area_vector.push_back(createVertex(setting_area_vector[2].GetX(),setting_area_vector[2].GetY()));
    RCUTILS_LOG_INFO_NAMED(NODE_NAME,"%s:%d: AREA%d= \n(%f,%f) \n(%f,%f) \n(%f,%f) \n(%f,%f)\n",__func__, __LINE__,area_lp,setting_area_vector[0].GetX(),setting_area_vector[0].GetY() ,setting_area_vector[1].GetX() ,setting_area_vector[1].GetY() ,setting_area_vector[2].GetX() ,setting_area_vector[2].GetY() ,setting_area_vector[3].GetX() ,setting_area_vector[3].GetY() );
  }

  int num_ranges = scan->ranges.size();
  double occ_x,occ_y = 0;

  std::map<int,bool> area_check_map;
  for(int lp=1;lp<num_ranges;lp++)
  {
    if((!std::isnan(scan->ranges[lp])) && (scan->ranges[lp] > SCAN_MIN_DIST && ((scan->ranges[lp]-scan->ranges[lp-1]) < SCAN_FILTER_DIST )))
    {
      occ_x=-scan->ranges[lp]*sin( double(scan->angle_min+scan->angle_increment*lp+global_angle) );
      occ_y=scan->ranges[lp]*cos( double(scan->angle_min+scan->angle_increment*lp+global_angle) );
      int area_check_flag=0;
      bool result_flag = area_check(occ_x,occ_y,&area_check_flag,obs_area_vector);
      if(result_flag) {
        RCUTILS_LOG_INFO_NAMED(
            NODE_NAME, "%s:%d: obstacle dot : %d]\n(%lf, %lf)", __func__,
            __LINE__, area_check_flag, occ_x, occ_y);
        area_check_map.insert(std::pair<int,bool>(area_check_flag,true));
      }
    }
  }

  for (int i = 0; i <= 5; i++) {
    if (area_check_map.find(i) == area_check_map.end()) {
      area_check_map.insert(std::make_pair(i, false));
    }
  }


  search::Priority priority = get_highest_priority(area_check_map);
  entity::Vertex center = get_area_center(
      obs_area_vector[static_cast<int>(priority)*4],
      obs_area_vector[(static_cast<int>(priority)*4)+1],
      obs_area_vector[(static_cast<int>(priority)*4)+2],
      obs_area_vector[(static_cast<int>(priority)*4)+3]);
  RCUTILS_LOG_INFO_NAMED(NODE_NAME,"%s:%d: center x :%lf, center y :%lf priority %d",__func__,__LINE__,center.GetX(),center.GetY(),static_cast<int>(priority));
  return center;
}
search::Priority search::SafeArea::get_highest_priority(
    const std::map<int, bool>& area_check_map) {
    static const int priority_order[] = {
        static_cast<int>(search::Priority::kFront),
        static_cast<int>(search::Priority::kLeftFront),
        static_cast<int>(search::Priority::kRightFront),
        static_cast<int>(search::Priority::kLeftRear),
        static_cast<int>(search::Priority::kRightRear),
        static_cast<int>(search::Priority::kRear)
    };
   // 0 ,1 ,5 ,2, 4, 3
      for (int key : priority_order) {
        auto it = area_check_map.find(key);
        if (it != area_check_map.end() && it->second != true) {
          return static_cast<search::Priority>(key);
        }
      }



  return search::Priority::kNone;
}
entity::Vertex search::SafeArea::get_area_center(entity::Vertex dot_one,
                                                 entity::Vertex dot_two,
                                                 entity::Vertex dot_three,
                                                 entity::Vertex dot_four) {


  auto x = (dot_two.GetX() + dot_four.GetX())/2;
  RCUTILS_LOG_INFO_NAMED(NODE_NAME,"%s:%d: one x :%lf, two x :%lf three x : %lf four : %lf",__func__,__LINE__,dot_one.GetX(),dot_two.GetX(),dot_three.GetX(),dot_four.GetX());
 return  entity::Vertex(   x
                              ,(dot_one.GetY() + dot_two.GetY() + dot_three.GetY() + dot_four.GetY())/4);
}

